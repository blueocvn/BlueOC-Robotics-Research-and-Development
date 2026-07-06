"""Scripted Pure-Pursuit/PID expert for the JetRacer staging-dock task.

NOT a learned component -- a hand-written controller used to generate demonstration transitions
for seeding SAC's replay buffer (see ``scripts/expert/collect_demos.py``). It reads the same
body-frame staging-pose error the RL policy observes (``StagingPoseCommand.command``: ``(dx, dy,
dheading)``, x-forward/y-left/heading-error-in-[-pi,pi] -- see that class's docstring) and outputs
actions in the same ``[steer_cmd, drive_cmd] in [-1, 1]`` space as :class:`AckermannAction`, so its
transitions are drop-in compatible with what SAC's replay buffer stores from the policy itself.

Vectorized over all envs (every method takes/returns ``(num_envs, ...)`` tensors) since Isaac Lab
runs many envs in parallel.
"""

from __future__ import annotations

import math

import torch


class PurePursuitExpert:
    """Pure-pursuit-style steering + distance-proportional throttle, with a turn-radius-feasibility
    gate that triggers a reverse pivot when the goal can't be reached by a forward arc.

    Reverse-pivot trigger (Dubins/bicycle-model chord-arc geometry): a circular arc tangent to the
    current heading that connects the vehicle to the goal position has radius
    ``R = dist / (2 * sin(bearing))``, where ``bearing`` is the goal's angle off the forward axis
    in the body frame. That arc is only physically drivable without slipping if
    ``R >= min_turn_radius`` (the vehicle's steering-limited minimum turn radius), i.e.
    ``|sin(bearing)| <= dist / (2 * min_turn_radius)``. As the goal gets close and/or ends up
    off to the side or behind, this condition fails -- no forward arc can reach it -- so the
    controller switches to reverse and re-angles instead, mirroring a real multi-point turn.

    Steer-sign convention: this class assumes ``positive steer_cmd -> positive (CCW) yaw rate``
    when driving forward (standard bicycle-model convention). Whether that matches this specific
    USD articulation's joint-axis sign is a hardware/asset detail, not something this class can
    know -- calibrate it empirically once (see ``scripts/expert/collect_demos.py``'s calibration
    step) and set :attr:`steer_sign` accordingly before using this controller.
    """

    def __init__(
        self,
        num_envs: int,
        device: torch.device | str,
        wheelbase: float = 0.16,
        steer_angle_limit: float = 0.349,
        success_pos_tolerance: float = 0.3,
        success_heading_tolerance: float = 0.175,
        heading_gate_distance: float = 0.6,
        max_drive_cmd: float = 0.6,
        steer_gain: float = 1.6,
        drive_gain: float = 1.0,
        min_gear_hold_steps: int = 15,
    ):
        self.device = device
        # L / tan(steer_limit): the tightest circle the vehicle can drive without wheel slip.
        self.min_turn_radius = wheelbase / math.tan(steer_angle_limit)
        self.success_pos_tolerance = success_pos_tolerance
        self.success_heading_tolerance = success_heading_tolerance
        self.heading_gate_distance = heading_gate_distance
        self.max_drive_cmd = max_drive_cmd
        self.steer_gain = steer_gain
        self.drive_gain = drive_gain
        self.min_gear_hold_steps = min_gear_hold_steps

        # Set from an empirical calibration step (see collect_demos.py) -- +1 if a positive
        # steer_cmd produces positive yaw rate while driving forward, else -1.
        self.steer_sign: float = 1.0

        self.gear = torch.ones(num_envs, device=device)  # +1 forward, -1 reverse
        # Start "past due" for a gear decision so the very first _update_command after a reset
        # can pick forward/reverse immediately instead of being stuck forward for
        # min_gear_hold_steps steps regardless of feasibility.
        self.gear_timer = torch.full((num_envs,), min_gear_hold_steps, dtype=torch.long, device=device)

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self.gear[ids] = 1.0
        self.gear_timer[ids] = self.min_gear_hold_steps

    @torch.no_grad()
    def compute(self, command: torch.Tensor) -> torch.Tensor:
        """``command``: ``(num_envs, 3)`` = ``(dx, dy, dheading)`` in the vehicle body frame.

        Returns actions ``(num_envs, 2)`` = ``[steer_cmd, drive_cmd]`` in ``[-1, 1]``.
        """
        dx, dy, dheading = command[:, 0], command[:, 1], command[:, 2]
        dist = torch.sqrt(dx**2 + dy**2).clamp(min=1e-4)

        bearing_fwd = torch.atan2(dy, dx)
        # Target's bearing as seen in a frame rotated 180 deg (i.e. "facing backward") -- the
        # relevant frame for aiming while reversing. Equivalent to wrap_to_pi(bearing_fwd + pi).
        bearing_rev = torch.atan2(-dy, -dx)

        feasible_fwd = torch.sin(bearing_fwd).abs() <= (dist / (2.0 * self.min_turn_radius)).clamp(max=1.0)

        self.gear_timer += 1
        can_switch = self.gear_timer >= self.min_gear_hold_steps
        switch_to_reverse = (~feasible_fwd) & (self.gear > 0) & can_switch
        switch_to_forward = feasible_fwd & (self.gear < 0) & can_switch
        switched = switch_to_reverse | switch_to_forward
        self.gear = torch.where(switch_to_reverse, torch.full_like(self.gear, -1.0), self.gear)
        self.gear = torch.where(switch_to_forward, torch.ones_like(self.gear), self.gear)
        self.gear_timer = torch.where(switched, torch.zeros_like(self.gear_timer), self.gear_timer)

        in_reverse = self.gear < 0
        active_bearing = torch.where(in_reverse, bearing_rev, bearing_fwd)

        # Near the goal, blend the pure-pursuit position bearing with the final heading error so
        # the car settles onto the exact staging heading, not just the right spot (same proximity
        # gate shape as heading_alignment/loiter_penalty in rewards.py).
        heading_gate = torch.exp(-((dist / self.heading_gate_distance) ** 2))
        steer_target = (1.0 - heading_gate) * active_bearing + heading_gate * dheading

        steer_cmd = (self.steer_sign * self.steer_gain * steer_target).clamp(-1.0, 1.0)

        speed_cmd = (self.drive_gain * dist).clamp(max=self.max_drive_cmd)
        drive_cmd = self.gear * speed_cmd

        # Park: once within the success tolerances, stop completely instead of continuing to
        # creep -- this is the "stop and hold" behavior the trained SAC policy never discovered
        # (see loiter_penalty's docstring in rewards.py).
        parked = (dist < self.success_pos_tolerance) & (dheading.abs() < self.success_heading_tolerance)
        steer_cmd = torch.where(parked, torch.zeros_like(steer_cmd), steer_cmd)
        drive_cmd = torch.where(parked, torch.zeros_like(drive_cmd), drive_cmd)

        return torch.stack([steer_cmd, drive_cmd], dim=1)
