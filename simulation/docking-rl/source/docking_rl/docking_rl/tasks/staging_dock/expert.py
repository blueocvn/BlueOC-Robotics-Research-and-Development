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
        wheelbase: float = 0.166,  # measured: center-to-center front-to-rear axle distance
        steer_angle_limit: float = 0.349,
        success_pos_tolerance: float = 0.3,
        success_heading_tolerance: float = 0.26,
        max_drive_cmd: float = 0.5,
        # STANLEY steering (Stanford DARPA controller) tracking the APPROACH LINE -- the ray through
        # the staging pose along the staging heading. delta = heading_error + atan2(k_cross*e, v+soft)
        # where e is the cross-track distance to that line. Because the line IS the correct final
        # heading, converging onto it makes the car arrive ALIGNED -- fixing the pose controller's
        # failure mode (it reached the goal position but 38 deg off heading, so it never held the
        # position+heading dwell gate). k_rho sets approach speed; stanley_softening avoids the
        # classic low-speed singularity (v in the denominator) as the car slows to a stop.
        k_rho: float = 1.2,
        k_cross: float = 1.5,
        stanley_softening: float = 0.35,
        steer_gain: float = 1.0,
        # --- K-turn (multi-point turn) params ----------------------------------------------------
        # A forward-only controller STALLS mis-aligned near the goal: speed ~ distance, so it crawls
        # to a stop before it can finish rotating (a car can't turn at zero speed). The K-turn fixes
        # this: when the car is close but its heading is off, BACK UP to regain room + re-angle, then
        # drive in aligned.
        kturn_trigger_dist: float = 0.45,  # "close" enough that stalling is the risk
        kturn_reverse_steps: int = 22,  # how long each reverse leg lasts (~0.7 s at 30 Hz)
        kturn_min_heading_err: float = 0.30,  # only K-turn when heading is clearly outside the (loosened) success gate
        min_gear_hold_steps: int = 20,
    ):
        self.device = device
        # L / tan(steer_limit): the tightest circle the vehicle can drive without wheel slip.
        self.min_turn_radius = wheelbase / math.tan(steer_angle_limit)
        self.success_pos_tolerance = success_pos_tolerance
        self.success_heading_tolerance = success_heading_tolerance
        self.max_drive_cmd = max_drive_cmd
        self.k_rho = k_rho
        self.k_cross = k_cross
        self.stanley_softening = stanley_softening
        self.steer_gain = steer_gain
        self.kturn_trigger_dist = kturn_trigger_dist
        self.kturn_reverse_steps = kturn_reverse_steps
        self.kturn_min_heading_err = kturn_min_heading_err
        self.min_gear_hold_steps = min_gear_hold_steps

        # Set from an empirical calibration step (see collect_demos.py) -- +1 if a positive
        # steer_cmd produces positive yaw rate while driving forward, else -1.
        self.steer_sign: float = 1.0

        self.gear = torch.ones(num_envs, device=device)  # +1 forward, -1 reverse
        # Steps remaining in the current reverse (back-up-and-realign) leg of a K-turn; 0 = driving
        # forward toward the goal. Set to kturn_reverse_steps when a mis-aligned stall is detected.
        self.reverse_timer = torch.zeros(num_envs, dtype=torch.long, device=device)

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self.gear[ids] = 1.0
        self.reverse_timer[ids] = 0

    @torch.no_grad()
    def compute(self, command: torch.Tensor) -> torch.Tensor:
        """``command``: ``(num_envs, 3)`` = ``(dx, dy, dheading)`` in the vehicle body frame.

        Returns actions ``(num_envs, 2)`` = ``[steer_cmd, drive_cmd]`` in ``[-1, 1]``.
        """
        def _wrap(a: torch.Tensor) -> torch.Tensor:
            return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi

        dx, dy, dheading = command[:, 0], command[:, 1], command[:, 2]
        dist = torch.sqrt(dx**2 + dy**2).clamp(min=1e-4)
        bearing_fwd = torch.atan2(dy, dx)  # angle to the goal in the body frame (0 = straight ahead)

        parked = (dist < self.success_pos_tolerance) & (dheading.abs() < self.success_heading_tolerance)

        # --- K-TURN STATE MACHINE ----------------------------------------------------------------
        # Decrement any in-progress reverse leg; a car with reverse_timer>0 is mid back-up.
        self.reverse_timer = (self.reverse_timer - 1).clamp(min=0)
        reversing = self.reverse_timer > 0
        # Trigger a new reverse leg when the car has arrived close in POSITION but is still off in
        # HEADING (the forward-only stall) and isn't already reversing or parked. Backing up here
        # regains room + swings the heading so the next forward leg can drive in aligned.
        stalled = (dist < self.kturn_trigger_dist) & (dheading.abs() > self.kturn_min_heading_err)
        start_reverse = stalled & (~reversing) & (~parked)
        self.reverse_timer = torch.where(
            start_reverse, torch.full_like(self.reverse_timer, self.kturn_reverse_steps), self.reverse_timer
        )
        reversing = self.reverse_timer > 0
        self.gear = torch.where(reversing, -torch.ones_like(self.gear), torch.ones_like(self.gear))

        # --- FORWARD leg: Stanley steering tracking the approach line (arrives aligned) -----------
        speed = (self.k_rho * dist).clamp(max=self.max_drive_cmd)
        speed_fwd = speed * torch.cos(bearing_fwd).clamp(min=0.0)
        cross_track = dx * torch.sin(dheading) - dy * torch.cos(dheading)
        steer_fwd = dheading - torch.atan2(self.k_cross * cross_track, speed_fwd + self.stanley_softening)

        # --- REVERSE leg: back up while steering to SWING THE HEADING toward the staging heading ---
        # We want the car's heading to rotate toward the goal heading, i.e. cancel dheading. While
        # reversing, a given steer produces the opposite yaw, so steering by +dheading and then
        # flipping the sign for reverse drives the heading error toward zero -- so that when the
        # forward leg resumes the car is closer to aligned. Back up at a modest fixed speed.
        steer_rev = -dheading  # sign flip vs forward so reverse motion reduces the heading error
        speed_rev = torch.full_like(speed, 0.35)

        delta = torch.where(reversing, steer_rev, steer_fwd)
        speed_mag = torch.where(reversing, speed_rev, speed_fwd)
        steer_cmd = (self.steer_sign * self.steer_gain * delta).clamp(-1.0, 1.0)
        drive_cmd = self.gear * speed_mag

        # Park: once within BOTH tolerances, cut to zero and hold.
        steer_cmd = torch.where(parked, torch.zeros_like(steer_cmd), steer_cmd)
        drive_cmd = torch.where(parked, torch.zeros_like(drive_cmd), drive_cmd)

        return torch.stack([steer_cmd, drive_cmd], dim=1)
