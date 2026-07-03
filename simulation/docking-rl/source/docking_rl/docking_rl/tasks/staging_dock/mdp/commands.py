"""Command term: the docking-bay staging pose (Section 6.1 of the setup guide).

The RL goal is not the tight slot/tag pose -- it's a looser staging pose offset from the tag
(``nominal_pos_b`` / ``nominal_heading``), such that the tag is reliably inside the front camera's
FOV once the staging tolerance is met. Sampling a new pose within ``ranges`` around that nominal
offset on every reset is the "randomize slot position" domain randomization the guide calls for:
each episode gets a slightly different bay layout without having to re-author the USD scene.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.markers.config import GREEN_ARROW_X_MARKER_CFG
from isaaclab.utils import configclass
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class StagingPoseCommand(CommandTerm):
    """Samples a staging-pose goal (x, y, heading) per environment, once per episode."""

    cfg: StagingPoseCommandCfg

    def __init__(self, cfg: StagingPoseCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]

        self.pos_command_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.heading_command_w = torch.zeros(self.num_envs, device=self.device)
        self.pos_command_b = torch.zeros(self.num_envs, 2, device=self.device)
        self.heading_command_b = torch.zeros(self.num_envs, device=self.device)

        self.metrics["error_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_heading"] = torch.zeros(self.num_envs, device=self.device)

        # Dwell-time success gate: the tolerance must hold for `success_hold_steps` CONSECUTIVE
        # steps before `success_held` goes True. Both the reward's sparse bonus and the episode
        # termination read this single field, so they can never disagree (no world where the
        # reward fires but the episode doesn't end, or vice versa). Without this, an instantaneous
        # crossing of a hard-threshold AND-of-two-conditions boundary is exactly the kind of thing
        # a policy that's dithering/orbiting near the goal (due to imprecise control, or a locally
        # awkward approach angle) can trigger repeatedly without ever settling -- each isolated
        # touch could grant the reward without the episode ending, letting a policy "graze" the
        # tolerance for repeated partial credit instead of actually completing the approach.
        # Requiring several consecutive in-tolerance steps closes that: a policy has to actually
        # arrive and hold, not just clip the boundary in passing.
        self._success_hold_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.success_held = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def __str__(self) -> str:
        return f"StagingPoseCommand:\n\tCommand dimension: {tuple(self.command.shape[1:])}"

    @property
    def command(self) -> torch.Tensor:
        """Staging pose error in the vehicle frame: ``(dx, dy, dheading)``. Shape ``(num_envs, 3)``."""
        return torch.cat([self.pos_command_b, self.heading_command_b.unsqueeze(1)], dim=1)

    def _update_metrics(self):
        self.metrics["error_pos"] = torch.norm(
            self.pos_command_w[:, :2] - self.robot.data.root_pos_w[:, :2], dim=1
        )
        self.metrics["error_heading"] = torch.abs(
            math_utils.wrap_to_pi(self.heading_command_w - self.robot.data.heading_w)
        )

    def _resample_command(self, env_ids: Sequence[int]):
        r = torch.empty(len(env_ids), device=self.device)
        origins = self._env.scene.env_origins[env_ids]

        self.pos_command_w[env_ids, 0] = origins[:, 0] + self.cfg.nominal_pos_b[0] + r.uniform_(*self.cfg.ranges.pos_x)
        self.pos_command_w[env_ids, 1] = origins[:, 1] + self.cfg.nominal_pos_b[1] + r.uniform_(*self.cfg.ranges.pos_y)
        self.pos_command_w[env_ids, 2] = origins[:, 2] + self.cfg.nominal_pos_b[2]
        self.heading_command_w[env_ids] = self.cfg.nominal_heading + r.uniform_(*self.cfg.ranges.heading)

    def _update_command(self):
        target_vec = self.pos_command_w[:, :2] - self.robot.data.root_pos_w[:, :2]
        yaw = self.robot.data.heading_w
        cos_yaw, sin_yaw = torch.cos(yaw), torch.sin(yaw)
        self.pos_command_b[:, 0] = cos_yaw * target_vec[:, 0] + sin_yaw * target_vec[:, 1]
        self.pos_command_b[:, 1] = -sin_yaw * target_vec[:, 0] + cos_yaw * target_vec[:, 1]
        self.heading_command_b[:] = math_utils.wrap_to_pi(self.heading_command_w - yaw)

        in_tolerance = (torch.norm(self.pos_command_b, dim=1) < self.cfg.success_pos_tolerance) & (
            torch.abs(self.heading_command_b) < self.cfg.success_heading_tolerance
        )
        self._success_hold_counter = torch.where(
            in_tolerance, self._success_hold_counter + 1, torch.zeros_like(self._success_hold_counter)
        )
        self.success_held = self._success_hold_counter >= self.cfg.success_hold_steps

    @property
    def hold_progress(self) -> torch.Tensor:
        """Fraction of the dwell-time gate completed so far, in [0, 1]. Shape ``(num_envs,)``.

        Exposed as an observation term (see ``ObservationsCfg``) so the dwell-time hold-counter --
        which the reward (``staging_pose_reached``) and termination (``staging_pose_success``)
        both depend on -- is visible to the policy/value function, not just internal command-term
        state. Without this, the same observed (pose-error, velocity, ...) state can map to
        different rewards depending on invisible history (how many consecutive in-tolerance steps
        already happened), which breaks the Markov property the critic's bootstrapped TD-target
        relies on. That mismatch is a plausible root cause of the sustained Q-value/critic-loss
        divergence seen in training (Q1(mean) growing roughly quadratically over thousands of
        updates while the actual per-episode reward stayed a flat, bounded plateau) -- reward-scale
        fixes alone slowed but never stopped it, consistent with the issue being structural
        (a violated Markov assumption) rather than purely a magnitude problem.
        """
        return (self._success_hold_counter.float() / self.cfg.success_hold_steps).clamp(max=1.0)

    def reset(self, env_ids: Sequence[int] | None = None) -> dict[str, float]:
        extras = super().reset(env_ids)
        resolved_ids = slice(None) if env_ids is None else env_ids
        self._success_hold_counter[resolved_ids] = 0
        self.success_held[resolved_ids] = False
        return extras

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_pose_visualizer"):
                self.goal_pose_visualizer = VisualizationMarkers(self.cfg.goal_pose_visualizer_cfg)
            self.goal_pose_visualizer.set_visibility(True)
        elif hasattr(self, "goal_pose_visualizer"):
            self.goal_pose_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        self.goal_pose_visualizer.visualize(
            translations=self.pos_command_w,
            orientations=math_utils.quat_from_euler_xyz(
                torch.zeros_like(self.heading_command_w),
                torch.zeros_like(self.heading_command_w),
                self.heading_command_w,
            ),
        )


@configclass
class StagingPoseCommandCfg(CommandTermCfg):
    """Configuration for :class:`StagingPoseCommand`."""

    class_type: type = StagingPoseCommand

    asset_name: str = MISSING
    """Name of the vehicle asset in the scene."""

    nominal_pos_b: tuple[float, float, float] = MISSING
    """Nominal staging position, in the env-local frame (relative to the env origin)."""

    nominal_heading: float = 0.0
    """Nominal staging heading, in radians, in the env-local frame."""

    @configclass
    class Ranges:
        pos_x: tuple[float, float] = (0.0, 0.0)
        pos_y: tuple[float, float] = (0.0, 0.0)
        heading: tuple[float, float] = (0.0, 0.0)

    ranges: Ranges = Ranges()
    """Uniform jitter ranges added to the nominal staging pose on every resample."""

    success_pos_tolerance: float = 0.3
    """Position tolerance (m) for the dwell-time success gate (``success_held``)."""

    success_heading_tolerance: float = 0.175
    """Heading tolerance (rad) for the dwell-time success gate (``success_held``)."""

    success_hold_steps: int = 5
    """Number of consecutive env-steps the tolerance must hold before ``success_held`` is True."""

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/staging_pose_goal"
    )
    goal_pose_visualizer_cfg.markers["arrow"].scale = (0.15, 0.15, 0.5)
