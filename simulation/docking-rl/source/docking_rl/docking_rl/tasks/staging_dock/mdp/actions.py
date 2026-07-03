"""Custom action term: 2D Ackermann-style drive command for the JetRacer.

The policy outputs a 2D continuous action ``[steer_cmd, drive_cmd]`` in ``[-1, 1]``. This is the
same action-space shape used by the kinematic-bicycle-model shortcut described for the generic
"no model yet" case, but here it drives the JetRacer's real steering/drive joints (see
``assets/jetracer.py``) instead of writing pose directly to the rigid body -- the JetRacer USD is
a real articulation with a steering servo + rear drive motors, so there's no need for the shortcut.

Both front steering joints receive the same target angle (single-servo tie-rod steering, no
independent per-wheel Ackermann geometry -- matches the real hardware). Both rear wheel joints
receive the same target velocity (no explicit differential-steering term; left/right can still
diverge slightly under the implicit PD controller if resisted unevenly).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class AckermannAction(ActionTerm):
    """Maps a 2D ``[steer, drive]`` action to the JetRacer's steering/drive joint targets."""

    cfg: AckermannActionCfg

    def __init__(self, cfg: AckermannActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._asset: Articulation

        self._steer_joint_ids, self._steer_joint_names = self._asset.find_joints(cfg.steer_joint_names)
        self._drive_joint_ids, self._drive_joint_names = self._asset.find_joints(cfg.drive_joint_names)

        self._raw_actions = torch.zeros(self.num_envs, 2, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, 2, device=self.device)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return 2

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        clipped = torch.clamp(actions, -1.0, 1.0)
        # column 0: steering angle command, column 1: drive velocity command
        self._processed_actions[:, 0] = clipped[:, 0] * self.cfg.steer_angle_limit
        self._processed_actions[:, 1] = clipped[:, 1] * self.cfg.drive_velocity_limit

    def apply_actions(self):
        steer_target = self._processed_actions[:, 0].unsqueeze(1).expand(-1, len(self._steer_joint_ids))
        drive_target = self._processed_actions[:, 1].unsqueeze(1).expand(-1, len(self._drive_joint_ids))
        self._asset.set_joint_position_target(steer_target, joint_ids=self._steer_joint_ids)
        self._asset.set_joint_velocity_target(drive_target, joint_ids=self._drive_joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._raw_actions[env_ids] = 0.0
        self._processed_actions[env_ids] = 0.0


@configclass
class AckermannActionCfg(ActionTermCfg):
    """Configuration for :class:`AckermannAction`."""

    class_type: type = AckermannAction

    steer_joint_names: list[str] = MISSING
    """Regex(es) matching the front steering joints (commanded to the same angle)."""

    drive_joint_names: list[str] = MISSING
    """Regex(es) matching the rear drive joints (commanded to the same velocity)."""

    steer_angle_limit: float = 0.5236
    """Max magnitude of the steering angle command, in radians (default: 30 deg)."""

    drive_velocity_limit: float = 25.0
    """Max magnitude of the drive wheel angular-velocity command, in rad/s."""
