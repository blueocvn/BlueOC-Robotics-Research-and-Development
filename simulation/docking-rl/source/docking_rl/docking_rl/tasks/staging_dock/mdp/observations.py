"""Observation terms for the JetRacer staging-pose docking task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def staging_pose_hold_progress(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Dwell-time gate progress, in [0, 1]. See :meth:`StagingPoseCommand.hold_progress`.

    Shape ``(num_envs, 1)`` to concatenate cleanly with the other 1D observation terms.
    """
    command_term = env.command_manager.get_term(command_name)
    return command_term.hold_progress.unsqueeze(-1)
