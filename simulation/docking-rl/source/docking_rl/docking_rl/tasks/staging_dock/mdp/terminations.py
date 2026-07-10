"""Termination terms for the JetRacer staging-pose docking task."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def staging_pose_success(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Success: the staging-pose tolerance has held for several consecutive steps (dwell-time
    gate; see :class:`StagingPoseCommand`.success_held) -- not just an instantaneous crossing.
    This is the only success condition the RL task cares about -- the tight, final approach onto
    the AprilTag is a separate classical visual-servo stage (Section 6.4) that starts once this
    policy hands off.

    Reads the same ``success_held`` field the ``staging_pose_reached`` reward reads, so the
    episode always ends on exactly the step that reward is granted -- no world where the bonus
    fires without the episode terminating (which would let a policy farm it repeatedly)."""
    command_term = env.command_manager.get_term(command_name)
    return command_term.success_held


def out_of_bounds(
    env: ManagerBasedRLEnv, bound_x: float = 5.0, bound_y: float = 3.0, asset_cfg: SceneEntityCfg = SceneEntityCfg("car")
) -> torch.Tensor:
    """The vehicle drove off the lot (env-local position exceeds the given bounds), OR its
    position/velocity has gone non-finite.

    The non-finite check matters on its own: a NaN position fails every ``>`` comparison in
    IEEE754 (``NaN > bound_x`` is False, not True), so without it a NaN-corrupted environment --
    e.g. from a rare PhysX contact-resolution blow-up, plausible given this robot's fairly
    aggressive physics settings (high friction, disabled self-collision, tight solver) -- would
    silently keep running for the rest of its episode instead of resetting, feeding poisoned
    (non-finite or extreme) transitions into the replay buffer. That's a concrete, mechanistically
    plausible explanation for a sudden, sign-oscillating Q-value spike (as opposed to the smooth,
    monotonic runaway the since-fixed clip_actions bug produced): one corrupted transition can
    poison the critic's bootstrapped target in a single bad update, rather than a persistent bias
    compounding gradually.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    local_pos = asset.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
    out_of_area = (local_pos[:, 0].abs() > bound_x) | (local_pos[:, 1].abs() > bound_y)
    non_finite = ~torch.isfinite(asset.data.root_pos_w).all(dim=1) | ~torch.isfinite(asset.data.root_vel_w).all(
        dim=1
    )
    return out_of_area | non_finite


def out_of_approach_arc(
    env: ManagerBasedRLEnv,
    command_name: str,
    radius: float,
    approach_dir: float = math.pi,
    side_margin: float = 0.15,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("car"),
) -> torch.Tensor:
    """Terminate if the vehicle leaves the 180-degree approach apron in front of the staging pose.

    The valid region is the half-disk of ``radius`` on the approach side of the staging pose (the
    same region ``reset_root_state_in_approach_arc`` spawns into): the robot must stay within
    ``radius`` of the staging position AND not cross to the far side of it. "Far side" is defined
    relative to ``approach_dir`` (the direction, in the env-local XY frame, from the staging pose
    out toward where the robot approaches from -- ``pi`` here, i.e. the -x side, matching
    ``EventCfg.reset_car_pose``). ``side_margin`` lets the robot slightly overshoot past the
    staging pose plane (needed to actually settle ON the pose without instantly terminating) before
    the far-side cutoff triggers.

    Reads the staging position from the command term (``pos_command_w``) rather than a hardcoded
    center, so the bound automatically tracks the actual goal and stays in sync with it.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    command_term = env.command_manager.get_term(command_name)
    center_w = command_term.pos_command_w[:, :2]  # staging position in world, per env
    rel = asset.data.root_pos_w[:, :2] - center_w

    dist = torch.norm(rel, dim=1)
    approach_unit = torch.tensor(
        [math.cos(approach_dir), math.sin(approach_dir)], device=rel.device, dtype=rel.dtype
    )
    # component of the robot's offset along the approach direction; >= 0 means it's on the approach
    # (valid) side, < -side_margin means it crossed past the staging pose to the far side.
    side = rel @ approach_unit

    out_of_arc = (dist > radius) | (side < -side_margin)
    non_finite = ~torch.isfinite(asset.data.root_pos_w).all(dim=1) | ~torch.isfinite(asset.data.root_vel_w).all(
        dim=1
    )
    return out_of_arc | non_finite
