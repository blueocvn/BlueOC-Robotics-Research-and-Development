"""Task-specific reset events."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_root_state_in_approach_arc(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    center: tuple[float, float],
    approach_dir: float,
    arc_half_angle: float,
    radius_range: tuple[float, float],
    heading_noise: float,
    z: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Place the robot on an arc around ``center`` (the staging pose), on the tag's front side.

    Positions are sampled in polar coordinates so the start distribution is the tag's front
    hemisphere -- an ``2 * arc_half_angle`` wedge centred on ``approach_dir`` (the direction from
    the staging pose out into the approach zone, i.e. along the tag's outward normal), at a random
    radius in ``radius_range``. This is the "robot dropped anywhere within Nav2's handoff radius,
    in front of the tag" distribution.

    Heading is sampled around "facing the staging pose" +/- ``heading_noise`` (Nav2 leaves the
    robot at an uncertain heading, but roughly pointing along its travel direction).

    Args:
        center: (x, y) of the staging pose in the env-local frame (relative to the env origin).
        approach_dir: angle (rad) from the staging pose toward the approach zone (tag outward
            normal). E.g. ``pi`` if the robot approaches from -x.
        arc_half_angle: half-width of the arc (rad). ``pi/2`` gives a full 180-degree spread.
        radius_range: (min, max) distance (m) of the start position from the staging pose.
        heading_noise: max +/- deviation (rad) of the start heading from "facing the staging pose".
        z: spawn height (m) of the robot root.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    device = asset.device
    n = len(env_ids)
    cx, cy = center

    angle = torch.empty(n, device=device).uniform_(approach_dir - arc_half_angle, approach_dir + arc_half_angle)
    radius = torch.empty(n, device=device).uniform_(radius_range[0], radius_range[1])
    px = cx + radius * torch.cos(angle)
    py = cy + radius * torch.sin(angle)

    # base heading points from the sampled start back toward the staging pose, then add noise
    base_yaw = torch.atan2(cy - py, cx - px)
    yaw = base_yaw + torch.empty(n, device=device).uniform_(-heading_noise, heading_noise)

    origins = env.scene.env_origins[env_ids]
    positions = torch.stack([px, py, torch.full_like(px, z)], dim=-1) + origins
    orientations = math_utils.quat_from_euler_xyz(torch.zeros(n, device=device), torch.zeros(n, device=device), yaw)

    asset.write_root_pose_to_sim(torch.cat([positions, orientations], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros(n, 6, device=device), env_ids=env_ids)
