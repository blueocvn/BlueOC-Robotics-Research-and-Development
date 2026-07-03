"""Reward terms for the JetRacer staging-pose docking task.

Mirrors the multi-objective design from the guide's Section 3 table: safety (collision) dominates,
progress/heading are the primary shaping signals, comfort (jerk) / efficiency (time, gear shifts)
are secondary. The RL task only targets the staging pose -- the AprilTag final approach is a
separate, classical visual-servo stage (Section 6.4), so tag visibility is not part of this reward.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_progress(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Dense shaping: negative distance to the staging pose."""
    command = env.command_manager.get_command(command_name)
    return -torch.norm(command[:, :2], dim=1)


def heading_alignment(env: ManagerBasedRLEnv, command_name: str, gate_distance: float = 0.6) -> torch.Tensor:
    """Dense shaping: negative heading error to the staging pose, GATED by proximity.

    Only rewarded near the goal (gate ~1 at the staging pose, decaying to ~0 beyond
    ``gate_distance``). This matters a lot once the robot can start anywhere in the tag's front
    180-degree arc: far from the goal, "drive toward the goal" and "face the staging heading" point
    in different directions, so an ungated heading reward pulls the robot to keep rotating toward
    the final heading instead of driving -- i.e. it spins on the spot. Gating removes that conflict:
    far away only position matters (drive there, any heading); the final heading is aligned only
    near the goal, where it's actually meaningful for the AprilTag handoff.

    ``gate_distance=0.6`` (vs. the success radius of 0.3) is deliberately a bit wider than "right at
    the goal": at gate_distance == success_pos_tolerance exactly, a policy could loiter just
    *outside* the success radius (e.g. a wide, gentle circle at ~0.5-0.8 m, too wide to trip the
    spin_in_place_penalty's curvature threshold) while paying almost nothing for facing the wrong
    way, since the heading gate there would be near 0. Widening the gate means that loiter zone
    still costs meaningful heading-error reward, without reaching so far out that it recreates the
    original far-field "rotate to face the goal instead of driving" problem this gating exists to
    prevent (at a typical 1.5 m start, gate = exp(-(1.5/0.6)^2) ~= 0.002, still negligible there).
    """
    command = env.command_manager.get_command(command_name)
    dist = torch.norm(command[:, :2], dim=1)
    gate = torch.exp(-((dist / gate_distance) ** 2))
    return -torch.abs(command[:, 2]) * gate


def loiter_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("car"),
    gate_distance: float = 0.6,
) -> torch.Tensor:
    """Penalize speed (both linear and angular) near the goal -- i.e. reward actually STOPPING,
    not just arriving.

    Measured after training to step 5000: full-severity donuts are gone (spin_in_place_penalty
    averages ~-1.75/step, far below its ~-62.7 worst-case), but episodes still run the complete
    12s timeout with staging_pose_reached permanently at 0 -- meanwhile position_progress shows
    the car sitting at ~0.05 m average distance essentially the whole episode. That combination
    means the policy learned to drive up close to the goal and then ORBIT/circle there for the
    rest of the episode, never precisely satisfying the position+heading dwell gate. Nothing
    previously rewarded stopping specifically -- position_progress only cares about distance
    (which a tight orbit already satisfies well), and the (fairly mild) curvature penalty alone
    isn't enough to make circling clearly worse than the harder, more precise act of killing
    velocity and holding still. This term closes that gap directly: near the goal (same proximity
    gate as heading_alignment, so it engages in the same zone and doesn't fight the far-field
    approach), moving fast -- translating OR rotating -- costs reward; being still is free.
    """
    command = env.command_manager.get_command(command_name)
    dist = torch.norm(command[:, :2], dim=1)
    gate = torch.exp(-((dist / gate_distance) ** 2))

    asset: Articulation = env.scene[asset_cfg.name]
    lin_speed = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    ang_speed = asset.data.root_ang_vel_b[:, 2].abs()
    return -(lin_speed + ang_speed) * gate


def staging_pose_reached(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Sparse bonus: 1.0 on the step ``success_held`` goes True (tolerance held for
    ``success_hold_steps`` consecutive steps -- see :class:`StagingPoseCommand`), else 0.0.

    Reads the command term's single ``success_held`` field rather than re-checking tolerances
    itself, so this can never fire out of sync with the ``staging_pose_success`` termination --
    which matters because that termination ends the episode the same step this bonus is granted;
    if the two used independently-defined tolerances/logic they could disagree (e.g. this firing
    repeatedly on isolated boundary touches without the episode ever actually ending).
    """
    command_term = env.command_manager.get_term(command_name)
    return command_term.success_held.float()


def spin_in_place_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("car"),
    max_curvature: float = 2.275,  # 1/min_turn_radius; matches 20 deg steer, 0.16 m wheelbase
    min_speed_for_curvature: float = 0.05,
    # Capped at 4.0 (was 5.6, matched to the measured worst-case donut's curvature). At weight
    # -2.0 that made peak magnitude -62.7/step -- a 30-3000x outlier against every other reward
    # term (all in [-0.05, -2] range) -- which is a well-documented SAC destabilizer: a training
    # run to 20k steps showed Q1(mean)/critic loss/policy loss diverging in lockstep (Q1 mean
    # 0.14 -> 453,149 by step 9800, critic loss into the quintillions) while total reward
    # plateaued, never improving. See also the weight change below.
    max_excess_curvature: float = 4.0,
) -> torch.Tensor:
    """Penalize yaw rate beyond what clean (non-slipping) steering geometry can produce.

    An earlier version of this gated on "near-zero forward speed", meant to catch true
    spin-in-place. It missed a related failure mode: a *slip-driven tight donut* -- yaw_rate ~4.3
    rad/s at forward-body-speed ~0.5 m/s, i.e. a ~0.13 m turning radius -- which has real forward
    speed (so the speed-gate went to 0, killing the penalty entirely) but is still not legitimate
    driving; it's a fast rotation that stays roughly in place, exactly what "the car just spins"
    describes. This car's steering geometry (``steer_angle_limit``, wheelbase) has a minimum turn
    radius of clean rolling; kinematic curvature = yaw_rate / forward_speed cannot legitimately
    exceed ``1 / min_radius`` without wheel slip. So this penalizes *excess curvature* --
    ``yaw_rate / forward_speed`` beyond ``max_curvature`` -- which catches both true spin-in-place
    (forward_speed -> 0, curvature -> the clamp ceiling) and tight slip-donuts (real forward speed,
    but curvature far beyond what rolling contact allows), while leaving legitimate cornering
    within the vehicle's real turn-radius limit (needed for the 180-degree arc approach) unpenalized.

    Bounded by construction (curvature ratio clamped to ``max_excess_curvature`` before squaring)
    to avoid feeding an unbounded term into the SAC critic.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    yaw_rate = asset.data.root_ang_vel_b[:, 2].abs()
    fwd_speed = asset.data.root_lin_vel_b[:, 0].abs().clamp(min=min_speed_for_curvature)
    curvature = yaw_rate / fwd_speed
    excess = torch.clamp(curvature - max_curvature, min=0.0, max=max_excess_curvature)
    return excess**2


def gear_shift_penalty(env: ManagerBasedRLEnv, drive_action_index: int = 1, deadband: float = 0.05) -> torch.Tensor:
    """Penalize forward/reverse switches of the drive command (matches the paper's ANGS metric).

    Reads the raw (pre-scaling) drive command from the action manager's full action buffer.
    ``drive_action_index`` assumes the drive command is the only/first action term, at that index
    in the concatenated action vector -- true as long as :class:`AckermannAction` is the sole
    action term in the environment.
    """
    curr_drive = env.action_manager.action[:, drive_action_index]
    prev_drive = env.action_manager.prev_action[:, drive_action_index]
    switched = (prev_drive.abs() > deadband) & (curr_drive.abs() > deadband) & (curr_drive.sign() != prev_drive.sign())
    return switched.float()
