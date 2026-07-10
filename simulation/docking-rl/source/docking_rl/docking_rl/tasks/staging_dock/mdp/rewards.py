"""Reward terms for the JetRacer staging-pose docking task.

Mirrors the multi-objective design from the guide's Section 3 table: safety (collision) dominates,
progress/heading are the primary shaping signals, comfort (jerk) / efficiency (time, gear shifts)
are secondary. The RL task only targets the staging pose -- the AprilTag final approach is a
separate, classical visual-servo stage (Section 6.4), so tag visibility is not part of this reward.
"""

from __future__ import annotations

import functools
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _finite(func):
    """Decorator: replace NaN/+-Inf in a reward term's output with finite values.

    A physics blow-up writes NaN robot velocities/positions; terms that read those raw (e.g.
    loiter_penalty, spin_in_place_penalty) then return NaN. Crucially this bites EVEN WHEN A TERM'S
    WEIGHT IS 0 -- the RewardManager still evaluates every term and computes weight*value, and
    0.0 * NaN == NaN, so a single disabled physics-reading term silently NaN-poisons the total
    reward -> the SAC critic target -> the whole network, permanently (the seed-deterministic
    "training dies ~step 3000" failure). Sanitising each term's output closes that path.

    Uses functools.wraps so the wrapped function keeps its original signature, which Isaac Lab's
    RewardManager introspects to match term params.
    """

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        return torch.nan_to_num(func(*args, **kwargs), nan=0.0, posinf=1.0e3, neginf=-1.0e3)

    return wrapper


@_finite
def goal_attraction(
    env: ManagerBasedRLEnv,
    command_name: str,
    pos_scale: float = 1.0,
    heading_scale: float = 0.5,
    speed_scale: float = 1.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("car"),
) -> torch.Tensor:
    """DENSE pose-attraction: an always-positive reward maximized at the staging pose, aligned and
    stopped. Designed to *pull* the car to the docked state.

    Why this exists: the potential-based ``position_progress`` + proximity penalties setup was
    getting reward-hacked -- the policy learned to AVOID the goal vicinity because
    ``loiter_penalty`` punishes moving near the goal, so staying away minimized cost (success rate
    fell BELOW random). This term inverts that incentive: it pays the most for being AT the goal.

        base  = exp(-(dist/pos_scale)^2)                 # 1 at goal, ->0 far -- the dominant pull
        bonus = 1 + exp(-(dheading/heading_scale)^2)      # +up to 1 for facing the staging heading
              +     exp(-(speed/speed_scale))             # +up to 1 for being stopped
        reward = base * bonus                             # in [0, 3]; peak 3 only when docked+still

    ``pos_scale=1.0`` gives a usable gradient across the whole ~1.5 m approach (at 1.5 m base~0.1,
    at 0.5 m ~0.78), so the car is pulled in from the start; the heading/speed bonuses only matter
    once ``base`` is non-negligible (near the goal), so far away it just says "come here". Being at
    the goal, aligned and stopped is the global max -- exactly the docked state -- so unlike the
    potential form there is NO way to farm reward by hovering away from the goal.
    """
    command = env.command_manager.get_command(command_name)
    dist = torch.norm(command[:, :2], dim=1)
    dheading = command[:, 2]
    base = torch.exp(-((dist / pos_scale) ** 2))

    asset: Articulation = env.scene[asset_cfg.name]
    speed = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1) + asset.data.root_ang_vel_b[:, 2].abs()
    aligned = torch.exp(-((dheading / heading_scale) ** 2))
    slow = torch.exp(-speed / speed_scale)
    return base * (1.0 + aligned + slow)


@_finite
def position_progress(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Dense shaping: potential-based progress delta (prev_distance - curr_distance), NOT raw
    -distance.

    This is the textbook potential-based shaping form (Ng et al. 1999: F = gamma*Phi(s') - Phi(s),
    here undiscounted since it's evaluated every step with Phi = -distance) -- provably preserves
    the optimal policy, unlike an arbitrary shaping term. The distinction matters concretely here:
    a bare potential (``-distance``, the previous version of this function) pays the same fixed
    reward for merely SITTING at some distance every step; the delta form pays exactly 0 for
    standing still and only rewards actually closing the gap. A training run showed total reward
    and position_progress improving then plateauing hard around step 33k (of 100k) while
    staging_pose_reached stayed flat throughout and the SAC entropy coefficient collapsed to ~0 --
    consistent with the policy settling into "hover near the goal" as a locally-decent optimum
    under the old bare-potential form, since standing still there was already free reward.

    Reads ``StagingPoseCommand.progress_delta`` (computed once per step in ``_update_command``,
    zeroed on the first step after a reset) rather than recomputing distance here, so reward and
    observation always agree on the same underlying distance calculation.
    """
    command_term = env.command_manager.get_term(command_name)
    return command_term.progress_delta


@_finite
def staging_pose_hold_credit(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Dense shaping: partial credit for dwell-gate progress, in [0, 1] (see
    ``StagingPoseCommand.hold_progress``).

    The sparse ``staging_pose_reached`` bonus only fires once the tolerance has held for
    ``success_hold_steps`` consecutive steps -- an all-or-nothing cliff. At the observed ~0.2-0.3%
    per-step success rate, that +25 bonus is so rare in the replay buffer that it provides almost
    no gradient toward actually finishing the approach (as opposed to just getting close). This
    term gives a smooth, continuous signal for progress *through* the hold -- 2/5 steps in
    tolerance is worth more than 0/5 -- so the critic has something to climb toward completion
    instead of a near-invisible jackpot.
    """
    command_term = env.command_manager.get_term(command_name)
    return command_term.hold_progress


@_finite
def heading_alignment(env: ManagerBasedRLEnv, command_name: str, gate_distance: float = 0.6) -> torch.Tensor:
    """Dense shaping: negative heading error to the staging pose, GATED by proximity.

    Only rewarded near the goal (gate ~1 at the staging pose, decaying to ~0 beyond
    ``gate_distance``). This matters a lot once the robot can start anywhere in the tag's front
    180-degree arc: far from the goal, "drive toward the goal" and "face the staging heading" point
    in different directions, so an ungated heading reward pulls the robot to keep rotating toward
    the final heading instead of driving -- i.e. it spins on the spot. Gating removes that conflict:
    far away only position matters (drive there, any heading); the final heading is aligned only
    near the goal, where it's actually meaningful for the AprilTag handoff.

    NOTE: an experimental linear-gate-with-0.1-floor variant was tried here to "keep a heading
    gradient in the far field"; it BACKFIRED -- the 0.1 floor kept penalising heading error 1.5 m
    out, so the policy learned to rotate in place to face the goal (minimising that penalty) instead
    of driving to it. A 150k straight-line run showed spin_in_place_penalty growing (-0.001 ->
    -0.14) and staging_pose_reached decaying to ~0. Reverted to this exp gate, whose whole purpose
    is to prevent exactly that: at a 1.5 m start gate = exp(-(1.5/0.6)^2) ~= 0.002, negligible, so
    far-field heading exerts no pull and the car just drives.

    ``gate_distance=0.6`` (vs. the success radius of 0.3) is deliberately a bit wider than "right at
    the goal": at gate_distance == success_pos_tolerance exactly, a policy could loiter just
    *outside* the success radius (e.g. a wide, gentle circle at ~0.5-0.8 m, too wide to trip the
    spin_in_place_penalty's curvature threshold) while paying almost nothing for facing the wrong
    way, since the heading gate there would be near 0. Widening the gate means that loiter zone
    still costs meaningful heading-error reward, without reaching so far out that it recreates the
    original far-field "rotate to face the goal instead of driving" problem this gating exists to
    prevent.
    """
    command = env.command_manager.get_command(command_name)
    dist = torch.norm(command[:, :2], dim=1)
    gate = torch.exp(-((dist / gate_distance) ** 2))
    return -torch.abs(command[:, 2]) * gate



@_finite
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


@_finite
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


@_finite
def spin_in_place_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("car"),
    max_curvature: float = 2.193,  # 1/min_turn_radius; matches 20 deg steer, 0.166 m wheelbase
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
    return torch.tanh(excess)
    #return excess**2



@_finite
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
