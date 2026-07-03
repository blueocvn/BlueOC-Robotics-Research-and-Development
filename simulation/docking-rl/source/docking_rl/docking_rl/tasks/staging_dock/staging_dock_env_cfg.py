"""JetRacer staging-approach task, manager-based workflow.

Scope (Section 7 of the setup guide -- real-deployment framing): this RL policy owns only the
short-range, precise approach to a staging pose. It takes the robot's pose (from Nav2/AMCL at
deployment) and a known staging pose, and drives the last ~1.5-2 m to that pose.

Deliberately pose-only -- no obstacle perception in the policy:
  * Global navigation, obstacle avoidance, and *replanning* are Nav2's job (it stays live for
    localization and hands off to this policy within ~1.5-2 m of the staging pose).
  * If an obstacle appears during the RL segment, a separate non-learned safety reflex in the
    handoff node (LiDAR min-range) stops/backs the robot off and hands control back to Nav2 to
    replan -- rather than the RL policy trying to drive around it.
  * The final centimetres onto the dock are a separate classical AprilTag visual-servo stage.
So the only external input the policy needs is the staging-pose error (goal seen from the robot's
current, Nav2-provided pose); everything else is the robot's own motion state.
"""

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

import docking_rl.tasks.staging_dock.mdp as mdp
from docking_rl.assets import JETRACER_CFG

##
# Task geometry (env-local frame, i.e. relative to each cloned env's origin).
##
CAR_SPAWN_POS = (0.0, 0.0, 0.063)
STAGING_NOMINAL_POS_B = (1.5, 0.0, 0.06)


##
# Scene definition
##


@configclass
class DockingSceneCfg(InteractiveSceneCfg):
    """Ground plane + JetRacer + light. No obstacles -- avoidance is Nav2's job, not the policy's."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(
            size=(20.0, 20.0),
            # High tyre grip. The default 0.5 friction lets the light rear end slide, so the car
            # oversteers and pivots ("spins") under steering. More grip keeps the rear planted so a
            # steering command produces a proper turn radius instead of a spin. combine_mode="max"
            # ensures the high value wins regardless of the wheel material.
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=1.0,
                friction_combine_mode="max",
            ),
        ),
    )

    car: ArticulationCfg = JETRACER_CFG.replace(
        prim_path="{ENV_REGEX_NS}/JetRacer",
        init_state=JETRACER_CFG.init_state.replace(pos=CAR_SPAWN_POS),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    # The staging pose the robot must reach. At deployment its error is computed from the Nav2/AMCL
    # map-frame pose minus the known staging pose; in sim the command term tracks it directly.
    staging_pose = mdp.StagingPoseCommandCfg(
        asset_name="car",
        resampling_time_range=(1.0e6, 1.0e6),  # effectively: resample only on episode reset
        # STATIC goal: the staging pose is a fixed offset from the dock, with zero jitter. Heading 0
        # means the robot must arrive facing +x -- i.e. NOSE-IN toward the dock/tag (which sits at
        # larger x), so the front camera sees the AprilTag for the handoff. All the "approach from
        # anywhere" generalization comes from the start-pose arc below, not from moving the goal.
        nominal_pos_b=STAGING_NOMINAL_POS_B,
        nominal_heading=0.0,
        ranges=mdp.StagingPoseCommandCfg.Ranges(
            pos_x=(0.0, 0.0), pos_y=(0.0, 0.0), heading=(0.0, 0.0)
        ),
        # Success requires the tolerance to hold for 5 consecutive steps (dwell-time gate), not
        # just an instantaneous crossing -- see StagingPoseCommand's docstring. Both the
        # staging_pose_reached reward and the staging_pose_success termination read this same
        # gate (StagingPoseCommand.success_held), so they can never fire out of sync.
        success_pos_tolerance=0.3,
        success_heading_tolerance=0.175,
        success_hold_steps=5,
        debug_vis=True,
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP: 2D continuous [steer, drive]."""

    ackermann_drive = mdp.AckermannActionCfg(
        asset_name="car",
        steer_joint_names=["steer_joint_L", "steer_joint_R"],
        drive_joint_names=["wheel_joint_RL", "wheel_joint_RR"],
        # 20 deg, reduced from the 30 deg physical limit: a shorter max steer means a larger
        # minimum turn radius (~0.44 m vs ~0.28 m at this 0.16 m wheelbase), so the very short,
        # light chassis is much less prone to snap-oversteering into a spin. Still well within the
        # joints' +/-30 deg range.
        steer_angle_limit=0.349,
        drive_velocity_limit=20.0,  # rad/s; slightly reduced -> less momentum-driven rear slide
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        # goal: (dx, dy, dheading) to the staging pose, in the vehicle frame.
        # This is the only external input -- at deployment it comes from Nav2/AMCL minus the
        # known staging pose. The remaining terms are the robot's own motion state.
        staging_pose_error = ObsTerm(func=mdp.generated_commands, params={"command_name": "staging_pose"})
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("car")})
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("car")})
        steering_angle = ObsTerm(
            func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("car", joint_names=["steer_joint_L"])}
        )
        last_action = ObsTerm(func=mdp.last_action)
        # dwell-time success-gate progress, in [0,1] -- see hold_progress's docstring: without
        # this, the reward/termination depend on unobserved history (consecutive in-tolerance
        # step count), breaking the Markov property the critic's bootstrapped TD-target relies on
        staging_pose_hold_progress = ObsTerm(
            func=mdp.staging_pose_hold_progress, params={"command_name": "staging_pose"}
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events (domain randomization)."""

    # Start pose distribution: the robot is dropped anywhere in the tag's *front 180-degree arc*,
    # within Nav2's handoff radius, roughly (but not exactly) facing the staging pose. The staging
    # pose is at STAGING_NOMINAL_POS_B facing +x (toward the dock/tag at larger x), so the approach
    # zone is on its -x side -> approach_dir = pi, arc_half_angle = pi/2 gives the full 180 deg.
    reset_car_pose = EventTerm(
        func=mdp.reset_root_state_in_approach_arc,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("car"),
            "center": (STAGING_NOMINAL_POS_B[0], STAGING_NOMINAL_POS_B[1]),
            "approach_dir": math.pi,
            "arc_half_angle": math.pi / 2.0,  # +/- 90 deg -> 180 deg total spread
            "radius_range": (0.8, 1.8),  # Nav2 handoff radius
            "heading_noise": math.pi / 2.0,  # +/- 90 deg around "facing the staging pose"
            "z": CAR_SPAWN_POS[2],
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP: reach the staging pose smoothly and efficiently.

    All weights restored. The reward-only isolation test (only ``position_progress`` active)
    conclusively proved the sustained critic divergence wasn't about reward design -- root cause
    was ``clip_actions: True`` in ``agents/skrl_sac_cfg.yaml`` (fixed) plus a NaN-blind
    ``out_of_bounds`` check (also fixed, see terminations.py) that let a corrupted physics state
    silently poison training instead of resetting. A residual single-event NaN spike was still
    observed even with both fixes (step ~7600 of an 8000-step run) -- smaller in both frequency
    and character than the original sustained divergence, but not fully eliminated yet.
    """

    # -- primary shaping: progress toward + alignment with the staging pose
    position_progress = RewTerm(func=mdp.position_progress, weight=1.0, params={"command_name": "staging_pose"})
    heading_alignment = RewTerm(func=mdp.heading_alignment, weight=0.5, params={"command_name": "staging_pose"})
    staging_pose_reached = RewTerm(
        func=mdp.staging_pose_reached, weight=25.0, params={"command_name": "staging_pose"}
    )
    # reward actually STOPPING near the goal, not just arriving -- see loiter_penalty's docstring;
    # added after a step-5000 checkpoint showed the car orbiting near the goal for the full
    # episode instead of settling (staging_pose_reached stuck at 0 despite good position_progress)
    #
    # weight is POSITIVE, matching position_progress/heading_alignment's convention: the function
    # already returns a negative value (-(speed)*gate), so weight only scales magnitude. An
    # earlier version used weight=-1.0 here (spin_in_place_penalty's convention: function returns
    # a positive magnitude, weight supplies the minus sign) -- since loiter_penalty's function
    # ALREADY embeds the minus sign, that double-negated it into a net REWARD for speed near the
    # goal, the opposite of intended. Caught via TensorBoard: Episode_Reward/loiter_penalty was
    # logging positive (0.14-0.38), which is impossible for a term whose raw output is <= 0 by
    # construction -- that's what exposed the sign flip.
    loiter_penalty = RewTerm(func=mdp.loiter_penalty, weight=1.0, params={"command_name": "staging_pose"})

    # -- comfort / efficiency (secondary shaping)
    jerk_penalty = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
    gear_shift_penalty = RewTerm(func=mdp.gear_shift_penalty, weight=-0.2)
    time_penalty = RewTerm(func=mdp.is_alive, weight=-0.02)
    # directly and heavily penalizes rotating without translating -- see spin_in_place_penalty's
    # docstring for why action_rate_l2/heading_alignment don't already cover this.
    # weight reduced from -2.0 (peak -62.7/step) to -0.5 (peak -0.5*4.0^2=-8.0/step): the old peak
    # was a huge scale outlier against every other term (all in [-0.05,-2]) -- kept at this more
    # conservative scale even though reward magnitude turned out not to be the root cause of the
    # critic divergence, since it's still reasonable design hygiene (no reason to reintroduce a
    # scale outlier). -8 keeps it clearly the most punishing single term, no longer an outlier.
    spin_in_place_penalty = RewTerm(func=mdp.spin_in_place_penalty, weight=-0.5)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    out_of_bounds = DoneTerm(func=mdp.out_of_bounds, params={"asset_cfg": SceneEntityCfg("car")})
    # success: the staging-pose tolerance is met. Reaching here hands off to the classical
    # AprilTag visual-servo stage, which is out of scope for this RL task.
    staging_pose_success = DoneTerm(func=mdp.staging_pose_success, params={"command_name": "staging_pose"})


##
# Environment configuration
##


@configclass
class StagingDockEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the JetRacer staging-approach environment."""

    scene: DockingSceneCfg = DockingSceneCfg(num_envs=2048, env_spacing=10.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 12.0  # room for off-axis arc approaches (start up to 1.8 m out)
        self.viewer.eye = (4.0, -4.0, 4.0)
        self.viewer.lookat = (2.0, 0.0, 0.0)
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation
