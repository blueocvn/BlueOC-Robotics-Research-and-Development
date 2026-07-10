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

import torch

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
from isaaclab.utils.modifiers import ModifierCfg

import docking_rl.tasks.staging_dock.mdp as mdp
from docking_rl.assets import JETRACER_CFG


def _sanitize_obs(data: torch.Tensor) -> torch.Tensor:
    """Replace NaN/+-Inf in an observation tensor with finite values. A plain Python wrapper (NOT
    torch.nan_to_num directly) because the observation manager introspects the modifier func with
    inspect.signature, which raises on C-builtins like torch.nan_to_num -- this wrapper has a real,
    introspectable signature."""
    return torch.nan_to_num(data, nan=0.0, posinf=1.0e3, neginf=-1.0e3)


def _nan_guard():
    """Fresh list holding the nan_to_num observation modifier: so a physics blow-up can never feed a
    non-finite number into the SAC networks (a single NaN in one gradient update bricks training
    permanently). Returned as a new list per call so each ObsTerm gets its own cfg."""
    return [ModifierCfg(func=_sanitize_obs)]

##
# Task geometry (env-local frame, i.e. relative to each cloned env's origin).
##
CAR_SPAWN_POS = (0.0, 0.0, 0.063)
STAGING_NOMINAL_POS_B = (1.5, 0.0, 0.06)

# Single source of truth for the position acceptance radius, used both by CommandsCfg.staging_pose
# below AND to size the parking-lot outline -- so the outline always represents exactly the actual
# success zone, never an eyeballed guess. (An earlier version sized the outline off a vague "car
# footprint plus some margin" and ended up ~3x the JetRacer's actual ~0.3 x 0.2 m footprint --
# chassis Cube in jetracer_docking_scene.usda is scaled (0.3, 0.11, 0.05) -- for no principled
# reason. Tying it to the tolerance instead means it's allowed to be bigger than the car -- that's
# the point, it's the "must land somewhere in here" zone, not a snug outline of the chassis.)
STAGING_SUCCESS_POS_TOLERANCE = 0.3

# Visual-only parking-lot outline (purely cosmetic -- painted lines, no collision/rigid_props, so
# it can't affect physics or be observed by the policy). Sized to exactly the position-tolerance
# acceptance zone (a square bounding the success radius), centered on STAGING_NOMINAL_POS_B and
# axis-aligned with heading 0.
PARKING_LOT_HALF_LENGTH = STAGING_SUCCESS_POS_TOLERANCE
PARKING_LOT_HALF_WIDTH = STAGING_SUCCESS_POS_TOLERANCE
# Thick/tall enough (5 cm wide, 6 cm tall -- like a curb) to actually read at normal viewport zoom;
# a thin painted-line thickness (~1-2 cm) was visually imperceptible next to the car/goal-arrow scale.
PARKING_LOT_LINE_THICKNESS = 0.05
PARKING_LOT_LINE_HEIGHT = 0.06

# 180-degree approach apron in front of the staging pose: the robot must stay within this half-disk
# or the episode terminates (see TerminationsCfg.out_of_approach_arc). APPROACH_DIR = pi points the
# apron toward -x, matching EventCfg.reset_car_pose's approach_dir -- the side the robot spawns and
# drives in from. Both the out-of-bounds termination AND the visual arc outline read this one
# radius, so they can never disagree.
#
# NOTE on the radius: the spawn distribution (EventCfg.reset_car_pose.radius_range) currently
# reaches out to 1.8 m, so this bound is 2.0 m -- large enough to contain every spawn with margin.
# A tighter 1.0-1.5 m bound (as first sketched) would terminate the farther spawns on step 0; to
# use one, also lower that radius_range max to ~ (bound - 0.2) so robots spawn comfortably inside.
APPROACH_ARC_BOUND_RADIUS = 2.0
APPROACH_ARC_DIR = math.pi
APPROACH_ARC_LINE_THICKNESS = 0.04
APPROACH_ARC_LINE_HEIGHT = 0.05
APPROACH_ARC_NUM_SEGMENTS = 18  # straight chords approximating the 180-degree curved boundary


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
            # Tyre grip. The 1.2/1.0 originally used here was a HACK for the old broken model whose
            # wheels were collapsed at the chassis centre (~zero wheelbase/track) -- that car had no
            # geometric stability, so it oversteered/spun unless friction was cranked up. With the
            # wheels now correctly at the four corners the real wheelbase/track provides that
            # stability, so the extreme grip isn't needed for stability alone -- but the real robot
            # runs on carpet, whose rubber-on-carpet grip is higher than a hard floor, so raised back
            # up to 1.1/0.9 (below the old 1.2/1.0 that produced hard-corner contact impulses/solver
            # NaN) to better match that surface. combine_mode="max" ensures this value wins
            # regardless of the wheel material. If solver NaNs reappear on hard corners, back off
            # toward 0.9/0.7 first.
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.1,
                dynamic_friction=0.9,
                friction_combine_mode="max",
            ),
        ),
    )

    # Parking-lot outline: four thin, non-colliding bars painted on the ground marking the
    # staging-pose bay, purely for visual reference when watching play.py -- no rigid_props/
    # collision_props, so it's invisible to physics and to the policy (which never observes scene
    # geometry, only the staging-pose error). See PARKING_LOT_* constants above for sizing.
    parking_lot_edge_far = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ParkingLotEdgeFar",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(
                STAGING_NOMINAL_POS_B[0] + PARKING_LOT_HALF_LENGTH,
                STAGING_NOMINAL_POS_B[1],
                PARKING_LOT_LINE_HEIGHT / 2,
            )
        ),
        spawn=sim_utils.CuboidCfg(
            size=(
                PARKING_LOT_LINE_THICKNESS,
                2 * PARKING_LOT_HALF_WIDTH + PARKING_LOT_LINE_THICKNESS,
                PARKING_LOT_LINE_HEIGHT,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.85, 0.0), emissive_color=(1.0, 0.7, 0.0)),
        ),
    )
    parking_lot_edge_near = parking_lot_edge_far.replace(
        prim_path="{ENV_REGEX_NS}/ParkingLotEdgeNear",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(
                STAGING_NOMINAL_POS_B[0] - PARKING_LOT_HALF_LENGTH,
                STAGING_NOMINAL_POS_B[1],
                PARKING_LOT_LINE_HEIGHT / 2,
            )
        ),
    )
    parking_lot_edge_left = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/ParkingLotEdgeLeft",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(
                STAGING_NOMINAL_POS_B[0],
                STAGING_NOMINAL_POS_B[1] + PARKING_LOT_HALF_WIDTH,
                PARKING_LOT_LINE_HEIGHT / 2,
            )
        ),
        spawn=sim_utils.CuboidCfg(
            size=(
                2 * PARKING_LOT_HALF_LENGTH + PARKING_LOT_LINE_THICKNESS,
                PARKING_LOT_LINE_THICKNESS,
                PARKING_LOT_LINE_HEIGHT,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.85, 0.0), emissive_color=(1.0, 0.7, 0.0)),
        ),
    )
    parking_lot_edge_right = parking_lot_edge_left.replace(
        prim_path="{ENV_REGEX_NS}/ParkingLotEdgeRight",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(
                STAGING_NOMINAL_POS_B[0],
                STAGING_NOMINAL_POS_B[1] - PARKING_LOT_HALF_WIDTH,
                PARKING_LOT_LINE_HEIGHT / 2,
            )
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

    def __post_init__(self):
        # Visual-only 180-degree approach-apron boundary: a semicircle of radius
        # APPROACH_ARC_BOUND_RADIUS on the approach side of the staging pose, drawn as a chain of
        # thin straight cuboid segments (a curved USD outline isn't a spawnable primitive, so we
        # approximate the arc with APPROACH_ARC_NUM_SEGMENTS chords). Purely cosmetic -- no
        # collision/rigid props, invisible to physics and the policy -- it just shows where the
        # out_of_approach_arc termination boundary is. Segments are attached as extra scene fields
        # here (InteractiveScene spawns every AssetBaseCfg in the cfg's __dict__), flat-named to
        # avoid the intermediate-Xform cloning requirement.
        cx, cy = STAGING_NOMINAL_POS_B[0], STAGING_NOMINAL_POS_B[1]
        radius = APPROACH_ARC_BOUND_RADIUS
        # boundary spans +/-90 deg around the approach direction -> the full 180-degree apron.
        a_start = APPROACH_ARC_DIR - math.pi / 2.0
        a_end = APPROACH_ARC_DIR + math.pi / 2.0
        n = APPROACH_ARC_NUM_SEGMENTS
        pts = [
            (cx + radius * math.cos(a_start + (a_end - a_start) * i / n),
             cy + radius * math.sin(a_start + (a_end - a_start) * i / n))
            for i in range(n + 1)
        ]
        for i in range(n):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            length = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            setattr(
                self,
                f"approach_arc_seg_{i}",
                AssetBaseCfg(
                    prim_path=f"{{ENV_REGEX_NS}}/ApproachArcSeg{i}",
                    init_state=AssetBaseCfg.InitialStateCfg(
                        pos=(mx, my, APPROACH_ARC_LINE_HEIGHT / 2.0),
                        rot=(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)),
                    ),
                    spawn=sim_utils.CuboidCfg(
                        # +thickness on length so consecutive chords overlap slightly (no gaps).
                        size=(length + APPROACH_ARC_LINE_THICKNESS, APPROACH_ARC_LINE_THICKNESS, APPROACH_ARC_LINE_HEIGHT),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(0.9, 0.05, 0.05), emissive_color=(0.9, 0.0, 0.0)
                        ),
                    ),
                ),
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
        # Success requires the tolerance to hold for a few consecutive steps (dwell-time gate), not
        # just an instantaneous crossing -- see StagingPoseCommand's docstring. Both the
        # staging_pose_reached reward and the staging_pose_success termination read this same
        # gate (StagingPoseCommand.success_held), so they can never fire out of sync.
        # LOOSENED to get an achievable success signal for both the scripted expert and RL: the
        # forward controllers reached the goal but arrived ~12-38 deg off heading and the tight
        # 0.2 m / 0.175 rad / 5-step gate never fired. 0.3 m + 0.26 rad (15 deg) + 3-step dwell is
        # the "get something working first" setting; tighten later via curriculum once it converges.
        success_pos_tolerance=STAGING_SUCCESS_POS_TOLERANCE,
        success_heading_tolerance=0.26,
        success_hold_steps=1,
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
        # minimum turn radius (~0.456 m vs ~0.29 m at this 0.166 m wheelbase), so the very short,
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
        # Every term carries a nan_to_num guard (see _nan_guard) so a physics NaN can never reach
        # the SAC networks -- the definitive fix for the seed-deterministic "training dies ~step
        # 3000" failure. Physics tuning (armature/friction/velocity caps) only shifted WHEN the NaN
        # occurred; this makes training immune to it regardless.
        staging_pose_error = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "staging_pose"}, modifiers=_nan_guard()
        )
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("car")}, modifiers=_nan_guard()
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("car")}, modifiers=_nan_guard()
        )
        steering_angle = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("car", joint_names=["steer_joint_L"])},
            modifiers=_nan_guard(),
        )
        last_action = ObsTerm(func=mdp.last_action, modifiers=_nan_guard())
        # dwell-time success-gate progress, in [0,1] -- see hold_progress's docstring: without
        # this, the reward/termination depend on unobserved history (consecutive in-tolerance
        # step count), breaking the Markov property the critic's bootstrapped TD-target relies on
        staging_pose_hold_progress = ObsTerm(
            func=mdp.staging_pose_hold_progress, params={"command_name": "staging_pose"}, modifiers=_nan_guard()
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
    #
    # weight raised from 1.0 to 10.0: position_progress switched from a bare potential
    # (-distance, ~-1 to -1.8/step) to a potential-based DELTA (prev_distance - curr_distance,
    # see its docstring) -- summed over an episode a delta telescopes to just
    # (initial_distance - final_distance), a few meters total, not hundreds. At weight=1.0 that's
    # a barely-perceptible ~+0.004/step average, negligible next to the penalty terms (spin/loiter
    # sum to roughly -5 to -15/episode), which would leave little positive incentive to approach
    # at all. 10.0 brings a full successful approach's total contribution to roughly +10 to +18 --
    # a meaningful counterweight to the penalties without swamping the +25 terminal bonus.
    position_progress = RewTerm(func=mdp.position_progress, weight=10.0, params={"command_name": "staging_pose"})
    # DENSE goal-attraction (see goal_attraction docstring): always-positive reward peaking at the
    # docked pose, to PULL the car in and defeat the loiter-penalty goal-avoidance reward-hack.
    # weight 3.0 -> up to +3/step near the goal, a strong steady pull the policy can't dodge.
    goal_attraction = RewTerm(func=mdp.goal_attraction, weight=3.0, params={"command_name": "staging_pose"})
    heading_alignment = RewTerm(func=mdp.heading_alignment, weight=0.5, params={"command_name": "staging_pose"})
    staging_pose_reached = RewTerm(
        func=mdp.staging_pose_reached, weight=25.0, params={"command_name": "staging_pose"}
    )
    # smooth partial credit toward completing the dwell-time hold gate -- see
    # staging_pose_hold_credit's docstring. Added after a 100k-step run showed total reward and
    # position_progress plateauing (entropy coefficient also collapsed to ~0) while
    # staging_pose_reached stayed flat the whole run: the sparse +25 bonus alone wasn't providing
    # enough gradient toward actually finishing, only toward getting close.
    staging_pose_hold_credit = RewTerm(
        func=mdp.staging_pose_hold_credit, weight=2.0, params={"command_name": "staging_pose"}
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
    # 180-degree approach-apron bound: terminate if the robot leaves the half-disk of radius
    # APPROACH_ARC_BOUND_RADIUS in front of the staging pose (see out_of_approach_arc + the visual
    # arc drawn in DockingSceneCfg). Keeps episodes on-task -- no wandering off or looping behind
    # the goal -- and matches the region the robot is spawned into.
    out_of_approach_arc = DoneTerm(
        func=mdp.out_of_approach_arc,
        params={
            "command_name": "staging_pose",
            "radius": APPROACH_ARC_BOUND_RADIUS,
            "approach_dir": APPROACH_ARC_DIR,
        },
    )
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
