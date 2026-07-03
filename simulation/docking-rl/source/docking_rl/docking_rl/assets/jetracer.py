"""Configuration for the Waveshare JetRacer ROS AI Kit.

Matches the geometry and joint layout authored in ``usd/jetracer_docking_scene.usda``:
single front-steering servo (``steer_joint_L/R``, both commanded the same angle -- the real
hardware has one servo and a tie-rod, not independent Ackermann geometry) and independent
rear-wheel drive motors (``wheel_joint_RL/RR``). Front wheels (``wheel_joint_F[LR]``) free-spin
and are intentionally left out of ``actuators``.
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

USD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "usd"))

JETRACER_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/JetRacer",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(USD_DIR, "jetracer_docking_scene.usda"),
        activate_contact_sensors=True,
        # Stability overrides. The steering knuckles and front wheels are co-located, and the
        # wheels sit close to the chassis, so self-collision must be OFF or the articulation
        # jitters itself apart ("jiggling on the spot"). Bump solver iterations and cap
        # depenetration velocity so the tiny (20 g) wheels don't get flung on any spawn overlap.
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=1.0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # z chosen so the wheels (radius 0.0325, centre at chassis_z - 0.03) rest just on the
        # ground (wheel bottom = z - 0.0625) rather than spawning penetrated.
        pos=(0.0, 0.0, 0.063),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["steer_joint_.*"],
            stiffness=50.0,
            damping=5.0,
            effort_limit_sim=2.0,
        ),
        "drive": ImplicitActuatorCfg(
            joint_names_expr=["wheel_joint_R[LR]"],  # rear driven wheels only
            stiffness=0.0,
            damping=10.0,
            # NOTE: velocity_limit is silently ignored by implicit actuators; the drive command
            # (AckermannActionCfg.drive_velocity_limit) is the actual velocity target. effort is
            # what caps wheel torque -- too low and the wheels can't reach the commanded speed
            # (slip), too high and they spin up before traction catches (also slip).
            effort_limit_sim=5.0,
        ),
    },
)
"""JetRacer articulation config: single-servo Ackermann-like front steering + rear differential drive."""
