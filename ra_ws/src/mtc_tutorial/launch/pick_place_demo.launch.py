from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # use_sim_time: true for Isaac Sim (default), false for the real arm.
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Visual-servo args (image-based / IBVS: closes the loop on arm-cam pixels) ─
    # Two phases: phase 0 aligns left/right on pixels, phase 1 approaches straight in
    # on the world distance. The k_yaw sign depends on the camera mount — flip it if
    # the arm drives the object OFF-frame toward the edge.
    servo_img_k_yaw       = LaunchConfiguration("servo_img_k_yaw")
    servo_img_yaw_step    = LaunchConfiguration("servo_img_yaw_step")
    servo_img_deadband_px = LaunchConfiguration("servo_img_deadband_px")
    servo_img_u_offset_px = LaunchConfiguration("servo_img_u_offset_px")
    servo_img_align_tol    = LaunchConfiguration("servo_img_align_tol")
    servo_img_align_ticks  = LaunchConfiguration("servo_img_align_ticks")
    servo_img_standback    = LaunchConfiguration("servo_img_standback")
    servo_img_approach_tol = LaunchConfiguration("servo_img_approach_tol")
    servo_img_done_px      = LaunchConfiguration("servo_img_done_px")
    servo_img_slow_px      = LaunchConfiguration("servo_img_slow_px")
    servo_img_slow_floor   = LaunchConfiguration("servo_img_slow_floor")
    servo_img_slow_pow     = LaunchConfiguration("servo_img_slow_pow")
    servo_img_stop_on_center = LaunchConfiguration("servo_img_stop_on_center")
    servo_img_bearing_range = LaunchConfiguration("servo_img_bearing_range")
    servo_img_aw           = LaunchConfiguration("servo_img_aw")
    servo_img_phase1_cartesian = LaunchConfiguration("servo_img_phase1_cartesian")
    servo_img_cart_speed   = LaunchConfiguration("servo_img_cart_speed")
    servo_img_cart_step    = LaunchConfiguration("servo_img_cart_step")
    servo_img_p1_pullback  = LaunchConfiguration("servo_img_p1_pullback")
    servo_img_p1_lateral   = LaunchConfiguration("servo_img_p1_lateral")
    servo_img_p1_yaw_scale = LaunchConfiguration("servo_img_p1_yaw_scale")
    servo_img_p1_depth_stop   = LaunchConfiguration("servo_img_p1_depth_stop")
    servo_img_p1_depth_stop_m = LaunchConfiguration("servo_img_p1_depth_stop_m")
    # Single-jaw grasp geometry: the SO-101 pins the object against its FIXED jaw,
    # so the approach faces the object at grasp_yaw_bias (gap-capture angle) and the
    # grasp spot is shifted laterally to the fixed-jaw face (grasp_tcp_lateral).
    grasp_yaw_bias         = LaunchConfiguration("grasp_yaw_bias")
    grasp_tcp_lateral      = LaunchConfiguration("grasp_tcp_lateral")
    # Side-grasp height used by BOTH the pre-phase-0 gross move and the servo,
    # so the gripper sits at this height BEFORE centering (no vertical pop).
    # Default below mug mid-height (0.05986) per request: sit at/below the cup.
    servo_grasp_z          = LaunchConfiguration("servo_grasp_z")
    dispenser_standoff     = LaunchConfiguration("dispenser_standoff")
    dispenser_fill_depth   = LaunchConfiguration("dispenser_fill_depth")
    dispenser_flip_normal  = LaunchConfiguration("dispenser_flip_normal")
    # Build the SAME full config as mtc_demo.launch.py so mtc_node's internal
    # MoveGroup/executor knows about the controllers for trajectory execution.
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_moveit_config")
        .robot_description(file_path="config/so101_new_calib.urdf.xacro")
        .robot_description_semantic(file_path="config/so101_new_calib.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"mode": "hybrid"},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"servo_img_k_yaw": ParameterValue(servo_img_k_yaw, value_type=float)},
            {"servo_img_yaw_step": ParameterValue(servo_img_yaw_step, value_type=float)},
            {"servo_img_deadband_px": ParameterValue(servo_img_deadband_px, value_type=float)},
            {"servo_img_u_offset_px": ParameterValue(servo_img_u_offset_px, value_type=float)},
            {"servo_img_align_tol": ParameterValue(servo_img_align_tol, value_type=float)},
            {"servo_img_align_ticks": ParameterValue(servo_img_align_ticks, value_type=float)},
            {"servo_img_standback": ParameterValue(servo_img_standback, value_type=float)},
            {"servo_img_approach_tol": ParameterValue(servo_img_approach_tol, value_type=float)},
            {"servo_img_done_px": ParameterValue(servo_img_done_px, value_type=float)},
            {"servo_img_slow_px": ParameterValue(servo_img_slow_px, value_type=float)},
            {"servo_img_slow_floor": ParameterValue(servo_img_slow_floor, value_type=float)},
            {"servo_img_slow_pow": ParameterValue(servo_img_slow_pow, value_type=float)},
            {"servo_img_stop_on_center": ParameterValue(servo_img_stop_on_center, value_type=float)},
            {"servo_img_bearing_range": ParameterValue(servo_img_bearing_range, value_type=float)},
            {"servo_img_aw": ParameterValue(servo_img_aw, value_type=float)},
            {"servo_img_phase1_cartesian": ParameterValue(servo_img_phase1_cartesian, value_type=bool)},
            {"servo_img_cart_speed": ParameterValue(servo_img_cart_speed, value_type=float)},
            {"servo_img_cart_step": ParameterValue(servo_img_cart_step, value_type=float)},
            {"servo_img_p1_pullback": ParameterValue(servo_img_p1_pullback, value_type=float)},
            {"servo_img_p1_lateral": ParameterValue(servo_img_p1_lateral, value_type=float)},
            {"servo_img_p1_yaw_scale": ParameterValue(servo_img_p1_yaw_scale, value_type=float)},
            {"servo_img_p1_depth_stop": ParameterValue(servo_img_p1_depth_stop, value_type=bool)},
            {"servo_img_p1_depth_stop_m": ParameterValue(servo_img_p1_depth_stop_m, value_type=float)},
            {"grasp_yaw_bias": ParameterValue(grasp_yaw_bias, value_type=float)},
            {"grasp_tcp_lateral": ParameterValue(grasp_tcp_lateral, value_type=float)},
            {"servo_grasp_z": ParameterValue(servo_grasp_z, value_type=float)},
            {"dispenser_standoff": ParameterValue(dispenser_standoff, value_type=float)},
            {"dispenser_fill_depth": ParameterValue(dispenser_fill_depth, value_type=float)},
            {"dispenser_flip_normal": ParameterValue(dispenser_flip_normal, value_type=bool)},
            {"place_at_pickup": ParameterValue(
                LaunchConfiguration("place_at_pickup"), value_type=bool)},
            {"place_at_pickup_z_offset": ParameterValue(
                LaunchConfiguration("place_at_pickup_z_offset"), value_type=float)},
            {"bridge_standoff": ParameterValue(
                LaunchConfiguration("bridge_standoff"), value_type=float)},
            {"skip_servo": ParameterValue(
                LaunchConfiguration("skip_servo"), value_type=bool)},
            {"skip_servo_speed": ParameterValue(
                LaunchConfiguration("skip_servo_speed"), value_type=float)},
            {"place_z": ParameterValue(
                LaunchConfiguration("place_z"), value_type=float)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="true = Isaac Sim clock; false = real arm (wall clock)"),
        DeclareLaunchArgument(
            "place_at_pickup", default_value="false",
            description="Place the cup back at its original detected position instead "
                        "of the pink tray (stand-in destination when no tray is set up)."),
        DeclareLaunchArgument(
            "place_at_pickup_z_offset", default_value="0.0",
            description="Extra z (m) added to the pickup height when placing back."),
        DeclareLaunchArgument(
            "bridge_standoff", default_value="0.16",
            description="Pre-grasp claw pull-back (m) from the cup before servoing. "
                        "0.16 suits the far sim cup; a CLOSE real cup needs ~0.08 or the "
                        "standoff collapses onto the base (GOAL_STATE_INVALID)."),
        DeclareLaunchArgument(
            "skip_servo", default_value="false",
            description="true = SKIP the visual servo. After the MTC standoff move, drive "
                        "the claw straight in to the (predefined) grasp point open-loop — "
                        "deterministic, no cameras/pixels. Pair with a fake "
                        "/detected_object/position for a fully repeatable demo."),
        DeclareLaunchArgument(
            "skip_servo_speed", default_value="0.06",
            description="m/s end-effector speed of the open-loop straight-in approach "
                        "(skip_servo:=true). Lower = slower/gentler final approach."),
        DeclareLaunchArgument(
            "place_z", default_value="1e9",
            description="Absolute destination release height (m, base frame). Overrides "
                        "the tray_z+offset / pickup-z computation. 1e9 = unset (default)."),
        DeclareLaunchArgument(
            "servo_img_k_yaw", default_value="-0.0010",
            description="rad of base bearing per pixel of horizontal error (sign matches arm-cam mount; flip if it diverges)"),
        DeclareLaunchArgument(
            "servo_img_yaw_step", default_value="0.020",
            description="phase 0: max rad of bearing change per tick; keep small so one "
                        "step moves the object less than servo_img_done_px or the loop "
                        "limit-cycles (raise to center faster, lower if it overshoots)"),
        DeclareLaunchArgument(
            "servo_img_deadband_px", default_value="26.0",
            description="phase 0: ignore horizontal pixel errors smaller than this (raise "
                        "to park the arm once roughly centered instead of nudging)"),
        DeclareLaunchArgument(
            "servo_img_u_offset_px", default_value="280.0",
            description="horizontal setpoint offset from image center in px (+ = aim right of center; for side-grasp gripper offset)"),
        DeclareLaunchArgument(
            "servo_img_align_tol", default_value="0.5",
            description="phase 0→1: joint max_err (rad) at which visual left/right alignment is done and the straight approach begins"),
        DeclareLaunchArgument(
            "servo_img_align_ticks", default_value="3.0",
            description="phase 0: net centered ticks (leaky counter) needed to confirm "
                        "alignment and hand off; lower to commit faster through the "
                        "residual centering jitter"),
        DeclareLaunchArgument(
            "servo_img_standback", default_value="0.07",
            description="m held behind the grasp distance during phase 0; phase 1 then "
                        "approaches this gap straight in. Raise so phase 0 centers further "
                        "back (0 = center at the grasp distance, almost no approach phase)"),
        DeclareLaunchArgument(
            "servo_img_approach_tol", default_value="0.2",
            description="phase 1: joint max_err (rad) at which the straight position approach is done (grasp distance reached)"),
        DeclareLaunchArgument(
            "servo_img_stop_on_center", default_value="0.0",
            description="1.0 = HALT the servo the instant the blob is centered "
                        "(phase 0), skipping the phase-1 straight-in approach; "
                        "0.0 = normal (center then approach). Note: it stops at the "
                        "phase-0 reach, held back by servo_img_standback — set that "
                        "to 0 to stop at the cup, not short of it"),
        DeclareLaunchArgument(
            "servo_img_done_px", default_value="88.0",
            description="phase 0: |dx| (px) under which the object counts as centered and hands off to the approach; raise to accept the side-grasp offset residual and finish alignment faster"),
        DeclareLaunchArgument(
            "servo_img_slow_px", default_value="150.0",
            description="phase 0: within this |dx| (px) of the setpoint, taper the "
                        "per-tick yaw step down so the base SLOWS as the object nears "
                        "centre (0 = no taper, full speed until the deadband)"),
        DeclareLaunchArgument(
            "servo_img_slow_floor", default_value="0.2",
            description="phase 0: minimum fraction of servo_img_yaw_step kept inside "
                        "the slow zone, so the turn still creeps in to centre (raise "
                        "to finish faster, lower for a gentler final crawl)"),
        DeclareLaunchArgument(
            "servo_img_slow_pow", default_value="2.0",
            description="phase 0: taper curve exponent (|dx|/slow_px)^pow. 1.0 = linear; "
                        ">1 bends the curve so the turn slows HARDER the closer the "
                        "object gets to centre (raise for a gentler final creep, e.g. 3.0)"),
        DeclareLaunchArgument(
            "servo_img_bearing_range", default_value="0.8",
            description="max rad the alignment bearing may swing from its seed; raise if the bearing pins at the clamp before centering"),
        DeclareLaunchArgument(
            "servo_img_aw", default_value="1.0",
            description="phase-0 anti-windup (back-calculation) strength: scales the "
                        "bearing integrator step by (1 - aw*joint_err/align_tol), so "
                        "the command can't wind ahead of a lagging arm and overshoot "
                        "centre. Only modulates rate (floored at 0.1, never freezes); "
                        "0 = plain integrator. Raise for a softer/cleaner stop, lower "
                        "if centring feels throttled"),
        DeclareLaunchArgument(
            "servo_img_phase1_cartesian", default_value="true",
            description="phase 1: true = stream ONE straight-line Cartesian approach "
                        "(smooth, continuous); false = the old per-tick reach integrator + async IK"),
        DeclareLaunchArgument(
            "servo_img_cart_speed", default_value="0.11",
            description="phase 1 cartesian: end-effector speed (m/s) along the approach line; lower = slower/gentler approach"),
        DeclareLaunchArgument(
            "servo_img_cart_step", default_value="0.01",
            description="phase 1 cartesian: spacing (m) between sampled waypoints along the line; smaller = straighter path, more IK calls"),
        DeclareLaunchArgument(
            "servo_img_p1_pullback", default_value="0.0",
            description="m the phase-1 straight-in approach stops SHORT of the grasp distance "
                        "(on top of servo_standoff). Raise to reduce how far it travels toward the cup"),
        DeclareLaunchArgument(
            "servo_img_p1_lateral", default_value="0.01",
            description="m lateral bias on the phase-1 approach target, perpendicular to the cup "
                        "bearing: + = LEFT of the approach heading, - = right. Nudges the grasp "
                        "off the detected centre. Flip the sign if it biases the wrong way"),
        DeclareLaunchArgument(
            "servo_img_p1_depth_stop", default_value="false",
            description="if true, PHASE 1 halts the straight-in approach the instant the "
                        "live arm_cam perceived range (/detected_object/depth) drops below "
                        "servo_img_p1_depth_stop_m (closed-loop on the camera). If false, "
                        "the open-loop joint-arrival stop is used"),
        DeclareLaunchArgument(
            "servo_img_p1_depth_stop_m", default_value="0.1",
            description="m perceived arm_cam range at which the phase-1 approach stops "
                        "(only when servo_img_p1_depth_stop:=true). Lower = approach closer"),
        DeclareLaunchArgument(
            "servo_img_p1_yaw_scale", default_value="0.4",
            description="scale (0..1) on grasp_yaw_bias for the PHASE-1 approach only. "
                        "1.0 = full jaw-gap bias (~29° off the cup bearing); lower it to "
                        "make the straight-in direction point more directly at the cup "
                        "(0 = dead straight). Bridge/phase-0 facing keep the full bias"),
        DeclareLaunchArgument(
            "grasp_yaw_bias", default_value="-0.5",
            description="rad the approach faces RIGHT of the object so it lands IN THE "
                        "GAP between the SO-101's fixed and moving jaw (NOT dead-centre, "
                        "which swats it with the fixed jaw). -0.5 ≈ 29°; try -0.785 for "
                        "45°. Shared by gross-move, servo, place, dispenser"),
        DeclareLaunchArgument(
            "grasp_tcp_lateral", default_value="0.0",
            description="m the grasp spot is shifted ⊥ to the facing (+ = robot-left) so "
                        "the object seats against the FIXED jaw face, not the gap centre. "
                        "0 = old centre aim; dial in (~jaw half-gap) if the object lands "
                        "off the fixed jaw"),
        DeclareLaunchArgument(
            "servo_grasp_z", default_value="0.05986",
            description="m world height of the level side-grasp, used by BOTH the "
                        "pre-phase-0 gross move and the servo so the gripper is at this "
                        "height BEFORE centering. Mug spans base 0.01486..top 0.10486; "
                        "default 0.05986 = mug mid-height (cup height). Lower toward the "
                        "base for a lower grasp (watch gripper-table clearance)"),
        DeclareLaunchArgument(
            "dispenser_standoff", default_value="0.10",
            description="m the cup centre stands off from the tag along the tag normal before advancing in"),
        DeclareLaunchArgument(
            "dispenser_fill_depth", default_value="-0.08",
            description="m the cup ends relative to the tag along the normal: NEGATIVE = stops SHORT "
                        "(shorter forward stroke); 0 = at the tag; POSITIVE = press past the paddle"),
        DeclareLaunchArgument(
            "dispenser_flip_normal", default_value="false",
            description="flip the tag normal if the arm approaches from the wrong side (solvePnP sign ambiguity)"),
        pick_place_demo,
    ])