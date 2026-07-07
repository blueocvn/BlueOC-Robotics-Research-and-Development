# launch/perception.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Configurable args ──────────────────────────────────────
    camera_eth_ns   = LaunchConfiguration("camera_eth_ns",   default="top_cam")
    camera_eih_ns   = LaunchConfiguration("camera_eih_ns",   default="arm_cam")
    target_classes  = LaunchConfiguration("target_classes",  default="cup,bottle")
    yolo_model      = LaunchConfiguration("yolo_model",      default="yolo11n.pt")
    base_frame      = LaunchConfiguration("base_frame",      default="world")
    conf_threshold  = LaunchConfiguration("conf_threshold",  default="0.25")
    active_camera   = LaunchConfiguration("active_camera",   default="top_cam")
    # MUST match the rest of the stack (Isaac publishes /clock). Without sim time,
    # TF lookups against the robot's sim-time TF tree fail with "extrapolation
    # into the future".
    use_sim_time    = LaunchConfiguration("use_sim_time",    default="true")

    # Terminal verbosity toggle (DEFAULT OFF): when false, all perception node output
    # (detector prints + logger) is routed to the launch LOG FILES instead of the
    # terminal, keeping the console quiet. Set perception_verbose:=true to see it.
    perception_verbose = LaunchConfiguration("perception_verbose", default="false")
    node_output = PythonExpression(
        ["'screen' if '", perception_verbose,
         "'.lower() in ('true', '1', 'yes', 'on') else 'log'"]
    )

    # Eye-in-hand camera mount: gripper -> arm_sim_camera.
    # arm_sim_camera uses the Isaac/OpenGL camera convention (+x right, +y up,
    # +z BACKWARD), which is exactly the Isaac camera prim's local frame — so
    # these values are the camera prim's local transform relative to the gripper
    # link, readable from Isaac's Property panel. CALIBRATE THESE.
    eih_x     = LaunchConfiguration("eih_x",     default="0.0")
    eih_y     = LaunchConfiguration("eih_y",     default="0.0")
    eih_z     = LaunchConfiguration("eih_z",     default="0.05")
    eih_roll  = LaunchConfiguration("eih_roll",  default="0.0")
    eih_pitch = LaunchConfiguration("eih_pitch", default="0.0")
    eih_yaw   = LaunchConfiguration("eih_yaw",   default="0.0")

    # Eye-to-hand static transform
    # Adjust x, y, z, roll, pitch, yaw to match your camera placement
    eth_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="eth_static_tf",
        # Top_cam prim transform from Isaac's Property panel (pos + XYZ Euler):
        # pos (0.02967, 0.31201, 0.87595), rot (-37.913, 0, 180) deg.
        # Isaac's XYZ-Euler composes as Rx*Rz, which under the 180 yaw equals
        # tf2's RPY with roll = -Isaac_roll (the roll SIGN FLIPS). So Isaac -37.913
        # -> tf2 roll +37.913 deg = 0.661703 rad. (Was Isaac -45 -> +45.)
        arguments=[
            "--x",     "0.02967",
            "--y",     "0.31201",
            "--z",     "0.87595",
            "--roll",  "0.6617028",       # +37.913 deg (= Isaac -37.913 under the 180 yaw)
            "--pitch", "0.0",             #   0 deg
            "--yaw",   "3.1415926536",    # 180 deg
            "--frame-id",       "world",
            "--child-frame-id", "top_sim_camera",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output=node_output,
    )

    # Eye-in-hand static transform: gripper -> arm_sim_camera (camera moves with
    # the gripper, so this is a STATIC mount offset, NOT world-anchored).
    eih_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="eih_static_tf",
        arguments=[
            "--x",     eih_x,
            "--y",     eih_y,
            "--z",     eih_z,
            "--roll",  eih_roll,
            "--pitch", eih_pitch,
            "--yaw",   eih_yaw,
            "--frame-id",       "gripper",
            "--child-frame-id", "arm_sim_camera",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output=node_output,
    )

    # ── AprilTag args (dispenser paddle marker) ───────────────────────────────
    # A 5 cm tag is decodable from the repositioned top_cam (~1.1 m), so the
    # fixed eye-to-hand view localizes it stably regardless of arm pose. Switch to
    # arm_cam for eye-in-hand experiments (or for a tag too small for top_cam).
    apriltag_camera = LaunchConfiguration("apriltag_camera", default="top_cam")
    apriltag_size   = LaunchConfiguration("apriltag_size",   default="0.05")
    apriltag_family = LaunchConfiguration("apriltag_family", default="36h11")

    apriltag_node = Node(
        package="so_arm_perception",
        executable="apriltag_node",
        name="apriltag_node",
        output=node_output,
        parameters=[{
            "active_camera": apriltag_camera,   # top_cam | arm_cam (runtime-switchable)
            "tag_size":    apriltag_size,
            "tag_family":  apriltag_family,
            "world_frame": base_frame,
            "use_sim_time": use_sim_time,
        }],
    )

    # Cup-handle orientation estimator (reorient tool). OFF by default — it runs
    # only when you're working the top-grip/wrist-roll reorient. It needs the
    # eth_static_tf (top_sim_camera->world) above and sim time, both inherited
    # here, which is why it lives in this launch rather than a bare `ros2 run`.
    enable_handle_detector = LaunchConfiguration("handle_detector", default="false")
    handle_detector_node = Node(
        package="so_arm_perception",
        executable="handle_detector",
        name="handle_detector",
        output=node_output,
        condition=IfCondition(enable_handle_detector),
        parameters=[{
            "camera_ns":  camera_eth_ns,
            "base_frame": base_frame,
            "use_sim_time": use_sim_time,
        }],
    )

    # Main perception node
    perception_node = Node(
        package="so_arm_perception",
        executable="perception_node",
        name="perception_node",
        output=node_output,
        parameters=[{
            "camera_eth_ns":   camera_eth_ns,
            "camera_eih_ns":   camera_eih_ns,
            "target_classes":  target_classes,
            "yolo_model":      yolo_model,
            "base_frame":      base_frame,
            "conf_threshold":  conf_threshold,
            "active_camera":   active_camera,
            "use_sim_time":    use_sim_time,
            "eth_use_ray_plane": LaunchConfiguration("eth_use_ray_plane", default="true"),
            "eth_plane_z":       LaunchConfiguration("eth_plane_z", default="0.05986"),
        }],
        remappings=[
            # Remap if your Isaac Sim topic names differ
            # ("from_topic", "to_topic"),
        ],
    )

    return LaunchDescription([
        # Args
        DeclareLaunchArgument("camera_eth_ns",  default_value="top_cam",
                              description="Eye-to-hand camera namespace"),
        DeclareLaunchArgument("camera_eih_ns",  default_value="arm_cam",
                              description="Eye-in-hand camera namespace"),
        DeclareLaunchArgument("target_classes", default_value="cup,bottle",
                              description="Comma-separated YOLO class names to detect"),
        DeclareLaunchArgument("yolo_model",     default_value="yolo11n.pt",
                              description="YOLO model path or name"),
        DeclareLaunchArgument("base_frame",     default_value="world",
                              description="Robot base TF frame"),
        DeclareLaunchArgument("conf_threshold", default_value="0.25",
                              description="YOLO confidence threshold"),
        DeclareLaunchArgument("active_camera",  default_value="top_cam",
                              description="Detection source: top_cam or arm_cam"),
        DeclareLaunchArgument("eth_use_ray_plane", default_value="true",
                              description="top_cam: intersect the detection-pixel ray with z=eth_plane_z "
                                          "instead of unprojecting the depth buffer (removes the oblique "
                                          "surface-depth y-bias). arm_cam always uses depth."),
        DeclareLaunchArgument("eth_plane_z",       default_value="0.05986",
                              description="Horizontal plane height (m, world) for top_cam ray-plane: the "
                                          "mug grasp mid-height CUP_BASE_Z+MUG_HEIGHT/2 = 0.05986."),
        DeclareLaunchArgument("use_sim_time",   default_value="true",
                              description="Use /clock sim time (required with Isaac)"),
        DeclareLaunchArgument("perception_verbose", default_value="false",
                              description="false (default) = perception node output goes to the "
                                          "log files (quiet terminal); true = print to terminal"),
        DeclareLaunchArgument("eih_x",     default_value="0.0"),
        DeclareLaunchArgument("eih_y",     default_value="0.0"),
        DeclareLaunchArgument("eih_z",     default_value="0.05"),
        DeclareLaunchArgument("eih_roll",  default_value="0.0"),
        DeclareLaunchArgument("eih_pitch", default_value="0.0"),
        DeclareLaunchArgument("eih_yaw",   default_value="0.0"),
        DeclareLaunchArgument("apriltag_camera", default_value="top_cam",
                              description="camera the AprilTag detector runs on (top_cam for the 5 cm tag; arm_cam for eye-in-hand)"),
        DeclareLaunchArgument("apriltag_size",   default_value="0.05",
                              description="AprilTag black-square edge length (m)"),
        DeclareLaunchArgument("apriltag_family", default_value="36h11",
                              description="AprilTag family (36h11 / 25h9 / 16h5)"),
        DeclareLaunchArgument("handle_detector", default_value="false",
                              description="Start the cup-handle orientation estimator "
                                          "(publishes /cup_handle/*). Needs the top_cam "
                                          "static TF from this launch."),

        LogInfo(msg="Starting SO-ARM 101 perception pipeline..."),
        eth_static_tf,
        eih_static_tf,
        perception_node,
        apriltag_node,
        handle_detector_node,
    ])