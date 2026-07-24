"""One-command real-hardware bring-up for the full grasp pipeline.

Composes what were 5 terminals into one:
  1. bringup_real.launch.py   — real arm (Feetech), move_group, controllers, RViz
  2. two usb_camera_node       — arm_cam + top_cam (calibrated intrinsics + undistort)
  3. perception.launch.py      — YOLO + ray-plane, calibrated top_cam extrinsic
  4. fake /apriltag/pose       — stand-in dispenser marker (toggle with fake_apriltag)
  5. pick_place_demo.launch.py — mtc_node pipeline (toggle with run_mtc)

    ros2 launch mtc_tutorial real_all.launch.py

Args:
  run_mtc:=false        bring up everything EXCEPT the arm-moving pipeline, so you can
                        sanity-check /detected_object/position before it plans. Then
                        start the pipeline yourself: ros2 launch mtc_tutorial pick_place_demo.launch.py use_sim_time:=false
  fake_apriltag:=false  don't publish the fake dispenser pose (use a real tag instead).
  tag_x/tag_y/tag_z     fake dispenser marker world position (default 0.20, 0.0, 0.15).
  mtc_delay             seconds to wait before starting mtc_node, so move_group is up
                        first (default 10).

All nodes are forced onto ROS_DOMAIN_ID=10 + CycloneDDS so it works regardless of the
terminal's env. Camera device paths are the stable by-id links (survive re-plugging);
edit them here if the hardware changes. Calibration values: see
so_arm_perception reference-real-camera-bringup memory.
"""
import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, TimerAction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Stable by-id device links (arm_cam icSpring has no serial suffix; top_cam does).
ARM_CAM_DEV = "/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0"
TOP_CAM_DEV = "/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0"


def generate_launch_description():
    mtc_share  = get_package_share_directory("mtc_tutorial")
    perc_share = get_package_share_directory("so_arm_perception")

    run_mtc       = LaunchConfiguration("run_mtc")
    fake_apriltag = LaunchConfiguration("fake_apriltag")
    tag_x = LaunchConfiguration("tag_x")
    tag_y = LaunchConfiguration("tag_y")
    tag_z = LaunchConfiguration("tag_z")
    mtc_delay = LaunchConfiguration("mtc_delay")

    # 1. Real arm + MoveIt (move_group carries the loosened execution-duration watchdog).
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mtc_share, "launch", "bringup_real.launch.py")))

    # 2. Cameras. Gated by run_sensing so a fake_object demo can drop them entirely
    #    (otherwise perception would also publish /detected_object/position and fight
    #    the fake pose).
    run_sensing = LaunchConfiguration("run_sensing")
    arm_cam = Node(
        condition=IfCondition(run_sensing),
        package="so_arm_perception", executable="usb_camera_node", name="arm_cam_node",
        output="screen",
        parameters=[{"video_device": ARM_CAM_DEV, "camera_ns": "arm_cam"}])
    top_cam = Node(
        condition=IfCondition(run_sensing),
        package="so_arm_perception", executable="usb_camera_node", name="top_cam_node",
        output="screen",
        parameters=[{
            "video_device": TOP_CAM_DEV, "camera_ns": "top_cam", "frame_id": "top_sim_camera",
            "fx": 418.38762, "fy": 416.14640, "cx": 325.19068, "cy": 233.94865,
            "undistort": True,
            "d0": -0.300890, "d1": 0.078304, "d2": 0.001265, "d3": -0.001828, "d4": 0.0,
        }])

    # 3. Perception with the calibrated top_cam extrinsic + bias, YOLO on both cams.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perc_share, "launch", "perception.launch.py")),
        condition=IfCondition(run_sensing),
        launch_arguments={
            "use_sim_time": "false",
            "top_cam_use_green": "false", "arm_cam_use_green": "false",
            # Corrected frame: perception "world" was rotated -90 deg about Z from the
            # true base frame (front=-Y). Rotated the AprilTag-solved extrinsic by
            # -90 deg: pos (x,y)->(y,-x), yaw -= 90 deg. Bias reset (old -0.047 was in
            # the broken frame) — re-derive after verifying the front cup reads ~(0,-0.20).
            "eth_x": "0.17855", "eth_y": "-0.28819", "eth_z": "0.21155",
            "eth_roll": "0.4944438", "eth_pitch": "0.0766640", "eth_yaw": "0.4961072",
            "eth_plane_z": LaunchConfiguration("eth_plane_z"),
            "eth_x_correction": LaunchConfiguration("eth_x_correction"),
            "eth_y_correction": LaunchConfiguration("eth_y_correction"),
        }.items())

    # 4. Fake dispenser AprilTag (stand-in until a real tag is used).
    fake_tag = ExecuteProcess(
        condition=IfCondition(fake_apriltag),
        cmd=["ros2", "topic", "pub", "-r", "1", "/apriltag/pose",
             "geometry_msgs/msg/PoseStamped",
             ["{header: {frame_id: 'world'}, pose: {position: {x: ", tag_x,
              ", y: ", tag_y, ", z: ", tag_z, "}, orientation: {w: 1.0}}}"]],
        output="screen")

    # 5. The pipeline — delayed so move_group/controllers are up first. mtc_node then
    #    blocks on /detected_object/position, so a little extra slack is harmless.
    mtc = TimerAction(period=mtc_delay, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mtc_share, "launch", "pick_place_demo.launch.py")),
            launch_arguments={"use_sim_time": "false",
                              "place_at_pickup": LaunchConfiguration("place_at_pickup"),
                              "bridge_standoff": LaunchConfiguration("bridge_standoff"),
                              "grasp_yaw_bias": LaunchConfiguration("grasp_yaw_bias"),
                              "servo_img_p1_pullback": LaunchConfiguration("servo_img_p1_pullback"),
                              "servo_grasp_z": LaunchConfiguration("servo_grasp_z"),
                              "servo_img_u_offset_px": LaunchConfiguration("servo_img_u_offset_px"),
                              "skip_servo": LaunchConfiguration("skip_servo"),
                              "skip_servo_speed": LaunchConfiguration("skip_servo_speed"),
                              "place_z": LaunchConfiguration("place_z")}.items(),
            condition=IfCondition(run_mtc))])

    # 4b. Fake object pose (stand-in for perception) — a PREDEFINED cup position so
    #     the pick is deterministic with no camera. mtc_node reads this on
    #     /detected_object/position exactly like a real detection. Pair with
    #     skip_servo:=true for a fully repeatable, camera-free demo.
    fake_object = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("fake_object")),
        cmd=["ros2", "topic", "pub", "-r", "2", "/detected_object/position",
             "geometry_msgs/msg/PointStamped",
             ["{header: {frame_id: 'world'}, point: {x: ", LaunchConfiguration("obj_x"),
              ", y: ", LaunchConfiguration("obj_y"),
              ", z: ", LaunchConfiguration("obj_z"), "}}"]],
        output="screen")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "10"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"),
        DeclareLaunchArgument("run_mtc", default_value="true",
                              description="Auto-start the arm-moving mtc_node pipeline. "
                                          "false = bring up sensing only for a dry check."),
        DeclareLaunchArgument("fake_apriltag", default_value="true"),
        DeclareLaunchArgument("place_at_pickup", default_value="false",
                              description="Place the cup back where it was detected "
                                          "instead of the pink tray."),
        DeclareLaunchArgument("bridge_standoff", default_value="0.08",
                              description="Pre-grasp claw pull-back (m). 0.08 for the "
                                          "close real cup (0.16 collapses onto the base)."),
        DeclareLaunchArgument("grasp_yaw_bias", default_value="-0.2618",
                              description="Gripper facing offset from cup center (rad). "
                                          "-0.2618 = 15 deg (was -0.4 = 23 deg). Aims to one "
                                          "side to pin the cup against the fixed jaw."),
        DeclareLaunchArgument("eth_plane_z", default_value="0.04",
                              description="Ray-plane height (m) = cup grasp mid-height. "
                                          "LOWER it to push far detections further out."),
        DeclareLaunchArgument("eth_x_correction", default_value="0.02",
                              description="Constant offset (m) added to the published top_cam x. "
                                          "+0.02 shifts the detection 2 cm to the robot's RIGHT "
                                          "(+X), correcting a ~2 cm left bias."),
        DeclareLaunchArgument("eth_y_correction", default_value="-0.07",
                              description="Constant offset (m) added to the published top_cam y. "
                                          "Front is -Y, so a NEGATIVE value pushes the cup "
                                          "further out (e.g. -0.07 turns -0.20 into -0.27). "
                                          "Larger pushback also gives the pre-grasp standoff "
                                          "more clearance from the cup collision cylinder."),
        DeclareLaunchArgument("servo_img_u_offset_px", default_value="125.0",
                              description="Horizontal servo setpoint offset from image centre "
                                          "(px, + = aim cup RIGHT of centre for the side-grasp "
                                          "jaw offset). Sim used 280 -> targets u=600 on a "
                                          "640-wide image, unreachable -> never converges. 40 "
                                          "aims slightly right; tune to the real camera/gripper "
                                          "geometry (jog to a good grasp, read u, offset=u-320)."),
        DeclareLaunchArgument("servo_img_p1_pullback", default_value="-0.04",
                              description="m the phase-1 approach stops short of the grasp "
                                          "distance. NEGATIVE = drive further forward before "
                                          "closing (counters top_cam under-reading distance). "
                                          "-0.04 drives ~4 cm past the perceived grasp point."),
        DeclareLaunchArgument("servo_grasp_z", default_value="0.052",
                              description="Grasp height (m, base frame). Pins BOTH the servo "
                                          "target z and the MTC standoff z. 0.052 grabs slightly "
                                          "below the 0.05986 mid-height default."),
        DeclareLaunchArgument("tag_x", default_value="0.0"),
        DeclareLaunchArgument("tag_y", default_value="-0.25"),
        DeclareLaunchArgument("tag_z", default_value="0.20"),
        DeclareLaunchArgument("mtc_delay", default_value="10.0"),
        DeclareLaunchArgument("skip_servo", default_value="false",
                              description="Skip the visual servo — open-loop straight-in "
                                          "grasp using the (predefined) object pose. For "
                                          "a deterministic, camera-free demo."),
        DeclareLaunchArgument("skip_servo_speed", default_value="0.06",
                              description="m/s of the open-loop straight-in approach "
                                          "(skip_servo:=true)."),
        DeclareLaunchArgument("place_z", default_value="1e9",
                              description="Absolute destination release height (m). "
                                          "1e9 = unset. Set e.g. 0.04 to drop the cup at z=0.04."),
        DeclareLaunchArgument("run_sensing", default_value="true",
                              description="Bring up the cameras + perception. Set false for "
                                          "a fake_object demo so nothing competes on "
                                          "/detected_object/position."),
        DeclareLaunchArgument("fake_object", default_value="false",
                              description="Publish a PREDEFINED cup pose on "
                                          "/detected_object/position (no perception). "
                                          "Pair with skip_servo:=true + run_sensing:=false "
                                          "for a repeatable demo."),
        DeclareLaunchArgument("obj_x", default_value="0.0",
                              description="Fake cup world x (m). Front is -Y, right is +X."),
        DeclareLaunchArgument("obj_y", default_value="-0.30",
                              description="Fake cup world y (m). Front of the base is -Y. "
                                          "Placed further out so the 0.08 standoff clears "
                                          "the base instead of collapsing onto it."),
        DeclareLaunchArgument("obj_z", default_value="0.05986",
                              description="Fake cup world z (m) = grasp mid-height."),
        bringup,
        arm_cam,
        top_cam,
        perception,
        fake_tag,
        fake_object,
        mtc,
    ])