"""One-command top_cam + YOLO + live bbox viewer (no arm, no motion).

Just to SEE the cup detection. Starts:
  - top_cam usb_camera_node (calibrated intrinsics + undistort)
  - perception.launch.py (active_camera=top_cam, YOLO for the metal cup)
  - rqt_image_view on /perception/debug_image

The rqt window needs the sanitized snap env, so run the whole thing through it:

    env -i HOME=$HOME USER=$USER DISPLAY=:0 \
      XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.8BZXS3 \
      XDG_RUNTIME_DIR=/run/user/1000 PATH=/usr/bin:/bin QT_QPA_PLATFORM=xcb \
      bash -c 'source /opt/ros/jazzy/setup.bash; source ~/ra_ws/install/setup.bash; \
               ros2 launch so_arm_perception top_cam_view.launch.py'

Non-GUI nodes run fine under that env too, so one wrapper covers everything.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

TOP_CAM_DEV = "/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0"


def generate_launch_description():
    perc_share = get_package_share_directory("so_arm_perception")

    top_cam = Node(
        package="so_arm_perception", executable="usb_camera_node", name="top_cam_node",
        output="screen",
        parameters=[{
            "video_device": TOP_CAM_DEV, "camera_ns": "top_cam", "frame_id": "top_sim_camera",
            "fx": 418.38762, "fy": 416.14640, "cx": 325.19068, "cy": 233.94865,
            "undistort": True,
            "d0": -0.300890, "d1": 0.078304, "d2": 0.001265, "d3": -0.001828, "d4": 0.0,
        }])

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perc_share, "launch", "perception.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "active_camera": "top_cam",
            "top_cam_use_green": "false",
        }.items())

    viewer = Node(
        package="rqt_image_view", executable="rqt_image_view", name="bbox_viewer",
        arguments=["/perception/debug_image"], output="screen")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "10"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"),
        top_cam,
        perception,
        viewer,
    ])