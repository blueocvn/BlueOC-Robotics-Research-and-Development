# bringup.launch.py — one command to start the whole refill stack.
#
# Replaces running these three by hand:
#   ros2 launch mtc_tutorial      mtc_demo.launch.py        (move_group + controllers + RViz + servo)
#   ros2 launch so_arm_perception perception.launch.py      (YOLO cup + AprilTag perception)
#   ros2 launch mtc_tutorial      pick_place_demo.launch.py (mtc_node: the grasp→fill→place pipeline)
#
# They are STAGGERED with TimerActions so the dependencies are up before the next
# layer starts: move_group/controllers first, perception next, then mtc_node (which
# plans against move_group and waits on /detected_object/position from perception).
# Any launch argument of pick_place_demo (servo_*, grasp_*, task_flow, dispenser_*)
# can be forwarded here via launch_arguments={...} on the pick_place include below,
# or just overridden on the command line: `ros2 launch mtc_tutorial bringup.launch.py task_flow:=place`.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mtc_launch = os.path.join(get_package_share_directory("mtc_tutorial"), "launch")
    perception_launch = os.path.join(
        get_package_share_directory("so_arm_perception"), "launch"
    )

    # Layer 1: MoveIt move_group, ros2_control + controllers, RViz, servo.
    mtc_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mtc_launch, "mtc_demo.launch.py"))
    )

    # Layer 2: perception (cup detector + AprilTag). Cameras/TF only — no hard
    # dependency on move_group, a short delay just keeps the console readable.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_launch, "perception.launch.py")
        )
    )

    # The commonly-tuned pick/fill args, declared here so `ros2 launch ...
    # bringup.launch.py --show-args` lists them and no "undeclared argument"
    # warning fires. Defaults MUST match pick_place_demo.launch.py. Any OTHER
    # pick_place arg still propagates via the shared launch context when passed
    # on the command line.
    tuning_args = [
        DeclareLaunchArgument("grasp_yaw_bias", default_value="-0.5"),
        DeclareLaunchArgument("servo_grasp_z", default_value="0.05986"),
        DeclareLaunchArgument("dispenser_standoff", default_value="0.10"),
        DeclareLaunchArgument("dispenser_fill_depth", default_value="-0.08"),
        DeclareLaunchArgument("skip_servo", default_value="false"),
    ]

    # Layer 3: mtc_node — the grasp→fill→place pipeline. Needs move_group + the
    # spawned controllers (loaded at ~3 s in mtc_demo) and perception's
    # /detected_object/position, so start it last. The tuning args above are
    # forwarded explicitly; add more to launch_arguments below to bake them in.
    pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mtc_launch, "pick_place_demo.launch.py")),
        launch_arguments={
            "grasp_yaw_bias": LaunchConfiguration("grasp_yaw_bias"),
            "servo_grasp_z": LaunchConfiguration("servo_grasp_z"),
            "dispenser_standoff": LaunchConfiguration("dispenser_standoff"),
            "dispenser_fill_depth": LaunchConfiguration("dispenser_fill_depth"),
            "skip_servo": LaunchConfiguration("skip_servo"),
        }.items(),
    )

    return LaunchDescription(
        tuning_args
        + [
            mtc_demo,
            TimerAction(period=3.0, actions=[perception]),
            TimerAction(period=10.0, actions=[pick_place]),
        ]
    )