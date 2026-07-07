# launch/tracking.launch.py
#
# Closed-loop visual servoing controller for the SO-ARM 101.
# Consumes /detected_object/position (from perception_node) and streams
# position-only IK joint targets to the arm controller.
#
# Run perception_node (and your move_group / controllers) first, then:
#   ros2 launch so_arm_perception tracking.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "group": "arm_group",
        "ik_link": "gripper",
        "base_frame": "world",
        "object_topic": "/detected_object/position",
        "command_topic": "/arm_group_controller/joint_trajectory",
        "rate": "10.0",
        "standoff": "0.12",
        "z_offset": "0.0",
        "gain": "0.35",
        "max_joint_step": "0.15",
        "target_ema": "0.4",
        "pos_deadband": "0.01",
        "joint_deadband": "0.01",
        "target_timeout": "1.0",
        "avoid_collisions": "true",
        "use_sim_time": "true",
    }

    tracking_node = Node(
        package="so_arm_perception",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
        parameters=[{k: LaunchConfiguration(k) for k in args}],
    )

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
        + [tracking_node]
    )
