from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Build the SAME full config as mtc_demo.launch.py so mtc_node's internal
    # MoveGroup/executor knows about the controllers for trajectory execution.

    # controllers_yaml = os.path.join(
    #     get_package_share_directory("so_arm_moveit_config"),
    #     "config", "moveit_controllers.yaml"
    # )
    # with open(controllers_yaml, 'r') as f:
    #     controllers = yaml.safe_load(f)
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
            # controllers,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])