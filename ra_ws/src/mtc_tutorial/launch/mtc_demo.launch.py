import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction


def generate_launch_description():
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_moveit_config")
        .robot_description(file_path="config/so101_new_calib.urdf.xacro")
        .robot_description_semantic(file_path="config/so101_new_calib.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": True},
            {"move_group.default_planning_pipeline": "ompl"},
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("so_arm_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
        parameters=[{"use_sim_time": True}],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("so_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path, {"use_sim_time": True}],
        
        output="both",
    )

    # Load controllers
    load_controllers = [
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner joint_state_broadcaster --ros-args -p use_sim_time:=true"],
                    shell=True,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner arm_group_controller --ros-args -p use_sim_time:=true"],
                    shell=True,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner hand_group_controller --ros-args -p use_sim_time:=true"],
                    shell=True,
                    output="screen",
                ),
            ]
        )
    ]


    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )
