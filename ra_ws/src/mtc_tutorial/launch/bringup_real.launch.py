"""Real-hardware bring-up for the SO-ARM101 follower.

Mirrors mtc_demo.launch.py but drives the PHYSICAL arm instead of Isaac Sim:
  * robot_description is generated with ros2_control_hardware_type:=feetech, so the
    ros2_control system uses feetech_ros2_driver/FeetechHardwareInterface (serial
    STS3215) instead of the topic_based Isaac bridge.
  * use_sim_time is False everywhere (the real arm runs on the wall clock).

The rest of the stack (move_group, controllers, RViz, TF) is identical to sim —
that is the whole point of ros2_control: only the hardware interface changes.

SAFETY: on first bring-up, verify /joint_states matches the real arm BEFORE
planning any motion (see follower_joints.yaml — homing_offset must map the URDF
zero pose correctly). The joint_state_broadcaster is read-only; the arm
controller only holds current position until MoveIt commands a new goal.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = False

    # planning_context — build robot_description for the REAL Feetech hardware
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so_arm_moveit_config")
        .robot_description(
            file_path="config/so101_new_calib.urdf.xacro",
            mappings={"ros2_control_hardware_type": "feetech"},
        )
        .robot_description_semantic(file_path="config/so101_new_calib.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Execute MTC solutions
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Real SO-ARM servos run at reduced moving_speed (see follower_joints.yaml), so
    # trajectories physically take LONGER than MoveIt's timing estimate. The default
    # execution-duration watchdog (scaling 1.2 + 0.5 s margin) then TIMES_OUT and
    # cancels the trajectory mid-execution (esp. the slow Jaw close). Loosen the
    # watchdog so the slow-but-valid motion is allowed to finish. Ceiling only —
    # this changes the timeout, NOT the commanded speed.
    slow_hardware_execution = {
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": use_sim_time},
            {"move_group.default_planning_pipeline": "ompl"},
            slow_hardware_execution,
        ],
    )

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
            {"use_sim_time": use_sim_time},
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ros2_control — real Feetech hardware (serial /dev/ttyACM0)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("so_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path,
                    {"use_sim_time": use_sim_time}],
        output="both",
    )

    # Load controllers (same set as sim — controllers are hardware-agnostic)
    load_controllers = [
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner joint_state_broadcaster"],
                    shell=True,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner arm_group_controller"],
                    shell=True,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner hand_group_controller"],
                    shell=True,
                    output="screen",
                ),
            ],
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