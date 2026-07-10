import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(
            get_package_share_directory("slam_custom"), "params", "slam_toolbox_params.yaml"
        ),
    )

    startup_delay = LaunchConfiguration("startup_delay", default="5.0")

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch"
    )
    rviz_config = os.path.join(
        get_package_share_directory("slam_custom"), "rviz", "slam_custom.rviz"
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="True",
            description="Use simulation clock if True"
        ),
        DeclareLaunchArgument(
            "slam_params_file", default_value=slam_params_file,
            description="Full path to slam_toolbox params file"
        ),
        DeclareLaunchArgument(
            "startup_delay", default_value="5.0",
            description="Seconds to wait before starting slam_toolbox (lets sim clock stabilise)"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        TimerAction(period=startup_delay, actions=[slam_include]),
    ])
