from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value='/ros2_ws/maps/test_map_v1.yaml'),
        DeclareLaunchArgument('params_file',
            default_value='/ros2_ws/jetracer_nav2.yaml'),

        # static odom → base_link (no encoders)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','odom','base_link']),

        # static base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.10','0','0.05','0','0','0',
                       'base_link','laser']),

        # map server — serves the saved map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_file}]),

        # AMCL — localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file]),

        # controller server — pure pursuit
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file]),

        # planner server — NavFn
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file]),

        # BT navigator — action interface
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params_file]),

        # lifecycle manager — manages all nodes above
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                ]}]),
    ])
