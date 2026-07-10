from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/ros2_ws/maps/fake_map.yaml',
            description='Full path to map yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/ros2_ws/src/jetracer_bringup/config/jetracer_nav2.yaml',
            description='Full path to Nav2 params yaml'),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value='/ros2_ws/src/jetracer_driver/config/ekf.yaml',
            description='Full path to robot_localization EKF params yaml'),
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the JetRacer base controller'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyACM1',
            description='Serial port for the RPLidar'),

        # hardware driver: /cmd_vel -> serial, publishes /odom (twist) and /imu
        Node(
            package='jetracer_driver',
            executable='jetracer_driver',
            name='jetracer_driver',
            output='screen',
            parameters=[{'port': base_port}]),

        # EKF: fuses /odom + /imu, publishes odom -> base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file]),

        # RPLidar A1: publishes /scan in frame laser_frame
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': False,
                'scan_mode': 'Standard',
                'scan_frequency': 10.0,
            }]),

        # static TF: base_footprint -> laser_frame
        # z = 0.18 (lidar mount height); yaw = pi because the lidar is
        # mounted inverted on the Waveshare JetRacer.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_laser',
            arguments=['0.0', '0.0', '0.18', '3.14159', '0.0', '0.0',
                       'base_footprint', 'laser_frame'],
            output='screen'),

        # static TF: base_footprint -> base_imu_link (needed so EKF can
        # transform the driver's IMU data into the base frame).
        # z = tire radius (0.0325) + imu mount height (0.02) = 0.0525
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_imu',
            arguments=['0.02', '0', '0.0525', '0', '0', '0',
                       'base_footprint', 'base_imu_link'],
            output='screen'),

        # map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}]),

        # amcl localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]),

        # controller server (Regulated Pure Pursuit)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]),

        # planner server (NavFn)
        # DIAGNOSTIC: wrapped in gdb to capture a backtrace on the SIGSEGV.
        # On crash, the backtrace prints in this node's screen output.
        # Remove the `prefix=` line once the crash is diagnosed.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]),

        # behavior server (spin/back up/wait recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]),

        # bt navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]),

        # lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                # bond_timeout=0.0 disables the bond check. The Jetson is too
                # slow to bond bt_navigator within the 4s default, which made
                # the lifecycle manager kill the whole stack.
                'bond_timeout': 0.0,
                'node_names': lifecycle_nodes,
            }]),
    ])
