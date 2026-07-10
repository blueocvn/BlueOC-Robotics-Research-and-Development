from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Autonomous MAPPING stack (no hardware, no localization).

    For building a map with a live SLAM stack running on ANOTHER machine that
    publishes /map and the map->odom TF. Differs from nav_bringup.launch.py:

      * NO map_server / amcl -- the global costmap's static_layer takes /map
        straight from SLAM, and SLAM (not amcl) provides map->odom.
      * NO docking nodes -- mapping only.
      * ADDS explore_lite, which reads the Nav2 global costmap, picks frontiers
        and sends NavigateToPose goals to bt_navigator to drive the robot into
        unexplored space.

    Assumes the hardware layer (driver + lidar + EKF + static TFs) is already
    running (start_hardware.sh) and SLAM is up and publishing /map + TF.
    """
    params_file = LaunchConfiguration('params_file')
    explore_params_file = LaunchConfiguration('explore_params_file')

    # Same Nav2 motion stack as nav_bringup, minus map_server and amcl.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/ros2_ws/src/jetracer_bringup/config/jetracer_mapping.yaml',
            description='Full path to Nav2 params yaml (mapping-tuned)'),
        DeclareLaunchArgument(
            'explore_params_file',
            default_value='/ros2_ws/src/jetracer_bringup/config/explore.yaml',
            description='Full path to explore_lite params yaml'),

        # controller server (Regulated Pure Pursuit)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]),

        # planner server (NavFn)
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

        # lifecycle manager (motion stack only -- no map_server/amcl here)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                # bond_timeout=0.0 disables the bond check -- the Jetson is too
                # slow to bond bt_navigator within the 4s default.
                'bond_timeout': 0.0,
                'node_names': lifecycle_nodes,
            }]),

        # explore_lite: frontier-based autonomous exploration. Not a lifecycle
        # node -- it just starts exploring once the costmap + NavigateToPose
        # action are available.
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params_file]),
    ])
