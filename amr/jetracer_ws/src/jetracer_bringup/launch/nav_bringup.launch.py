from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Nav2 stack only (no hardware).

    Assumes the hardware layer (driver + lidar + EKF + static TFs) is already
    running -- see hardware.launch.py. Brings up map_server, AMCL, controller,
    planner, bt_navigator and the lifecycle manager.
    """
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    docking_params_file = LaunchConfiguration('docking_params_file')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/ros2_ws/maps/test_map_outer_v6.yaml',
            description='Full path to map yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/ros2_ws/src/jetracer_bringup/config/jetracer_nav2.yaml',
            description='Full path to Nav2 params yaml'),

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

        # smoother server (constrained smoother) — optimises the Hybrid-A* path
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file]),

        # planner server (Smac Hybrid-A*)
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

        # jetracer_docker: named-dock Pure Pursuit docking (replaces opennav_docking).
        # Phase 1: Nav2 staging navigation to the selected dock's map pose.
        # Phase 2: Pure Pursuit approach to that dock's AprilTag.
        # Trigger: ros2 topic pub --once /dock_robot std_msgs/msg/String '{data: "dock1"}'
        # Undock:  ros2 topic pub --once /undock_robot std_msgs/msg/Bool '{data: true}'
        #
        # dock_pose_x/y are surveyed map positions, parallel to dock_ids:
        #   dock0 = water station (1.690, -0.756),
        #   dock1 = (-6.615, -2.162), dock2 = (-6.571, 2.824).
        Node(
            package='jetracer_bringup',
            executable='jetracer_docker.py',
            name='jetracer_docker',
            output='screen',
            parameters=[{
                'wheelbase':         0.165,   # measured rear-axle to front-axle
                'delta_max_deg':     30.0,    # circle test ~28deg; 30 rough estimate
                'approach_speed':    0.10,
                'docking_threshold': 0.15,
                'detection_timeout': 0.5,
                'staging_distance':  2.0,    # ~2.06 m surveyed staging in front of dock0
                'skip_staging':      False,
                # Long approach runs on the smooth forward-only transit planner;
                # hand off to Hybrid-A* only within this distance of the staging
                # pose, where the reversing/heading lineup is needed.
                'last_mile_dist':    1.5,
                'steer_gain':        2.0,
                'blind_dock_distance': 0.45,
                # Abort a dock attempt if the tag isn't re-detected within this
                # many seconds mid-approach (0 = wait forever). Keeps a failed
                # dock from wedging the node / stalling a sequence.
                'approach_lost_timeout': 15.0,
                'dock_ids':          ['dock0', 'dock1', 'dock2'],
                'dock_tag_frames':   ['dock_0', 'dock_1', 'dock_2'],
                'dock_pose_x':       [1.69034, -6.61477, -6.57088],
                'dock_pose_y':       [-0.756399, -2.16233, 2.82399],
                # Robot heading (map yaw) when docked, all surveyed via 2D Pose
                # Estimate: dock0 ~ -pi/2 (-Y), dock1/dock2 ~ +/-pi (-X).
                'dock_pose_yaw':     [-1.57838, -3.11913, 3.12996],
                # Per-dock staging distance (m) in front of the dock, derived from
                # the surveyed staging poses: dock0 ~0.82, dock1 ~1.01, dock2 ~1.11.
                'dock_staging_dist': [0.82, 1.01, 1.11],
                'undock_distance':   0.5,
                'undock_speed':      0.10,
            }]),

        # dock pose publisher: apriltag TF (dock_0/1/2) -> /detected_dock_pose
        Node(
            package='jetracer_bringup',
            executable='dock_pose_publisher.py',
            name='dock_pose_publisher',
            output='screen',
            parameters=[{
                'camera_frame': 'camera_optical_frame',
                'dock_frames': ['dock_0', 'dock_1', 'dock_2'],
                'detection_topic': 'detected_dock_pose',
            }]),

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
