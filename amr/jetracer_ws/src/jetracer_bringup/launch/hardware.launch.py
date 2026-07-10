from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

# Default GStreamer pipeline for the CSI IMX219 (overridable via $GSCAM_CONFIG).
# Output RGB to match gscam2's rgb8 encoding -- gscam2 does NOT accept bgr8, and
# a BGR pipeline fails to link to its appsink. If you set $GSCAM_CONFIG yourself,
# end it in format=RGB too (or unset it to use this default).
GSCAM_DEFAULT = (
    'nvarguscamerasrc sensor-id=0 sensor-mode=2 ! '
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=15/1,format=NV12 ! '
    # Downscale on the ISP to 640x360 (half res, same 16:9 aspect): ~4x less
    # rectify/apriltag CPU and smaller frames, so detection keeps up during the
    # dock approach. Calibration (config/imx219.yaml) MUST match this resolution.
    'nvvidconv ! video/x-raw,format=BGRx,width=640,height=360 ! videoconvert ! '
    'video/x-raw,format=RGB ! queue max-size-buffers=2 leaky=downstream'
)


def generate_launch_description():
    """Hardware + localization layer only (no Nav2).

    Brings up the base driver, RPLidar, EKF and the static TFs, so you get
    /odom, /imu, /scan and /odometry/filtered without the navigation stack.
    """
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
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
                'angle_compensate': True,
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

        # ---------------- Camera + AprilTag dock detection ----------------
        # static TF: base_footprint -> camera_link (from URDF camera_joint).
        # Mirrors jetracer_description/urdf/jetracer.urdf; measure the real
        # CSI camera mount and correct these -- docking accuracy depends on it.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_camera',
            arguments=['0.12', '0', '0.07', '0', '0.25', '0',
                       'base_footprint', 'camera_link'],
            output='screen'),
        # static TF: camera_link -> camera_optical_frame (optical convention).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_camera_optical',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'camera_link', 'camera_optical_frame'],
            output='screen'),

        # CSI IMX219 (gscam2) + rectify (image_proc) + AprilTag detector, all
        # composed in ONE container so frames pass intra-process (zero-copy).
        # gscam2 used to run as a standalone Node, which serialized every frame
        # across the process boundary into the container and starved the
        # detector (~1 Hz) -- the cause of "Lost detection / timeout exceeded".
        # AprilTag publishes a TF per tag (dock_0/1/2); dock_pose_publisher (nav
        # layer) turns that into /detected_dock_pose for opennav_docking.
        ComposableNodeContainer(
            name='apriltag_container',
            namespace='',
            package='rclcpp_components',
            # Multithreaded executor: gscam, rectify and apriltag each get their
            # own thread, so the heavy detector can't monopolise one core and
            # starve the camera feed. Single-threaded 'component_container'
            # saturated one core (~88%) at ~0.3 Hz and the image_rect/camera_info
            # exact-time synchroniser never paired (0 synchronized pairs).
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='gscam2',
                    plugin='gscam2::GSCamNode',
                    name='gscam_main',
                    parameters=[{
                        'gscam_config': EnvironmentVariable(
                            'GSCAM_CONFIG', default_value=GSCAM_DEFAULT),
                        # False: stamp image AND camera_info with the same ROS
                        # clock time so rectify/apriltag's image<->camera_info
                        # sync succeeds. With True (gstreamer time) the stamps
                        # never match and frames drop.
                        'use_gst_timestamps': False,
                        'sync_sink': False,
                        'camera_name': 'imx219',
                        'frame_id': 'camera_optical_frame',
                        # Permanent, version-controlled calibration matching the
                        # 640x360 pipeline output. Overrides ~/.ros/camera_info.
                        # Two variants live side-by-side: swap this filename to
                        # switch. imx219_inferred.yaml = rough spec-inferred (wide
                        # lens, zero distortion); imx219_measured.yaml = oST result.
                        'camera_info_url':
                            'file:///ros2_ws/src/jetracer_bringup/config/imx219_inferred.yaml',
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify',
                    remappings=[
                        ('image', '/image_raw'),
                        ('camera_info', '/camera_info'),
                        ('image_rect', '/image_rect'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    remappings=[
                        ('image_rect', '/image_rect'),
                        ('camera_info', '/camera_info'),
                    ],
                    parameters=[
                        '/ros2_ws/src/jetracer_bringup/config/dock_tags_36h11.yaml'],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ]),
    ])
