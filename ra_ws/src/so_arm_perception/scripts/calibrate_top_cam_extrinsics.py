#!/usr/bin/python3
"""Solve the top_cam eye-to-hand extrinsic (world -> camera TF) from a single
AprilTag lying flat on the table at a KNOWN world pose.

Run under the SYSTEM python (ROS deps): /usr/bin/python3, with ROS sourced.

    source /opt/ros/jazzy/setup.bash
    source ~/ra_ws/install/setup.bash
    /usr/bin/python3 src/so_arm_perception/scripts/calibrate_top_cam_extrinsics.py \
        --ros-args -p tag_x:=0.15 -p tag_y:=0.0 -p tag_z:=0.0 -p tag_size:=0.05

What it does
------------
1. Subscribes to /<camera_ns>/rgb + /<camera_ns>/camera_info (so it uses the SAME
   intrinsics the perception node will — calibrate those FIRST, Step 1).
2. Detects the tag (pupil_apriltags, pose estimation on) -> T_camCV_tag, the tag
   pose in the OpenCV optical frame (x-right, y-DOWN, z-FORWARD).
3. Composes with the measured T_world_tag to get the camera pose in world, then
   converts OpenCV-optical -> the OpenGL/Isaac convention (x, -y, -z) that the
   ray-plane unprojector actually uses (see unprojector.pixel_ray_to_plane_world:
   dx=+, dy=-, dz=-1). This flip is why a naive solvePnP extrinsic comes out
   mirrored.
4. Averages over N frames (rotation via SVD) and prints the
   static_transform_publisher arguments to paste into perception.launch.py's
   eth_static_tf.

Tag placement convention (IMPORTANT — must match reality)
---------------------------------------------------------
Tag flat on the table, printed side up (facing the ceiling camera), with its
top edge pointing along world +X and its right edge along world +Y by default.
- tag_x/tag_y/tag_z: the tag CENTER in the world (robot-base) frame, metres.
- tag_yaw_deg: rotate this if you couldn't align the tag to the axes (deg, about
  world +Z).
- tag_z_up (default true): the tag's face-normal points UP toward the camera. If
  the recovered camera ends up BELOW the table / the verify step is mirrored,
  flip this to false.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

try:
    from pupil_apriltags import Detector
except ImportError:
    raise SystemExit("pupil_apriltags missing: /usr/bin/python3 -m pip install --user pupil-apriltags")


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def mat_to_rpy(R):
    """Extract roll,pitch,yaw for R = Rz(yaw) @ Ry(pitch) @ Rx(roll) — the
    convention tf2 static_transform_publisher's --roll/--pitch/--yaw use."""
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.hypot(R[2, 1], R[2, 2]))
    roll = np.arctan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


def avg_rotation(mats):
    """Mean rotation via SVD orthonormalization of the summed matrices."""
    M = np.sum(mats, axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:            # keep it a proper rotation
        U[:, -1] *= -1
        R = U @ Vt
    return R


class ExtrinsicSolver(Node):
    def __init__(self):
        super().__init__("calibrate_top_cam_extrinsics")
        self.camera_ns = self.declare_parameter("camera_ns", "top_cam").value
        self.child_frame = self.declare_parameter("child_frame", "top_sim_camera").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.tag_size = float(self.declare_parameter("tag_size", 0.05).value)
        self.tag_family = self.declare_parameter("tag_family", "tag36h11").value
        self.tag_x = float(self.declare_parameter("tag_x", 0.0).value)
        self.tag_y = float(self.declare_parameter("tag_y", 0.0).value)
        self.tag_z = float(self.declare_parameter("tag_z", 0.0).value)
        self.tag_yaw = np.deg2rad(float(self.declare_parameter("tag_yaw_deg", 0.0).value))
        self.tag_z_up = bool(self.declare_parameter("tag_z_up", True).value)
        self.num_frames = int(self.declare_parameter("num_frames", 30).value)

        # world <- tag rotation. Tag face-up: its normal (z) = world +Z; x->+X, y->+Y.
        base = np.eye(3) if self.tag_z_up else np.diag([1.0, -1.0, -1.0])
        self.R_wt = rot_z(self.tag_yaw) @ base
        self.t_wt = np.array([self.tag_x, self.tag_y, self.tag_z])

        self.bridge = CvBridge()
        self.K = None
        self.detector = Detector(families=self.tag_family)
        self.samples_t = []   # camera-in-world translations (GL frame)
        self.samples_R = []   # camera-in-world rotations (GL frame)

        self.create_subscription(CameraInfo, f"/{self.camera_ns}/camera_info",
                                 self._info_cb, 10)
        self.create_subscription(Image, f"/{self.camera_ns}/rgb", self._rgb_cb, 10)
        self.get_logger().info(
            f"Waiting for tag on /{self.camera_ns}/rgb "
            f"(size={self.tag_size} m, center world=({self.tag_x},{self.tag_y},{self.tag_z}))")

    def _info_cb(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx, fy, cx, cy

    def _rgb_cb(self, msg):
        if self.K is None or len(self.samples_t) >= self.num_frames:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        dets = self.detector.detect(gray, estimate_tag_pose=True,
                                    camera_params=self.K, tag_size=self.tag_size)
        if not dets:
            return
        d = max(dets, key=lambda x: x.decision_margin)     # best tag if several

        # T_camCV_tag : tag pose in the OpenCV optical frame.
        T_ct = np.eye(4)
        T_ct[:3, :3] = np.array(d.pose_R)
        T_ct[:3, 3] = np.array(d.pose_t).reshape(3)

        # T_world_tag (measured) and camera pose in world (still OpenCV optical).
        T_wt = np.eye(4)
        T_wt[:3, :3] = self.R_wt
        T_wt[:3, 3] = self.t_wt
        T_w_camCV = T_wt @ np.linalg.inv(T_ct)

        # OpenCV-optical -> OpenGL/Isaac frame the ray-plane code uses (x, -y, -z).
        flip = np.diag([1.0, -1.0, -1.0, 1.0])
        T_w_camGL = T_w_camCV @ flip

        self.samples_t.append(T_w_camGL[:3, 3].copy())
        self.samples_R.append(T_w_camGL[:3, :3].copy())
        n = len(self.samples_t)
        if n % 5 == 0 or n == self.num_frames:
            self.get_logger().info(f"collected {n}/{self.num_frames}")
        if n >= self.num_frames:
            self._report()
            rclpy.shutdown()

    def _report(self):
        t = np.mean(self.samples_t, axis=0)
        R = avg_rotation(self.samples_R)
        roll, pitch, yaw = mat_to_rpy(R)
        std = np.std(self.samples_t, axis=0)
        print("\n" + "=" * 68)
        print(f"  top_cam extrinsic  ({self.world_frame} -> {self.child_frame})")
        print("=" * 68)
        print(f"  camera position (world): x={t[0]:+.4f} y={t[1]:+.4f} z={t[2]:+.4f}")
        print(f"  position stddev over {len(self.samples_t)} frames: "
              f"{std[0]*1000:.1f}/{std[1]*1000:.1f}/{std[2]*1000:.1f} mm")
        if t[2] < 0:
            print("  !! camera z is BELOW the table — tag_z_up is probably wrong; "
                  "re-run with -p tag_z_up:=false")
        print("\n  Paste into eth_static_tf (perception.launch.py):")
        print(f'      "--x",     "{t[0]:.5f}",')
        print(f'      "--y",     "{t[1]:.5f}",')
        print(f'      "--z",     "{t[2]:.5f}",')
        print(f'      "--roll",  "{roll:.7f}",')
        print(f'      "--pitch", "{pitch:.7f}",')
        print(f'      "--yaw",   "{yaw:.7f}",')
        print(f'      "--frame-id",       "{self.world_frame}",')
        print(f'      "--child-frame-id", "{self.child_frame}",')
        print("\n  Or test it live first:")
        print(f"      ros2 run tf2_ros static_transform_publisher "
              f"--x {t[0]:.5f} --y {t[1]:.5f} --z {t[2]:.5f} "
              f"--roll {roll:.7f} --pitch {pitch:.7f} --yaw {yaw:.7f} "
              f"--frame-id {self.world_frame} --child-frame-id {self.child_frame}")
        print("=" * 68 + "\n")


def main():
    rclpy.init()
    node = ExtrinsicSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()