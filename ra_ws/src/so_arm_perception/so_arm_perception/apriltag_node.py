# apriltag_node.py
#
# Continuously detects the AprilTag in the arm (eye-in-hand) camera and publishes
# its pose, both in the camera optical frame and transformed to the world frame.
# The world pose is what mtc_node consumes: it marks the water-dispenser paddle,
# so the grasped-cup centre is driven to (tag + standoff) and then advanced in.
#
# Runs independently of perception_node so the cup-detection / servo loop is left
# untouched.
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)

from so_arm_perception.apriltag_detector import AprilTagDetector


def _mat_to_quat(R):
    """Rotation matrix -> (x, y, z, w) quaternion."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class AprilTagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")

        # ACTIVE camera that drives /apriltag/pose + /apriltag/pixel. top_cam (fixed
        # eye-to-hand) localizes the 5 cm tag stably regardless of arm pose; arm_cam
        # (eye-in-hand) is used for closed-loop centering on the tag once the cup is
        # grasped. Both cameras are subscribed always; `active_camera` (runtime-
        # switchable, like perception_node) picks which one is processed.
        self.declare_parameter("active_camera", "top_cam")
        self.declare_parameter("eth_ns",        "top_cam")   # eye-to-hand camera ns
        self.declare_parameter("eih_ns",        "arm_cam")   # eye-in-hand  camera ns
        self.declare_parameter("tag_size",    0.05)        # metres (5 cm)
        self.declare_parameter("tag_family",  "36h11")
        self.declare_parameter("world_frame", "world")
        # -1 = publish whichever tag has the lowest id; otherwise lock to this id.
        self.declare_parameter("target_id",   -1)

        self.active_camera = self.get_parameter("active_camera").value
        eth_ns           = self.get_parameter("eth_ns").value
        eih_ns           = self.get_parameter("eih_ns").value
        tag_size         = self.get_parameter("tag_size").value
        family           = self.get_parameter("tag_family").value
        self.world_frame = self.get_parameter("world_frame").value
        self.target_id   = int(self.get_parameter("target_id").value)

        # Map the friendly labels used in `active_camera` to the actual namespaces.
        self._ns = {"top_cam": eth_ns, "arm_cam": eih_ns}

        self.detector = AprilTagDetector(tag_size_m=tag_size, family=family)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Broadcast world -> apriltag_<id> so the tag's pose is a TF frame (RViz TF
        # display, and any consumer can look up where the tag is). The tag is
        # static, so the last broadcast remains valid even after it leaves the FOV.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Per-camera latest frames / intrinsics (keyed by label, like perception_node).
        self.K        = {"top_cam": None, "arm_cam": None}
        self.frame_id = {"top_cam": None, "arm_cam": None}
        self.rgb      = {"top_cam": None, "arm_cam": None}
        self.depth    = {"top_cam": None, "arm_cam": None}

        for label, ns in self._ns.items():
            self.create_subscription(
                Image, f"/{ns}/rgb", lambda m, l=label: self._rgb_cb(m, l), 10)
            self.create_subscription(
                Image, f"/{ns}/depth", lambda m, l=label: self._depth_cb(m, l), 10)
            self.create_subscription(
                CameraInfo, f"/{ns}/camera_info", lambda m, l=label: self._info_cb(m, l), 10)

        self.pose_world_pub = self.create_publisher(PoseStamped, "/apriltag/pose", 10)
        self.pose_cam_pub   = self.create_publisher(PoseStamped, "/apriltag/pose_cam", 10)
        self.pixel_pub      = self.create_publisher(Float32MultiArray, "/apriltag/pixel", 10)
        self.marker_pub     = self.create_publisher(Marker, "/apriltag/marker", 10)
        self.debug_pub      = self.create_publisher(Image, "/apriltag/debug_image", 10)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(0.1, self._loop)   # 10 Hz
        self.get_logger().info(
            f"apriltag_node ready (active={self.active_camera}, "
            f"top_cam={eth_ns}, arm_cam={eih_ns}, family={family}, size={tag_size} m)")

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "active_camera":
                if p.value not in ("top_cam", "arm_cam"):
                    return SetParametersResult(
                        successful=False,
                        reason="active_camera must be 'top_cam' or 'arm_cam'")
                self.active_camera = p.value
                self.get_logger().info(f"active_camera -> {p.value}")
        return SetParametersResult(successful=True)

    def _rgb_cb(self, msg, label):
        self.rgb[label] = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _depth_cb(self, msg, label):
        d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if d.dtype == np.uint16:
            d = d.astype(np.float32) / 1000.0
        self.depth[label] = d

    def _info_cb(self, msg, label):
        self.K[label] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.frame_id[label] = msg.header.frame_id

    def _depth_at(self, depth, u, v, patch=4):
        """Median metric depth in a small patch around (u, v), or None."""
        if depth is None:
            return None
        v0, v1 = max(0, v - patch), v + patch
        u0, u1 = max(0, u - patch), u + patch
        p = depth[v0:v1, u0:u1]
        valid = p[(p > 0.1) & (p < 5.0)]
        return float(np.median(valid)) if valid.size else None

    def _loop(self):
        cam = self.active_camera
        rgb, depth = self.rgb[cam], self.depth[cam]
        K, frame_id = self.K[cam], self.frame_id[cam]
        if rgb is None or K is None:
            self.get_logger().warn(f"waiting for {cam} rgb/camera_info...", once=True)
            return

        detections = self.detector.detect(rgb)

        # Always publish a debug image (even with no detection).
        vis = self.detector.draw(rgb, detections, K)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(
            np.asarray(vis, dtype=np.uint8), encoding="rgb8"))

        if not detections:
            return

        # Pick the target tag (specific id, else lowest id).
        if self.target_id >= 0:
            picks = [d for d in detections if d["id"] == self.target_id]
            if not picks:
                return
            det = picks[0]
        else:
            det = min(detections, key=lambda d: d["id"])

        # Publish the tag PIXEL first (before depth/TF, which can fail) so an image-
        # space servo (arm-cam centering) keeps getting fixes. [u, v, img_w, img_h].
        img_h, img_w = rgb.shape[:2]
        px = Float32MultiArray()
        px.data = [float(det["u"]), float(det["v"]), float(img_w), float(img_h)]
        self.pixel_pub.publish(px)

        pose = self.detector.estimate_pose(det["corners"], K)
        if pose is None:
            self.get_logger().warn("solvePnP failed for detected tag")
            return
        rvec, _tvec_pnp = pose

        # POSITION from the DEPTH BUFFER, not solvePnP. solvePnP infers distance
        # from the tag's apparent SIZE, so a small error in the configured tag
        # size scales the whole translation (seen here: z=1.82 m vs true 1.37 m →
        # ~45 cm world error). The sim depth buffer is geometrically exact, so we
        # unproject the tag-centre pixel through it (median patch) → 2 mm accuracy.
        u, v = int(round(det["u"])), int(round(det["v"]))
        z = self._depth_at(depth, u, v)
        if z is None:
            self.get_logger().warn("no valid depth at tag centre", throttle_duration_sec=2.0)
            return
        x_cam = (u - K[0, 2]) * z / K[0, 0]
        y_cam = (v - K[1, 2]) * z / K[1, 1]

        # ORIENTATION from solvePnP (scale-invariant → unaffected by tag size).
        # Both position and orientation are then converted from the OpenCV optical
        # frame (x right, y DOWN, z FORWARD) to the *_camera TF frame's Isaac/
        # OpenGL body convention (x right, y UP, z BACK) via F = diag(1,-1,-1)
        # (= Rx 180) — the same flip unprojector.py's depth path applies.
        F = np.diag([1.0, -1.0, -1.0])
        R_body = F @ cv2.Rodrigues(rvec)[0]
        pos_body = F @ np.array([x_cam, y_cam, z])   # = [x_cam, -y_cam, -z]
        tvec = pos_body.reshape(3, 1)
        qx, qy, qz, qw = _mat_to_quat(R_body)
        stamp = self.get_clock().now().to_msg()

        # Pose in the camera optical frame.
        pc = PoseStamped()
        pc.header.stamp = stamp
        pc.header.frame_id = frame_id
        pc.pose.position.x = float(tvec[0, 0])
        pc.pose.position.y = float(tvec[1, 0])
        pc.pose.position.z = float(tvec[2, 0])
        pc.pose.orientation.x, pc.pose.orientation.y = qx, qy
        pc.pose.orientation.z, pc.pose.orientation.w = qz, qw
        self.pose_cam_pub.publish(pc)

        self.get_logger().debug(
            f"tag id={det['id']} cam=[{tvec[0,0]:.3f},{tvec[1,0]:.3f},{tvec[2,0]:.3f}] "
            f"px=({det['u']:.0f},{det['v']:.0f})",
            throttle_duration_sec=2.0)

        # Transform to world. stamp=Time(0) → latest available TF (avoids
        # future-extrapolation when sim/clock skew, fine for a slow target).
        pc_latest = PoseStamped()
        pc_latest.header.frame_id = frame_id
        pc_latest.pose = pc.pose
        try:
            pw = self.tf_buffer.transform(
                pc_latest, self.world_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f"TF {frame_id}->{self.world_frame} failed: {e}",
                throttle_duration_sec=5.0)
            return

        pw.header.stamp = stamp
        self.pose_world_pub.publish(pw)
        self._broadcast_tf(pw, det["id"])
        self._publish_marker(pw)
        self.get_logger().debug(
            f"tag id={det['id']} world=[{pw.pose.position.x:.3f},"
            f"{pw.pose.position.y:.3f},{pw.pose.position.z:.3f}]",
            throttle_duration_sec=2.0)

    def _broadcast_tf(self, pw: PoseStamped, tag_id: int):
        """Broadcast world -> apriltag_<id> from the world pose."""
        t = TransformStamped()
        t.header.stamp = pw.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = f"apriltag_{tag_id}"
        t.transform.translation.x = pw.pose.position.x
        t.transform.translation.y = pw.pose.position.y
        t.transform.translation.z = pw.pose.position.z
        t.transform.rotation = pw.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _publish_marker(self, pw: PoseStamped):
        m = Marker()
        m.header = pw.header
        m.ns = "apriltag"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pw.pose
        m.scale.x = self.detector.tag_size
        m.scale.y = self.detector.tag_size
        m.scale.z = 0.001
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.0, 0.9
        m.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(AprilTagNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()