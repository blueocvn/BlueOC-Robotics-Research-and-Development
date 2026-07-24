#!/usr/bin/python3
"""Minimal USB (UVC) camera publisher for the SO-ARM real-hardware perception stack.

Publishes a V4L2 webcam to the SAME topics the perception_node subscribes to, so a
plain USB webcam drops straight into the pipeline:

    /<camera_ns>/rgb          sensor_msgs/Image   (rgb8)
    /<camera_ns>/camera_info  sensor_msgs/CameraInfo   (only if publish_camera_info)

For the eye-in-hand (arm_cam) IMAGE-SPACE servo phase, only /rgb is needed — depth and
camera_info/extrinsics are not required (see perception_node.py image-mode note). depth
is intentionally NOT published: a mono webcam has none, and arm_cam image servoing
doesn't use it.

Run one instance per physical camera, e.g.
    ros2 run so_arm_perception usb_camera_node --ros-args \
        -p video_device:=/dev/video2 -p camera_ns:=arm_cam

Parameters
----------
video_device          : str   V4L2 device path (default /dev/video2). Prefer a stable
                              /dev/v4l/by-id/... path so it survives re-plugging.
camera_ns             : str   Topic namespace -> /<ns>/rgb (default "arm_cam").
width, height         : int   Requested capture size (default 640x480).
fps                   : float Publish rate AND requested capture fps (default 30.0).
fourcc                : str   V4L2 pixel format, "MJPG" (default) or "YUYV". MJPG gives
                              higher fps at 640x480 on most UVC cams.
frame_id              : str   Image header.frame_id (default = camera_ns).
flip                  : int   -1/0/1 cv2.flip code, or 99 (default) = no flip.
publish_camera_info   : bool  Publish a trivial CameraInfo (default False). Fill real
                              intrinsics once the camera is calibrated.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera_node")
        self.declare_parameter("video_device", "/dev/video2")
        self.declare_parameter("camera_ns", "arm_cam")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("flip", 99)
        # perception_node._process gates on rgb AND depth AND camera_info all being
        # present before it publishes /detected_object/pixel. A plain RGB webcam has
        # no depth, so we publish a DUMMY zero depth + a rough camera_info to pass the
        # gate. The node then computes the pixel/bbox from RGB (image-space servo) and
        # cleanly skips the world unprojection (zero depth -> None). Defaults ON so the
        # webcam is a drop-in for the arm_cam image servo. Fill real fx/fy/cx/cy once
        # the camera is intrinsically calibrated (needed for top_cam world grasps).
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", -1.0)   # <0 -> use width/2
        self.declare_parameter("cy", -1.0)   # <0 -> use height/2
        # Lens distortion (plumb_bob / OpenCV order k1,k2,p1,p2,k3) from the
        # checkerboard calibration. Only USED when undistort:=true, in which case
        # each frame is rectified (cv2.remap) BEFORE publishing and the published
        # camera_info carries the RECTIFIED K with zero distortion. That way both
        # the ray-plane unprojector and the AprilTag pose solver — neither of which
        # undistorts — operate on a distortion-free image. Default off so arm_cam
        # (image-space servo, no world math) is untouched.
        self.declare_parameter("undistort", False)
        self.declare_parameter("d0", 0.0)   # k1
        self.declare_parameter("d1", 0.0)   # k2
        self.declare_parameter("d2", 0.0)   # p1
        self.declare_parameter("d3", 0.0)   # p2
        self.declare_parameter("d4", 0.0)   # k3
        # getOptimalNewCameraMatrix alpha: 0 = crop to all-valid pixels (no black
        # borders, slightly narrower FOV); 1 = keep full FOV (black wedges at the
        # rectified corners). 0 is cleanest for detection.
        self.declare_parameter("undistort_alpha", 0.0)

        self.device = self.get_parameter("video_device").value
        ns = self.get_parameter("camera_ns").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.fourcc = str(self.get_parameter("fourcc").value)
        self.flip = int(self.get_parameter("flip").value)
        self.frame_id = self.get_parameter("frame_id").value or ns
        self.want_info = bool(self.get_parameter("publish_camera_info").value)
        self.want_depth = bool(self.get_parameter("publish_depth").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        self.cx = cx if cx >= 0 else self.width / 2.0
        self.cy = cy if cy >= 0 else self.height / 2.0
        self.undistort = bool(self.get_parameter("undistort").value)
        self.dist = np.array([float(self.get_parameter(f"d{i}").value) for i in range(5)])
        self.alpha = float(self.get_parameter("undistort_alpha").value)
        self.map1 = None
        self.map2 = None
        # Intrinsics actually PUBLISHED in camera_info. Equal to the raw fx/fy/cx/cy
        # unless undistort is on, in which case _build_undistort overwrites them
        # with the rectified newK on the first frame.
        self.pub_fx, self.pub_fy = self.fx, self.fy
        self.pub_cx, self.pub_cy = self.cx, self.cy

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, f"/{ns}/rgb", 10)
        self.info_pub = (
            self.create_publisher(CameraInfo, f"/{ns}/camera_info", 10)
            if self.want_info else None
        )
        self.depth_pub = (
            self.create_publisher(Image, f"/{ns}/depth", 10)
            if self.want_depth else None
        )

        self.cap = None
        self._open()
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self._tick)
        self.get_logger().info(
            f"USB camera '{self.device}' -> /{ns}/rgb "
            f"({self.width}x{self.height}@{self.fps:.0f}, {self.fourcc})"
        )

    def _open(self):
        if self.cap is not None:
            self.cap.release()
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not cap.isOpened():
            self.get_logger().error(f"Could not open {self.device}; will retry.")
        self.cap = cap

    def _build_undistort(self, w, h):
        """Build the rectification map once, from the calibrated K + distortion,
        and set the PUBLISHED intrinsics to the rectified newK."""
        K = np.array([[self.fx, 0.0, self.cx],
                      [0.0, self.fy, self.cy],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        newK, _ = cv2.getOptimalNewCameraMatrix(K, self.dist, (w, h), self.alpha)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, self.dist, None, newK, (w, h), cv2.CV_16SC2)
        self.pub_fx = float(newK[0, 0]); self.pub_fy = float(newK[1, 1])
        self.pub_cx = float(newK[0, 2]); self.pub_cy = float(newK[1, 2])
        self.get_logger().info(
            f"undistort ON (alpha={self.alpha}): raw K "
            f"(fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}) "
            f"-> rectified K "
            f"(fx={self.pub_fx:.1f} fy={self.pub_fy:.1f} cx={self.pub_cx:.1f} cy={self.pub_cy:.1f})")

    def _tick(self):
        if self.cap is None or not self.cap.isOpened():
            self._open()
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Frame grab failed; reopening device.")
            self._open()
            return
        if self.flip in (-1, 0, 1):
            frame = cv2.flip(frame, self.flip)
        if self.undistort:
            if self.map1 is None:
                self._build_undistort(frame.shape[1], frame.shape[0])
            frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.rgb_pub.publish(msg)

        if self.depth_pub is not None:
            # Dummy depth (all-zero) so perception_node's rgb+depth gate passes.
            # Zero -> no valid depth -> world unprojection cleanly returns None AFTER
            # /detected_object/pixel is published (image-space servo unaffected).
            depth = np.zeros((h, w), dtype=np.float32)
            dmsg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            dmsg.header = msg.header
            self.depth_pub.publish(dmsg)

        if self.info_pub is not None:
            info = CameraInfo()
            info.header = msg.header
            info.width = w
            info.height = h
            info.distortion_model = "plumb_bob"
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.k = [self.pub_fx, 0.0, self.pub_cx,
                      0.0, self.pub_fy, self.pub_cy,
                      0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [self.pub_fx, 0.0, self.pub_cx, 0.0,
                      0.0, self.pub_fy, self.pub_cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)


def main():
    rclpy.init()
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()