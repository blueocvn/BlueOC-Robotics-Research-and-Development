# handle_detector.py
#
# Cup-handle orientation estimator (overhead / eye-to-hand top_cam).
#
# WHY: to reorient a mug into a favourable grasp we grip it from the TOP, roll
# the wrist to spin the cup about its vertical axis, then release. To know HOW
# MUCH to roll we must first know where the handle currently points.
#
# THE MUG: green INTERIOR, BLACK OUTER body AND black handle. The silhouette is
# GREEN interior UNION BLACK body — the green fills the opening so the blob is
# solid and the axis centre sits inside it (on the 37deg-tilted view the black body
# alone is a crescent below the opening that does NOT enclose the centre). The
# handle is the part of that silhouette sticking OUT past the body: we fit an
# ellipse to the BODY (fit, drop the highest-residual points = the handle, refit —
# a plain fit stretches to swallow the handle) and take the contour points beyond
# it as the handle. Ellipse-residual (not a fixed radius) because the oblique cam
# renders the round body as an ellipse. The handle centroid is unprojected to world
# and compared against the base->cup approach azimuth to give the required cup turn.
#
# Two cups are present, so we LOCK onto the pipeline's target cup via
# /detected_object/pixel (the largest-blob pick would grab the wrong one).
#
# Standalone (own subs + unprojector) so it never touches the working pipeline.
# Publishes:
#   /cup_handle/bearing        std_msgs/Float32          handle yaw in world (rad)
#   /cup_handle/required_turn  std_msgs/Float32          cup rotation to apply (rad)
#   /cup_handle/state          std_msgs/Float32MultiArray [cx, cy, handle_yaw,
#                                approach_yaw, desired_yaw, required_turn, conf]
#   /cup_handle/debug_image    sensor_msgs/Image         annotated overhead view
#
# Run (inherits top_sim_camera->world static TF + sim time):
#   ros2 launch so_arm_perception perception.launch.py handle_detector:=true
# Standalone works ONLY if perception.launch.py (eth_static_tf) is already up:
#   ros2 run so_arm_perception handle_detector --ros-args -p use_sim_time:=true
# Tune black_v_hi + roi_scale against the live render (watch /cup_handle/debug_image).

import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import tf2_ros

from so_arm_perception.unprojector import DepthUnprojector


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HandleDetector(Node):
    def __init__(self):
        super().__init__("handle_detector")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.unproj = DepthUnprojector()

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("camera_ns", "top_cam")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("plane_z", 0.05986)      # mug grasp mid-height
        self.declare_parameter("base_x", 0.0)           # arm Rotation axis (world)
        self.declare_parameter("base_y", 0.0)

        # Green-interior HSV — used to find the axis when the pipeline lock is
        # unavailable (fallback). Same band as detector.py.
        self.declare_parameter("green_h_lo", 35)
        self.declare_parameter("green_s_lo", 80)
        self.declare_parameter("green_v_lo", 80)
        self.declare_parameter("green_h_hi", 85)
        self.declare_parameter("green_s_hi", 255)
        self.declare_parameter("green_v_hi", 255)
        self.declare_parameter("green_min_area", 120)

        # BLACK outer-mug segmentation: a pixel is dark when its Value < black_v_hi.
        # 90 (not 60) so the lit top rim of the black body is captured too.
        self.declare_parameter("black_v_hi", 90)
        self.declare_parameter("black_s_hi", 255)
        # ROI radius = roi_scale * green-interior radius. Isolates the target mug
        # from dark background and the OTHER cup; must be big enough to contain the
        # black body + handle (body/handle are larger than the green interior).
        self.declare_parameter("roi_scale", 4.0)
        # Body ellipse is fit ROBUSTLY: fit, keep the lowest body_fit_keep_pct% of
        # residuals (drops the protruding handle), refit — otherwise the ellipse
        # stretches to swallow the handle and nothing protrudes.
        self.declare_parameter("body_fit_keep_pct", 72.0)
        # A contour point is "handle" when its normalised ellipse radius exceeds this.
        self.declare_parameter("handle_ratio", 1.15)
        self.declare_parameter("min_handle_pts", 8)
        self.declare_parameter("fallback_body_radius_px", 30.0)

        # "along_approach" (default): handle along the radial line (fingers on bare
        # sides, handle fore/aft). "along_jaw": handle along the finger line.
        self.declare_parameter("handle_target_mode", "along_approach")
        self.declare_parameter("wrist_roll_limit", 2.793)      # 160 deg
        self.declare_parameter("target_pixel_timeout", 1.5)    # s

        self.base_frame = self.get_parameter("base_frame").value
        cam_ns = self.get_parameter("camera_ns").value

        # ── Subscriptions ───────────────────────────────────────────
        self.latest_rgb = None
        self.target_px = None      # [u, v, img_w, img_h, bbox_w, bbox_h]
        self.target_stamp = None
        self.create_subscription(Image, f"/{cam_ns}/rgb", self._rgb_cb, 10)
        self.create_subscription(
            CameraInfo, f"/{cam_ns}/camera_info",
            self.unproj.update_from_camera_info, 10)
        self.create_subscription(
            Float32MultiArray, "/detected_object/pixel", self._target_cb, 10)

        # ── Publishers ──────────────────────────────────────────────
        self.bearing_pub = self.create_publisher(Float32, "/cup_handle/bearing", 10)
        self.turn_pub = self.create_publisher(Float32, "/cup_handle/required_turn", 10)
        self.state_pub = self.create_publisher(
            Float32MultiArray, "/cup_handle/state", 10)
        self.debug_pub = self.create_publisher(Image, "/cup_handle/debug_image", 10)

        self.create_timer(0.1, self._loop)
        self.get_logger().info("Handle detector ready")

    # ── Callbacks ───────────────────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _target_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.target_px = list(msg.data)
            self.target_stamp = self.get_clock().now()

    def _hsv_band(self, prefix):
        lo = np.array([int(self.get_parameter(f"{prefix}_h_lo").value),
                       int(self.get_parameter(f"{prefix}_s_lo").value),
                       int(self.get_parameter(f"{prefix}_v_lo").value)])
        hi = np.array([int(self.get_parameter(f"{prefix}_h_hi").value),
                       int(self.get_parameter(f"{prefix}_s_hi").value),
                       int(self.get_parameter(f"{prefix}_v_hi").value)])
        return lo, hi

    # ── Locate the TARGET cup axis centre + green-interior radius (px) ──
    def _target_center(self, hsv):
        """Return (center_xy, green_radius_px, source_str) or (None, None, reason)."""
        if self.target_px is not None and self.target_stamp is not None:
            age = (self.get_clock().now() - self.target_stamp).nanoseconds * 1e-9
            if age <= float(self.get_parameter("target_pixel_timeout").value):
                u, v, _iw, _ih, bw, bh = self.target_px[:6]
                R = max(float(bw), float(bh)) * 0.5
                if R < 5.0:
                    R = float(self.get_parameter("fallback_body_radius_px").value)
                return np.array([float(u), float(v)]), R, "pipeline-lock"

        # Fallback: largest green blob (unreliable with two cups — warn).
        g_lo, g_hi = self._hsv_band("green")
        gm = cv2.inRange(hsv, g_lo, g_hi)
        gm = cv2.dilate(cv2.erode(gm, None, 2), None, 2)
        gc, _ = cv2.findContours(gm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        gc = [c for c in gc
              if cv2.contourArea(c) > float(self.get_parameter("green_min_area").value)]
        if not gc:
            return None, None, "no target lock and no green blob"
        g_big = max(gc, key=cv2.contourArea)
        M = cv2.moments(g_big)
        center = np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])
        _x, _y, w, h = cv2.boundingRect(g_big)
        self.get_logger().warn(
            "no fresh /detected_object/pixel lock — using largest green blob "
            "(may be the WRONG cup of two)", throttle_duration_sec=5.0)
        return center, max(w, h) * 0.5, "green-fallback"

    @staticmethod
    def _ellipse_radius(pts, ellipse):
        """Normalised ellipse radius of each point (=1 on the ellipse boundary)."""
        (ex, ey), (MA, ma), ang = ellipse
        a = max(MA * 0.5, 1e-3)
        b = max(ma * 0.5, 1e-3)
        th = math.radians(ang)
        ct, st = math.cos(th), math.sin(th)
        dx = pts[:, 0] - ex
        dy = pts[:, 1] - ey
        xr = dx * ct + dy * st
        yr = -dx * st + dy * ct
        return np.sqrt((xr / a) ** 2 + (yr / b) ** 2)

    def _robust_body_ellipse(self, pts):
        """Fit an ellipse to the BODY only. A plain fitEllipse over body+handle
        stretches to swallow the handle; instead we fit, drop the highest-residual
        points (the protruding handle), and refit — twice."""
        keep_pct = float(self.get_parameter("body_fit_keep_pct").value)
        ell = cv2.fitEllipse(pts.astype(np.float32))
        for _ in range(2):
            r = self._ellipse_radius(pts, ell)
            inliers = pts[r <= np.percentile(r, keep_pct)]
            if len(inliers) >= 5:
                ell = cv2.fitEllipse(inliers.astype(np.float32))
        return ell

    # ── Main loop ───────────────────────────────────────────────────
    def _loop(self):
        rgb = self.latest_rgb
        if rgb is None or not self.unproj.ready:
            return
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        center, R, src = self._target_center(hsv)
        if center is None:
            self._publish_debug(rgb, None, None, None, None, None, note=src)
            return

        # Full mug silhouette = GREEN interior UNION BLACK body. The green fills the
        # opening so the silhouette is a solid blob the axis centre sits inside — on
        # the tilted view the black body alone is a crescent BELOW the opening that
        # does NOT enclose the centre. Restricted to a ROI so dark background and the
        # OTHER cup can't join in.
        v_hi = int(self.get_parameter("black_v_hi").value)
        s_hi = int(self.get_parameter("black_s_hi").value)
        roi_r = R * float(self.get_parameter("roi_scale").value)
        g_lo, g_hi = self._hsv_band("green")
        green = cv2.dilate(cv2.erode(cv2.inRange(hsv, g_lo, g_hi), None, 2), None, 2)
        black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([179, s_hi, v_hi]))
        mug_mask = cv2.bitwise_or(green, black)
        roi = np.zeros(mug_mask.shape, np.uint8)
        cv2.circle(roi, (int(center[0]), int(center[1])), int(roi_r), 255, -1)
        mug_mask = cv2.bitwise_and(mug_mask, roi)
        mug_mask = cv2.morphologyEx(
            mug_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)

        # The target mug = the silhouette contour enclosing the axis centre.
        cnts, _ = cv2.findContours(mug_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx, cy = float(center[0]), float(center[1])
        containing = [c for c in cnts
                      if cv2.pointPolygonTest(c, (cx, cy), False) >= 0 and len(c) >= 5]
        mug = None
        if containing:
            mug = max(containing, key=cv2.contourArea)
        elif cnts:
            big = [c for c in cnts if len(c) >= 5]
            if big:
                mug = min(big, key=lambda c: -cv2.pointPolygonTest(c, (cx, cy), True))

        handle_dir_px = None
        confidence = 0.0
        ellipse = None
        handle_pts = None
        if mug is not None:
            pts = mug.reshape(-1, 2).astype(np.float64)
            ellipse = self._robust_body_ellipse(pts)
            r = self._ellipse_radius(pts, ellipse)
            sel = r > float(self.get_parameter("handle_ratio").value)
            n = int(sel.sum())
            if n >= int(self.get_parameter("min_handle_pts").value):
                handle_pts = pts[sel]
                handle_dir_px = handle_pts.mean(axis=0) - center
                if np.linalg.norm(handle_dir_px) < 1e-3:
                    handle_dir_px = None
                else:
                    confidence = float(np.clip(n / (0.10 * len(pts) + 1e-6), 0.0, 1.0))

        # World conversion + required turn.
        handle_yaw = approach_yaw = desired_yaw = required_turn = None
        cw = self.unproj.pixel_ray_to_plane_world(
            int(round(cx)), int(round(cy)), self.tf_buffer,
            target_frame=self.base_frame,
            plane_z=float(self.get_parameter("plane_z").value), stamp=None)
        if cw is not None:
            bx = float(self.get_parameter("base_x").value)
            by = float(self.get_parameter("base_y").value)
            approach_yaw = math.atan2(cw[1] - by, cw[0] - bx)
            if handle_dir_px is not None:
                # Unproject the ACTUAL handle-silhouette centroid (not a fixed step
                # along the image direction): under the 37deg tilt the image
                # direction is not the world direction, so we let ray-plane invert
                # the projection with the real extrinsics. Both center and handle
                # hit the same mid-height plane, so the tilt cancels in the bearing.
                hp = center + handle_dir_px
                hw = self.unproj.pixel_ray_to_plane_world(
                    int(round(hp[0])), int(round(hp[1])), self.tf_buffer,
                    target_frame=self.base_frame,
                    plane_z=float(self.get_parameter("plane_z").value), stamp=None)
                if hw is not None:
                    handle_yaw = math.atan2(hw[1] - cw[1], hw[0] - cw[0])
                    mode = self.get_parameter("handle_target_mode").value
                    offset = 0.0 if mode == "along_approach" else math.pi / 2.0
                    target = approach_yaw + offset
                    cands = [_wrap(target - handle_yaw),
                             _wrap(target + math.pi - handle_yaw)]
                    required_turn = min(cands, key=abs)
                    desired_yaw = _wrap(handle_yaw + required_turn)

        # Publish.
        if handle_yaw is not None:
            self.bearing_pub.publish(Float32(data=float(handle_yaw)))
        if required_turn is not None:
            limit = float(self.get_parameter("wrist_roll_limit").value)
            beyond = abs(required_turn) > limit
            self.turn_pub.publish(Float32(data=float(required_turn)))
            self.state_pub.publish(Float32MultiArray(data=[
                float(cw[0]), float(cw[1]), float(handle_yaw),
                float(approach_yaw), float(desired_yaw),
                float(required_turn), float(confidence)]))
            self.get_logger().info(
                f"[{src}] handle={math.degrees(handle_yaw):+.0f} deg  "
                f"approach={math.degrees(approach_yaw):+.0f}  "
                f"turn={math.degrees(required_turn):+.0f} deg  conf={confidence:.2f}"
                + ("  [BEYOND wrist-roll limit]" if beyond else ""),
                throttle_duration_sec=2.0)
        elif mug is not None:
            self.get_logger().info(
                f"[{src}] mug found but no handle protrusion (raise handle_ratio "
                "sensitivity or check black_v_hi)", throttle_duration_sec=3.0)

        self._publish_debug(rgb, center, roi_r, ellipse, handle_pts, handle_dir_px,
                            handle_yaw=handle_yaw, required_turn=required_turn,
                            confidence=confidence, note=None)

    # ── Debug image ─────────────────────────────────────────────────
    def _publish_debug(self, rgb, center, roi_r, ellipse, handle_pts, handle_dir_px,
                       handle_yaw=None, required_turn=None, confidence=0.0, note=None):
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if center is not None:
            c = (int(center[0]), int(center[1]))
            if roi_r is not None:
                cv2.circle(vis, c, int(roi_r), (255, 150, 0), 1)      # ROI
            if ellipse is not None:
                cv2.ellipse(vis, ellipse, (0, 200, 200), 1)           # fitted body
            if handle_pts is not None:
                for p in handle_pts.astype(int):
                    cv2.circle(vis, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)
            cv2.circle(vis, c, 5, (0, 0, 255), -1)                     # axis
            if handle_dir_px is not None:
                hd = handle_dir_px / (np.linalg.norm(handle_dir_px) + 1e-9)
                tip = (int(center[0] + hd[0] * 90), int(center[1] + hd[1] * 90))
                cv2.arrowedLine(vis, c, tip, (0, 255, 0), 3, tipLength=0.25)
        y = 24
        for line in [
            note,
            None if handle_yaw is None else f"handle: {math.degrees(handle_yaw):+.0f} deg",
            None if required_turn is None else f"turn:   {math.degrees(required_turn):+.0f} deg",
            None if handle_yaw is None else f"conf:   {confidence:.2f}",
        ]:
            if line:
                cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)
                y += 28
        out = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out, encoding="rgb8"))


def main():
    rclpy.init()
    node = HandleDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()