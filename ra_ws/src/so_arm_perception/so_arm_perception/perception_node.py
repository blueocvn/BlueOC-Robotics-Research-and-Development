# perception_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from rclpy.duration import Duration
import tf2_ros
import numpy as np

from rcl_interfaces.msg import SetParametersResult

from so_arm_perception.detector import YOLODetector
from so_arm_perception.unprojector import DepthUnprojector


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.debug_pub = self.create_publisher(Image, "/perception/debug_image", 10)

        # One unprojector per camera
        self.unproj_eth = DepthUnprojector()  # eye-to-hand
        self.unproj_eih = DepthUnprojector()  # eye-in-hand

        self.declare_parameter("camera_eth_ns",  "top_cam")
        self.declare_parameter("camera_eih_ns",  "arm_cam")
        self.declare_parameter("target_classes", "cup,bottle")
        self.declare_parameter("yolo_model",     "yolo11n.pt")
        self.declare_parameter("base_frame",     "world")
        # Synthetic Isaac renders score lower than COCO photos; the mug detects
        # at ~0.39, so keep this below that (HSV still backstops a miss).
        self.declare_parameter("conf_threshold", 0.25)
        # Which camera drives /detected_object/position:
        #   "top_cam" (eye-to-hand) or "arm_cam" (eye-in-hand)
        self.declare_parameter("active_camera",  "top_cam")
        # When detecting on arm_cam (eye-in-hand phase-0 servo), centre on the GREEN
        # INTERIOR (HSV-first) instead of the YOLO outer-body bbox, so the gripper
        # aligns to the green inner pocket it actually grasps into. top_cam keeps
        # YOLO-first (green interior carries a y-bias from the oblique overhead view).
        self.declare_parameter("arm_cam_use_green", True)
        self.arm_cam_use_green = bool(self.get_parameter("arm_cam_use_green").value)
        # top_cam green-first too: the COCO YOLO model false-positives the yellow
        # robot ARM as a cup/bottle from the overhead view (largest "cup" box landed
        # on the arm, ~16 cm off), whereas HSV green segmentation locks onto the
        # actual green mug (validated: ray-plane within ~0.7 cm x / 1.5 cm y of the
        # known mug pose). The small green-centroid y-bias is far better than YOLO
        # grabbing the wrong object — and final grasp centring is closed-loop on
        # arm_cam anyway. Set False to restore YOLO-first top_cam.
        self.declare_parameter("top_cam_use_green", True)
        self.top_cam_use_green = bool(self.get_parameter("top_cam_use_green").value)
        # Systematic-bias correction (metres, world frame) added to the published
        # top_cam (eye-to-hand) world point. The green-INTERIOR centroid ranged
        # from the oblique top_cam lands partway down the cup interior, NOT the
        # base: measured detected z = 0.064 for a cup with base 0.01486 + height
        # 0.09, i.e. center = 0.05986 — so the raw z already lands ~4 mm above the
        # cup's vertical CENTER, which is the correct side-grasp height. Hence
        # z_correction defaults to 0 (do NOT push it down to the base). y was
        # -0.031 m off but that is only a seed — final centring is closed-loop on
        # arm_cam pixels — and the bias is view-dependent, so leave it at 0 too.
        # Only applied for active_camera == top_cam.
        self.declare_parameter("eth_x_correction", 0.0)
        self.declare_parameter("eth_y_correction", 0.0)
        self.declare_parameter("eth_z_correction", 0.0)
        self.eth_corr = (
            float(self.get_parameter("eth_x_correction").value),
            float(self.get_parameter("eth_y_correction").value),
            float(self.get_parameter("eth_z_correction").value),
        )

        # Ray-plane unprojection for top_cam (eye-to-hand). The depth buffer
        # gives the range to the mug's NEAR SURFACE, so unprojecting it from the
        # oblique top_cam biases the world point off the mug's vertical axis
        # (the ~3 cm y error). Since the mug height is known and fixed, intersect
        # the detection-pixel ray with the horizontal plane z = eth_plane_z to
        # recover the axis (x, y) exactly. Defaults to the mug grasp mid-height
        # (CUP_BASE_Z 0.01486 + MUG_HEIGHT 0.09 / 2 = 0.05986 in mtc_node). Only
        # for active_camera == top_cam; arm_cam keeps depth-buffer unprojection.
        self.declare_parameter("eth_use_ray_plane", True)
        self.declare_parameter("eth_plane_z", 0.05986)
        self.eth_use_ray_plane = bool(self.get_parameter("eth_use_ray_plane").value)
        self.eth_plane_z       = float(self.get_parameter("eth_plane_z").value)

        # ── Pink destination-tray detection (eye-to-hand / top_cam) ──
        # The tray is a fixed pink slab; we detect it by HSV colour on the
        # overhead cam and recover its (x, y) by ray-plane intersection at the
        # known tray-top height. Runs INDEPENDENTLY of active_camera so the tray
        # fix keeps updating even while arm_cam drives the cup servo. HSV bounds
        # and plane_z are live-tunable ROS params (tune against the Isaac render).
        self.declare_parameter("detect_tray", True)
        self.declare_parameter("tray_h_lo", 145)
        self.declare_parameter("tray_s_lo", 35)
        self.declare_parameter("tray_v_lo", 40)
        self.declare_parameter("tray_h_hi", 170)
        self.declare_parameter("tray_s_hi", 255)
        self.declare_parameter("tray_v_hi", 255)
        self.declare_parameter("tray_min_area", 400)
        # Tray-top height in world frame (ray-plane target) = the tray coordinate
        # the pink surface sits at. Updated for the moved tray (-0.02744).
        self.declare_parameter("tray_plane_z", -0.02744)
        # Constant world-frame correction added to the published tray point. The
        # oblique top_cam sees the tray's tall far wall, stretching the pink bbox
        # toward the back → the bbox-centre lands ~2.6 cm behind the true centre
        # (measured raw y=-0.295 vs known -0.269). Default tray_y_correction
        # cancels that so the published point equals the true tray centre. Tray
        # footprint is 0.26 m (long) x 0.19 m (wide).
        # Bias corrections added to the published tray point. RESET to 0 for the
        # newly moved tray (-0.19928,-0.25747,-0.02744): the previous calibration
        # is stale. RE-CALIBRATE on the next sim run with Isaac PLAYING (grab a
        # frame, compare raw bbox-centre world vs true centre, set = true - raw).
        self.declare_parameter("tray_x_correction", 0.0)
        self.declare_parameter("tray_y_correction", 0.0)
        self.declare_parameter("tray_z_correction", 0.0)
        self.detect_tray_enabled = bool(self.get_parameter("detect_tray").value)
        # Multi-cup refill: reject a green-cup detection whose world (x,y) falls
        # within this radius of the last detected tray centre — i.e. a cup already
        # PLACED in the tray — so top_cam locks onto the remaining platform cup.
        # Only applied to top_cam (eye-to-hand). Latest tray world (x,y) is cached
        # by _process_tray.
        self.declare_parameter("cup_tray_exclude", True)
        self.declare_parameter("cup_tray_exclude_radius", 0.15)
        self.last_tray_xy = None
        # Eye-in-hand (arm_cam) target lock: the pixel of the blob we are servoing.
        # We keep tracking the blob nearest this pixel frame-to-frame so a SECOND cup
        # that pans into view during the phase-0 sweep can't hijack the servo (the
        # raw "largest blob" pick would otherwise flip to whichever cup looks bigger).
        # Reset whenever the servo (active_camera) toggles.
        self.arm_target_px = None
        # If the nearest detection jumps farther than this (px) from the locked
        # target, treat the lock as lost and re-seed to the largest blob.
        self.declare_parameter("arm_lock_max_jump_px", 160.0)

        eth_ns      = self.get_parameter("camera_eth_ns").value
        eih_ns      = self.get_parameter("camera_eih_ns").value
        classes     = self.get_parameter("target_classes").value.split(",")
        model_path  = self.get_parameter("yolo_model").value
        self.base_frame     = self.get_parameter("base_frame").value
        conf        = self.get_parameter("conf_threshold").value
        self.active_camera  = self.get_parameter("active_camera").value

        # Allow `active_camera` to be switched at runtime (e.g. mtc_node flips to
        # "arm_cam" for eye-in-hand visual servoing, then back to "top_cam").
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.detector = YOLODetector(
            model_path=model_path,
            target_classes=classes,
            conf_threshold=conf,
        )

        # Latest frames — updated by callbacks, consumed by timer
        self.latest = {
            "top_cam_rgb":   None,
            "top_cam_depth": None,
            "arm_cam_rgb":   None,
            "arm_cam_depth": None,
        }

        # --- Subscribers ---
        # Eye-to-hand
        self.create_subscription(
            Image, f"/{eth_ns}/rgb",
            lambda msg: self._rgb_cb(msg, "top_cam_rgb"), 10
        )
        self.create_subscription(
            Image, f"/{eth_ns}/depth",
            lambda msg: self._depth_cb(msg, "top_cam_depth"), 10
        )
        self.create_subscription(
            CameraInfo, f"/{eth_ns}/camera_info",
            lambda msg: self.unproj_eth.update_from_camera_info(msg), 10
        )

        # Eye-in-hand
        self.create_subscription(
            Image, f"/{eih_ns}/rgb",
            lambda msg: self._rgb_cb(msg, "arm_cam_rgb"), 10
        )
        self.create_subscription(
            Image, f"/{eih_ns}/depth",
            lambda msg: self._depth_cb(msg, "arm_cam_depth"), 10
        )
        self.create_subscription(
            CameraInfo, f"/{eih_ns}/camera_info",
            lambda msg: self.unproj_eih.update_from_camera_info(msg), 10
        )

        # --- Publisher: detected object position in base frame ---
        self.obj_pub = self.create_publisher(
            PointStamped, "/detected_object/position", 10
        )

        # --- Publisher: raw detection in IMAGE space (for image-based servoing) ---
        # Layout: [u, v, img_w, img_h, bbox_w, bbox_h]. Published the moment a
        # detection exists, BEFORE the depth/TF unprojection below — so an
        # image-space servo loop keeps getting fixes even when depth/TF (which
        # cause the world-position dropouts) fail. No extrinsic/depth needed.
        self.pixel_pub = self.create_publisher(
            Float32MultiArray, "/detected_object/pixel", 10
        )

        # Perceived metric range (m) from the ACTIVE camera to the detection centre
        # (camera-frame depth). The eye-in-hand servo (active_camera = arm_cam) uses
        # this to close the loop on phase-1 approach distance.
        self.depth_pub = self.create_publisher(
            Float32, "/detected_object/depth", 10
        )

        # --- Publisher: 3D detection bounding box for RViz ---
        # A wireframe rectangle drawn at the object's depth, in the ACTIVE camera's
        # optical frame, so it shows what the arm cam sees during servoing. RViz
        # transforms it via TF — add a "Marker" display on this topic.
        self.bbox_pub = self.create_publisher(
            Marker, "/detected_object/bbox_marker", 10
        )

        # --- Publishers: pink destination tray (world + image space + debug) ---
        self.tray_pub = self.create_publisher(
            PointStamped, "/detected_tray/position", 10
        )
        self.tray_pixel_pub = self.create_publisher(
            Float32MultiArray, "/detected_tray/pixel", 10
        )
        self.tray_debug_pub = self.create_publisher(
            Image, "/perception/tray_debug_image", 10
        )

        # --- Publisher: ALL platform cups (world frame) for obstacle avoidance ---
        # mtc_node adds the non-target cups as collision cylinders so the pre-grasp
        # planner routes around the cup(s) it isn't currently picking.
        self.cups_pub = self.create_publisher(PoseArray, "/detected_cups", 10)

        # --- Main loop at 10Hz ---
        self.create_timer(0.1, self.perception_loop)
        self.get_logger().info("Perception node ready")

    # ── Dynamic parameters ─────────────────────────────────────
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "active_camera":
                if p.value not in ("top_cam", "arm_cam"):
                    return SetParametersResult(
                        successful=False,
                        reason="active_camera must be 'top_cam' or 'arm_cam'",
                    )
                self.active_camera = p.value
                self.arm_target_px = None  # drop stale eye-in-hand lock on toggle
                self.get_logger().info(f"active_camera -> {p.value}")
        return SetParametersResult(successful=True)

    # ── Callbacks ──────────────────────────────────────────────

    def _rgb_cb(self, msg: Image, key: str):
        self.latest[key] = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

    def _depth_cb(self, msg: Image, key: str):
        depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        # Normalize to float32 meters
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        self.latest[key] = depth

    # ── Main perception loop ────────────────────────────────────

    def perception_loop(self):
        # Pick the detection source. arm_cam = eye-in-hand, top_cam = eye-to-hand.
        if self.active_camera == "arm_cam":
            self._process("arm_cam", "arm_cam_rgb", "arm_cam_depth", self.unproj_eih)
        else:
            self._process("top_cam", "top_cam_rgb", "top_cam_depth", self.unproj_eth)

        # Pink destination-tray detection always runs on the overhead top_cam,
        # regardless of the active cup camera.
        if self.detect_tray_enabled:
            self._process_tray()

    # ── Pink destination-tray processing (top_cam / eye-to-hand) ──
    def _process_tray(self):
        rgb = self.latest["top_cam_rgb"]
        if rgb is None or not self.unproj_eth.ready:
            return

        lo = [int(self.get_parameter("tray_h_lo").value),
              int(self.get_parameter("tray_s_lo").value),
              int(self.get_parameter("tray_v_lo").value)]
        hi = [int(self.get_parameter("tray_h_hi").value),
              int(self.get_parameter("tray_s_hi").value),
              int(self.get_parameter("tray_v_hi").value)]
        min_area = float(self.get_parameter("tray_min_area").value)

        dets = self.detector.detect_tray(rgb, lower_hsv=lo, upper_hsv=hi,
                                          min_area=min_area)

        # Always publish a tray debug image (magenta box) so the HSV can be tuned.
        dbg = self.detector.draw_detections(rgb, dets, color=(255, 0, 255))
        dbg = np.asarray(dbg, dtype=np.uint8)
        self.tray_debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="rgb8"))

        if not dets:
            return
        best = dets[0]

        img_h, img_w = rgb.shape[:2]
        x1, y1, x2, y2 = best["bbox"]
        px = Float32MultiArray()
        px.data = [
            float(best["u"]), float(best["v"]),
            float(img_w), float(img_h),
            float(x2 - x1), float(y2 - y1),
        ]
        self.tray_pixel_pub.publish(px)

        # Ray-plane at the known tray-top height → axis-accurate (x, y), z pinned.
        plane_z = float(self.get_parameter("tray_plane_z").value)
        point_base = self.unproj_eth.pixel_ray_to_plane_world(
            best["u"], best["v"],
            self.tf_buffer,
            target_frame=self.base_frame,
            plane_z=plane_z,
            stamp=None,
        )
        if point_base is None:
            self.get_logger().warn(
                "[tray] ray-plane/TF failed — is "
                f"{self.unproj_eth.frame_id}->{self.base_frame} published?",
                throttle_duration_sec=5.0,
            )
            return

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.point.x = float(point_base[0]) + float(self.get_parameter("tray_x_correction").value)
        out.point.y = float(point_base[1]) + float(self.get_parameter("tray_y_correction").value)
        out.point.z = float(point_base[2]) + float(self.get_parameter("tray_z_correction").value)
        self.tray_pub.publish(out)
        # Cache for the cup tray-exclusion (multi-cup refill).
        self.last_tray_xy = (out.point.x, out.point.y)
        self.get_logger().info(
            f"[tray] pink tray at x={point_base[0]:.3f} "
            f"y={point_base[1]:.3f} z={point_base[2]:.3f} "
            f"pixel=({best['u']},{best['v']})",
            throttle_duration_sec=5.0,
        )

    def _process(self, label, rgb_key, depth_key, unproj):
        rgb   = self.latest[rgb_key]
        depth = self.latest[depth_key]

        if rgb is None or depth is None:
            self.get_logger().warn(f"[{label}] Waiting for image/depth...", once=True)
            return
        if not unproj.ready:
            self.get_logger().warn(f"[{label}] Waiting for camera_info...", once=True)
            return

        # 1. Detect. Prefer the green mug (HSV) on both cams: arm_cam centres on the
        # green interior (jaw target); top_cam uses green because COCO YOLO mistakes
        # the yellow arm for a cup from overhead. YOLO stays as the fallback.
        prefer_green = ((label == "arm_cam") and self.arm_cam_use_green) or \
                       ((label == "top_cam") and self.top_cam_use_green)
        detections = self.detector.detect(rgb, prefer_green=prefer_green)

        # 2. ALWAYS publish debug image — even if no detections
        debug_img = self.detector.draw_detections(rgb, detections)
        debug_img = np.asarray(debug_img, dtype=np.uint8)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="rgb8")
        self.debug_pub.publish(debug_msg)

        # NOW check detections
        if not detections:
            return

        # Multi-cup handling on top_cam (eye-to-hand overhead):
        #   1. compute each candidate's world (x,y,z) via ray-plane at the cup plane
        #   2. drop any inside the tray zone (a cup already placed)
        #   3. TARGET = NEAREST platform cup to the base (world origin) — so the arm
        #      never has to reach PAST one cup to grab another (collision avoidance)
        #   4. publish ALL platform cups on /detected_cups so mtc_node can add the
        #      non-target ones as collision obstacles.
        if label == "top_cam":
            exclude = (bool(self.get_parameter("cup_tray_exclude").value)
                       and self.last_tray_xy is not None)
            excl_r = float(self.get_parameter("cup_tray_exclude_radius").value)
            cand = []  # (detection, world_xyz)
            for d in detections:
                wp = self.unproj_eth.pixel_ray_to_plane_world(
                    d["u"], d["v"], self.tf_buffer,
                    target_frame=self.base_frame,
                    plane_z=self.eth_plane_z, stamp=None,
                )
                if wp is None:
                    continue
                if exclude and float(np.hypot(wp[0] - self.last_tray_xy[0],
                                              wp[1] - self.last_tray_xy[1])) <= excl_r:
                    continue  # inside tray → placed cup, skip
                cand.append((d, wp))
            if not cand:
                self.get_logger().warn(
                    "[top_cam] no platform cup to grasp (none detected outside the "
                    "tray, or TF not ready)", throttle_duration_sec=5.0)
                return
            # Nearest to base first.
            cand.sort(key=lambda dw: float(np.hypot(dw[1][0], dw[1][1])))
            detections = [d for (d, _wp) in cand]
            # Publish every platform cup (world frame) for obstacle avoidance.
            # Apply the SAME eth_*_correction the single target position gets below:
            # mtc_node matches the target against this list (cup_obstacle_match_radius,
            # 5 cm) to skip it. If only the target were corrected, a correction larger
            # than that radius would stop the target matching its own entry here and it
            # would be added as an obstacle to itself (two cups in RViz, blocked plan).
            self._publish_cups([wp + np.array(self.eth_corr) for (_d, wp) in cand])

        # Eye-in-hand target LOCK: during the phase-0 sweep a second cup can pan into
        # view; the raw "largest blob" pick (detections[0]) would then jump to it. Once
        # we have a lock, keep the detection nearest the locked pixel instead, so the
        # servo stays on the cup it started on. Re-seed to the largest blob only if the
        # nearest is implausibly far (lock lost) or there is no lock yet.
        if label == "arm_cam" and len(detections) > 1:
            if self.arm_target_px is not None:
                lu, lv = self.arm_target_px
                near = min(detections,
                           key=lambda d: (d["u"] - lu) ** 2 + (d["v"] - lv) ** 2)
                jump = float(np.hypot(near["u"] - lu, near["v"] - lv))
                max_jump = float(self.get_parameter("arm_lock_max_jump_px").value)
                if jump <= max_jump:
                    detections = [near] + [d for d in detections if d is not near]
                else:
                    self.get_logger().warn(
                        f"[arm_cam] lock lost (nearest blob {jump:.0f}px away > "
                        f"{max_jump:.0f}) — re-seeding to largest",
                        throttle_duration_sec=2.0)
            # else: no lock yet -> keep largest-first ordering to seed below.

        best = detections[0]
        if label == "arm_cam":
            self.arm_target_px = (best["u"], best["v"])  # update the lock
        self.get_logger().info(
            f"[{label}] Detected: {best['class_name']} "
            f"conf={best['confidence']:.2f} "
            f"pixel=({best['u']}, {best['v']})",
            throttle_duration_sec=5.0
        )

        # Publish the image-space detection FIRST (before depth/TF, which can fail
        # and drop the world fix). bbox = (x1, y1, x2, y2).
        img_h, img_w = rgb.shape[:2]
        x1, y1, x2, y2 = best["bbox"]
        px = Float32MultiArray()
        px.data = [
            float(best["u"]), float(best["v"]),
            float(img_w), float(img_h),
            float(x2 - x1), float(y2 - y1),
        ]
        self.pixel_pub.publish(px)

        # top_cam recovers (x, y) by ray-plane intersection at the known object
        # height — it does NOT need the depth buffer. A real overhead webcam has no
        # depth (usb_camera_node publishes a dummy zero frame just to pass the gate
        # above), so the depth unprojection below WILL fail for it; that must NOT
        # block the ray-plane world position. Only require depth on the non-ray-plane
        # path (arm_cam eye-in-hand, whose height isn't fixed).
        use_ray_plane = (label == "top_cam" and self.eth_use_ray_plane)

        # 3. Unproject to camera frame (for the bbox marker, the servo range, and the
        #    non-ray-plane world transform). Optional when ray-plane is in use.
        point_cam = unproj.pixel_to_camera_frame(best["u"], best["v"], depth)
        if point_cam is None:
            if not use_ray_plane:
                # Expected on a depthless webcam (arm_cam eye-in-hand): the image
                # servo uses /detected_object/pixel + bbox size, never depth, so the
                # pixel is already published above. Debug-level to avoid spam.
                self.get_logger().debug(f"[{label}] Depth lookup failed at detection center")
                return
        else:
            # 3b. 3D bounding-box marker at the object's depth (RViz viz).
            self._publish_bbox_marker(best["bbox"], float(-point_cam[2]), unproj)
            # 3c. Perceived metric range (camera-frame depth, +ve). Phase-1 of the
            # servo halts the approach once this drops below its threshold.
            self.depth_pub.publish(Float32(data=float(-point_cam[2])))

        # 4. Transform to base frame (needs TF: <camera frame> -> base_frame).
        #    For arm_cam this requires a gripper->arm_sim_camera static TF.
        # stamp=None → look up the LATEST available transform (Time(0)). This
        # avoids "extrapolation into the future" when clocks skew, and is fine
        # for a slow tracking loop where the gripper pose barely moves per cycle.
        #
        # top_cam: prefer the ray-plane intersection (axis-accurate, no
        # surface-depth bias) at the known mug height. arm_cam stays on the
        # depth-buffer unprojection (eye-in-hand height is not fixed).
        if use_ray_plane:
            point_base = unproj.pixel_ray_to_plane_world(
                best["u"], best["v"],
                self.tf_buffer,
                target_frame=self.base_frame,
                plane_z=self.eth_plane_z,
                stamp=None,
            )
        else:
            point_base = unproj.camera_to_world(
                point_cam,
                self.tf_buffer,
                target_frame=self.base_frame,
                stamp=None,
            )
        if point_base is None:
            self.get_logger().warn(
                f"[{label}] TF transform failed — is {unproj.frame_id}->{self.base_frame} published?"
            )
            return

        self.get_logger().info(
            f"[{label}] Base frame: x={point_base[0]:.3f} "
            f"y={point_base[1]:.3f} z={point_base[2]:.3f}",
            throttle_duration_sec=5.0
        )

        # 4b. Correct the systematic top_cam green-interior ranging bias (see
        # eth_*_correction params). arm_cam (eye-in-hand) is left uncorrected.
        cx, cy, cz = (self.eth_corr if label == "top_cam" else (0.0, 0.0, 0.0))

        # 5. Publish position
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.point.x = float(point_base[0]) + cx
        out.point.y = float(point_base[1]) + cy
        out.point.z = float(point_base[2]) + cz
        self.obj_pub.publish(out)


    # ── Platform cups (world) for obstacle avoidance ───────────
    def _publish_cups(self, world_pts):
        """Publish every detected platform cup as a PoseArray in the base frame.
        mtc_node treats the non-target ones as collision obstacles."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        for wp in world_pts:
            p = Pose()
            p.position.x = float(wp[0])
            p.position.y = float(wp[1])
            p.position.z = float(wp[2])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.cups_pub.publish(msg)

    # ── Bounding-box marker ────────────────────────────────────
    def _publish_bbox_marker(self, bbox, z, unproj):
        """Draw the detection bbox as a wireframe rectangle at depth z, in the
        active camera's optical frame. RViz transforms it via TF; if the camera
        TF is missing the box simply won't render (no node-side failure)."""
        if not unproj.ready or unproj.frame_id is None:
            return
        x1, y1, x2, y2 = bbox
        # Closed loop of corners: TL → TR → BR → BL → TL.
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)]

        m = Marker()
        m.header.frame_id = unproj.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "detection_bbox"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.004                       # line width (m)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        m.lifetime = Duration(seconds=0.5).to_msg()  # fades if detection stops
        for (u, v) in corners:
            pc = unproj.pixel_at_depth_to_camera_frame(u, v, z)
            if pc is None:
                return
            m.points.append(Point(x=float(pc[0]), y=float(pc[1]), z=float(pc[2])))
        self.bbox_pub.publish(m)


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()