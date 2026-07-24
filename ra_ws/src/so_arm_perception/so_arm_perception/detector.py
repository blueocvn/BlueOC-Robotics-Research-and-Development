# detector.py
import numpy as np
from ultralytics import YOLO
import cv2


class YOLODetector:
    """Outer-object detector.

    PRIMARY path: a real YOLO model (yolo11n COCO) detecting the configured
    target_classes (e.g. cup/bottle). YOLO brackets the OUTER mug body, so the
    bbox centre sits over the mug's vertical axis — unlike the HSV green-interior
    centroid, which (from the oblique top_cam) lands partway down the inside and
    biased the published world point by ~3 cm in y. See PerceptionNode.

    FALLBACK path: the original HSV green-interior blob. Used whenever YOLO
    returns no qualifying detection (e.g. a synthetic render it doesn't
    recognise) so the pipeline never goes blind — the eye-in-hand green servo
    and the no-YOLO case keep working exactly as before.
    """

    def __init__(self, model_path="yolo11n.pt", target_classes=("cup", "bottle"),
                 conf_threshold=0.25, **kwargs):
        self.conf = float(conf_threshold)
        self._wanted_names = {c.strip().lower() for c in target_classes if c.strip()}

        # Load YOLO. If the model is unavailable we degrade gracefully to HSV.
        try:
            self.model = YOLO(model_path)
            # Map wanted class NAMES -> model class IDs.
            self.wanted_ids = {
                i for i, n in self.model.names.items()
                if n.lower() in self._wanted_names
            }
            print(f"[detector] YOLO loaded ({model_path}); "
                  f"target ids={sorted(self.wanted_ids)} "
                  f"names={sorted(self._wanted_names)}")
        except Exception as e:  # pragma: no cover - load-time only
            self.model = None
            self.wanted_ids = set()
            print(f"[detector] YOLO load FAILED ({e}); HSV-only fallback")

        # HSV green-interior fallback params (unchanged from the original).
        self.lower_hsv = np.array([35, 80, 80])
        self.upper_hsv = np.array([85, 255, 255])
        self.min_area = 150

        # HSV pink-tray params (destination tray). Pink/magenta lives at the top
        # of the OpenCV hue wheel (H 0-179); default is a broad pink band to be
        # tuned against the actual Isaac render. See PerceptionNode.detect_tray.
        # Tuned to the Isaac render: tray hue is a stable ~156, but rims wash out
        # to S~55 / V~216 while the floor sits at S~114 / V~156 — so keep S/V
        # thresholds low and let the (pink 150-165 vs wood 10-46, green 60-85)
        # hue band do the discrimination.
        self.tray_lower_hsv = np.array([145, 35, 40])
        self.tray_upper_hsv = np.array([170, 255, 255])
        self.tray_min_area = 400

    # ── Primary: YOLO ──────────────────────────────────────────
    def _detect_yolo(self, rgb_image: np.ndarray):
        if self.model is None:
            return []
        # ultralytics expects BGR numpy arrays; our frames are RGB.
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        res = self.model.predict(bgr, conf=self.conf, verbose=False)[0]
        if res.boxes is None or len(res.boxes) == 0:
            return []

        dets = []
        for b in res.boxes:
            cls_id = int(b.cls[0])
            if self.wanted_ids and cls_id not in self.wanted_ids:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            dets.append({
                "u": int((x1 + x2) / 2),
                "v": int((y1 + y2) / 2),
                "bbox": (x1, y1, x2, y2),
                "confidence": float(b.conf[0]),
                "class_name": self.model.names[cls_id],
                "_area": (x2 - x1) * (y2 - y1),
            })
        if not dets:
            return []
        # Largest qualifying box = the mug in the foreground.
        dets.sort(key=lambda d: d["_area"], reverse=True)
        best = dets[0]
        print(f"[detector] YOLO: {best['class_name']} "
              f"conf={best['confidence']:.2f} at ({best['u']}, {best['v']})")
        return [best]

    # ── Fallback: HSV green interior ───────────────────────────
    def _detect_hsv(self, rgb_image: np.ndarray):
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=4)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            print("[detector] HSV fallback: nothing found")
            return []

        valid = [c for c in contours if cv2.contourArea(c) > self.min_area]
        if not valid:
            print("[detector] HSV fallback: contours too small")
            return []

        # Return ALL qualifying green blobs, largest first. With two identical cups
        # in view the caller can pick the largest one OUTSIDE the tray exclusion
        # zone (so a cup already placed in the tray is not re-detected). Callers
        # that want a single detection just take element [0] (the largest).
        valid.sort(key=cv2.contourArea, reverse=True)
        out = []
        for c in valid:
            x, y, w, h = cv2.boundingRect(c)
            out.append({
                "u": x + w // 2,
                "v": y + h // 2,
                "bbox": (x, y, x + w, y + h),
                "confidence": 1.0,
                "class_name": "mug",
            })
        print(f"[detector] HSV fallback: {len(out)} mug(s), largest at "
              f"({out[0]['u']}, {out[0]['v']})")
        return out

    # ── Pink destination tray (HSV) ────────────────────────────
    def detect_tray(self, rgb_image: np.ndarray,
                    lower_hsv=None, upper_hsv=None, min_area=None) -> list[dict]:
        """Detect the pink destination tray by HSV colour segmentation.

        The tray is a large, flat, uniformly-coloured pink slab — a colour blob
        is far more robust here than a COCO class (YOLO has no "tray"). Returns
        the largest qualifying pink contour as a single detection dict with the
        bbox CENTROID as (u, v), matching the cup-detection schema so the rest of
        the pipeline (unprojection, markers) is reused unchanged.

        lower/upper_hsv and min_area override the instance defaults so the caller
        (PerceptionNode) can expose them as live-tunable ROS params.
        """
        lo = self.tray_lower_hsv if lower_hsv is None else np.asarray(lower_hsv)
        hi = self.tray_upper_hsv if upper_hsv is None else np.asarray(upper_hsv)
        area_min = self.tray_min_area if min_area is None else float(min_area)

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=4)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid = [c for c in contours if cv2.contourArea(c) > area_min]
        if not valid:
            return []

        c = max(valid, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        # Use the OUTER bounding-box centre, NOT the area-weighted centroid: the
        # tray is a rigid rectangle whose rim defines the bbox, so the bbox centre
        # stays put even when cups placed INSIDE the tray occlude the pink floor
        # (a centroid would drift toward the unoccluded side).
        u, v = x + w // 2, y + h // 2
        return [{
            "u": u,
            "v": v,
            "bbox": (x, y, x + w, y + h),
            "confidence": 1.0,
            "class_name": "tray",
            "_area": cv2.contourArea(c),
        }]

    # ── Public ─────────────────────────────────────────────────
    def detect(self, rgb_image: np.ndarray, prefer_green: bool = False) -> list[dict]:
        """Detect the mug.

        prefer_green=False (default): YOLO first (outer mug body), HSV green
        interior only as a fallback — used for the top_cam (eye-to-hand) where the
        green-interior centroid carries a ~3 cm y-bias.

        prefer_green=True: HSV green INTERIOR first, YOLO only as a fallback — used
        for the arm_cam (eye-in-hand) phase-0 servo so it centres on the green
        inner pocket (the actual jaw target) instead of the black outer body.
        """
        if prefer_green:
            dets = self._detect_hsv(rgb_image)
            if dets:
                return dets
            return self._detect_yolo(rgb_image)
        # YOLO-only: green HSV backup removed (metal cup false-positives on stray
        # green; a missed YOLO frame is preferable to a bogus green detection).
        return self._detect_yolo(rgb_image)

    def draw_detections(self, image: np.ndarray, detections: list,
                        color=(0, 255, 0)) -> np.ndarray:
        vis = image.copy()
        for d in detections:
            x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
            label = f"{d.get('class_name', 'obj')} {d.get('confidence', 1.0):.2f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.circle(vis, (d["u"], d["v"]), 5, (0, 0, 255), -1)
            cv2.putText(vis, f"{label} ({d['u']},{d['v']})",
                (x1, max(y1 - 8, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return vis