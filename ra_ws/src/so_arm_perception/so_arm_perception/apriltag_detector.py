# apriltag_detector.py
#
# AprilTag detection + 6-DOF pose recovery using the OFFICIAL AprilRobotics
# apriltag3 detector via its Python binding (pupil_apriltags). This replaces the
# previous cv2.aruco path: aruco's contour-based quad finder was failing to DECODE
# the small / oblique / aliased synthetically-rendered dispenser tag most frames
# (the arm cam looks up at it at an angle), so /apriltag/pixel went quiet and the
# centering loop reported "tag lost" while the tag was plainly in frame. apriltag3
# uses graph-segmentation quad detection + payload sharpening and decodes that view
# far more reliably. The node around this class is UNCHANGED: position still comes
# from the depth buffer (not PnP), and the same /apriltag/pixel + /apriltag/pose
# topics are published. Pose ORIENTATION still uses cv2.solvePnP on the corners.
import numpy as np
import cv2
from pupil_apriltags import Detector

# Friendly family name -> pupil_apriltags / apriltag3 family string.
_FAMILY = {
    "36h11": "tag36h11",
    "25h9":  "tag25h9",
    "16h5":  "tag16h5",
}


class AprilTagDetector:
    """Detect AprilTags and recover each tag's pose in the camera optical frame.

    tag_size is the black-square edge length in metres. The recovered tvec is in
    the OpenCV/REP-103 optical convention (X right, Y down, Z forward) — i.e. the
    same convention the `*_camera` TF frame uses, so no Isaac axis flip is needed
    (unlike the depth path in unprojector.py, which flips Y/Z)."""

    def __init__(self, tag_size_m=0.01, family="36h11"):
        if family not in _FAMILY:
            raise ValueError(f"unknown tag family {family!r}; pick from {list(_FAMILY)}")
        self.tag_size = float(tag_size_m)
        # apriltag3 detector, tuned for the SLANTED / small / synthetically-rendered
        # dispenser tag seen by the eye-in-hand arm cam:
        #   quad_decimate=1.0   full resolution (no downscale) — best for small/oblique tags
        #   quad_sigma=0.8      slight Gaussian blur smooths the aliased synthetic edges
        #   refine_edges=1      subpixel edge refinement → cleaner corners + pose
        #   decode_sharpening   sharpen the payload before bit sampling → marginal decodes land
        self.detector = Detector(
            families=_FAMILY[family],
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.5,
            debug=0,
        )
        # Tag corner model, ordered TL, TR, BR, BL to match the corners we hand to
        # solvePnP below. Tag frame: origin at centre, X right, Y up, Z out of face.
        s = self.tag_size / 2.0
        self._objp = np.array(
            [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32
        )

    def detect(self, rgb_image: np.ndarray) -> list[dict]:
        """Return a list of detections, one per tag:
        {id, u, v, corners(4x2), bbox(x1,y1,x2,y2)}. Pose is added later by
        estimate_pose() which needs the camera intrinsics."""
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        results = self.detector.detect(gray)
        out = []
        for r in results:
            # apriltag3 corner order (tag frame) is [TR, TL, BL, BR]; reindex to
            # [TL, TR, BR, BL] to match _objp + solvePnP's SOLVEPNP_IPPE_SQUARE
            # requirement. This index map is fixed by the decoded tag orientation,
            # so it holds however the tag is rotated/skewed in the image.
            c = np.asarray(r.corners, dtype=np.float32)
            pts = c[[1, 0, 3, 2]].copy()
            x1, y1 = pts[:, 0].min(), pts[:, 1].min()
            x2, y2 = pts[:, 0].max(), pts[:, 1].max()
            out.append({
                "id": int(r.tag_id),
                "u": float(r.center[0]),
                "v": float(r.center[1]),
                "corners": pts,
                "bbox": (float(x1), float(y1), float(x2), float(y2)),
            })
        return out

    def estimate_pose(self, corners_2d, K, dist=None):
        """Recover (rvec, tvec) of a single tag in the camera optical frame.
        corners_2d: 4x2 pixel corners (cv2.aruco order). K: 3x3 intrinsics.
        Returns (rvec(3,1), tvec(3,1)) or None on failure."""
        if dist is None:
            dist = np.zeros(5, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self._objp, corners_2d.astype(np.float32), K, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        return rvec, tvec

    def draw(self, rgb_image, detections, K=None):
        """Annotate a copy of the image with marker outlines + axes (if K given)."""
        vis = rgb_image.copy()
        if not detections:
            return vis
        corners = [d["corners"].reshape(1, 4, 2).astype(np.float32) for d in detections]
        ids = np.array([[d["id"]] for d in detections])
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        if K is not None:
            for d in detections:
                pose = self.estimate_pose(d["corners"], K)
                if pose is not None:
                    rvec, tvec = pose
                    cv2.drawFrameAxes(vis, K, np.zeros(5), rvec, tvec, self.tag_size)
        return vis