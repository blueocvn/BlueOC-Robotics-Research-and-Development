# unprojector.py
import numpy as np
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration


class DepthUnprojector:
    """
    Converts (u, v, depth) → 3D point in any target frame.
    Populated from /camera_info topic.
    """

    def __init__(self):
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = None
        self.ready = False

    def update_from_camera_info(self, msg: CameraInfo):
        """Call this in your camera_info subscriber callback."""
        # ROS CameraInfo K matrix is row-major:
        # [fx,  0, cx,
        #   0, fy, cy,
        #   0,  0,  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id
        self.ready = True

    def pixel_to_camera_frame(self, u, v, depth_image, patch_size=5):
        if not self.ready:
            return None

        v0 = max(0, v - patch_size)
        v1 = min(depth_image.shape[0], v + patch_size)
        u0 = max(0, u - patch_size)
        u1 = min(depth_image.shape[1], u + patch_size)

        patch = depth_image[v0:v1, u0:u1]
        valid = patch[(patch > 0.1) & (patch < 5.0)]

        if valid.size == 0:
            return None

        z = float(np.median(valid))

        # Standard pinhole unprojection
        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy
        z_cam = z

        print(f"Camera optical frame: x={x_cam:.3f} y={y_cam:.3f} z={z_cam:.3f}")

        # Isaac Sim depth image convention:
        # depth = distance along camera -Z axis
        # Y axis is flipped vs ROS convention
        # Convert to ROS camera optical frame
        x_ros =  x_cam
        y_ros = -y_cam  # flip Y
        z_ros = -z_cam

        print(f"ROS camera frame:     x={x_ros:.3f} y={y_ros:.3f} z={z_ros:.3f}")

        return np.array([x_ros, y_ros, z_ros])

    def pixel_at_depth_to_camera_frame(self, u, v, z):
        """Unproject a pixel at an EXPLICIT metric depth z (no depth-image lookup).
        Used to draw the detection bounding box at the object's depth — all four
        corners share the center depth so the box is a clean planar rectangle
        facing the camera. Returns the point in the ROS camera optical frame
        (same axis convention as pixel_to_camera_frame)."""
        if not self.ready:
            return None
        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy
        return np.array([x_cam, -y_cam, -z])  # flip Y, negate Z (Isaac→ROS optical)

    def pixel_ray_to_plane_world(
        self,
        u, v,
        tf_buffer: tf2_ros.Buffer,
        target_frame: str = "world",
        plane_z: float = 0.0,
        stamp=None,
    ):
        """Intersect the pixel's line-of-sight ray with the horizontal plane
        z = plane_z in target_frame, and return that 3D world point.

        Why: the depth buffer gives the range to the object's NEAR SURFACE, so
        unprojecting at that depth from an oblique camera lands the point offset
        from the object's true vertical axis (the ~3 cm top_cam y-bias). When the
        object's height is KNOWN and fixed (mug on a table), the ray-plane
        intersection recovers the axis (x, y) exactly — no depth lookup, no
        surface bias. z is pinned to plane_z by construction.

        Geometry: build the ray in the camera optical frame (same x, -y, -z flip
        as pixel_to_camera_frame), transform the camera origin AND a point one
        unit down the ray into target_frame, then intersect O + t*D with the
        plane. Returns None if the ray is parallel to the plane or TF fails.
        """
        if not self.ready:
            return None
        from rclpy.time import Time

        # Ray direction in the (flipped) camera optical frame, for unit depth.
        dx = (u - self.cx) / self.fx
        dy = -(v - self.cy) / self.fy
        dz = -1.0
        hdr_stamp = stamp or Time().to_msg()

        def to_world(px, py, pz):
            ps = PointStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = hdr_stamp
            ps.point.x, ps.point.y, ps.point.z = float(px), float(py), float(pz)
            try:
                t = tf_buffer.transform(
                    ps, target_frame, timeout=Duration(seconds=0.1)
                )
                return np.array([t.point.x, t.point.y, t.point.z])
            except Exception as e:
                print(f"TF ERROR (ray-plane): {e}")
                return None

        origin = to_world(0.0, 0.0, 0.0)          # camera centre in world
        far    = to_world(dx, dy, dz)             # one unit down the ray
        if origin is None or far is None:
            return None

        ray = far - origin
        if abs(ray[2]) < 1e-9:                    # parallel to the plane
            return None
        t = (plane_z - origin[2]) / ray[2]
        if t <= 0:                                # plane is behind the camera
            return None
        return origin + t * ray

    def camera_to_world(
        self,
        point_cam: np.ndarray,
        tf_buffer: tf2_ros.Buffer,
        target_frame: str = "world",
        stamp=None,
    ):
        """
        Transform point from camera frame to target_frame via TF2.
        target_frame is typically your robot base_link.
        """
        from rclpy.time import Time

        ps = PointStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = stamp or Time().to_msg()
        ps.point.x = float(point_cam[0])
        ps.point.y = float(point_cam[1])
        ps.point.z = float(point_cam[2])

        try:
            transformed = tf_buffer.transform(
                ps, target_frame, timeout=Duration(seconds=0.1)
            )
            return np.array([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ])
        except Exception as e:
            print(f"TF ERROR: {e}")
            return None