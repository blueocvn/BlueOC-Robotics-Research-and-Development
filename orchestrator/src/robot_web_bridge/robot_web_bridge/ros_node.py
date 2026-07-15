"""rclpy bridge node — the one process that actually talks to the robot.

Runs a ROS 2 node on a daemon thread (SingleThreadedExecutor.spin()). Web
handlers / the dispatcher call the thread-safe ``publish_*`` methods; the node
keeps a thread-safe cache of the latest ``/docking_state`` and robot pose.

rclpy + the message packages are only importable when ROS 2 is sourced, so the
imports are guarded: if they fail (e.g. local dev in a plain venv), ``RCLPY_OK``
is False and :func:`start_ros_node` returns ``None`` — the app then falls back to
the simulated backend. Nothing else in the package imports rclpy directly.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    RCLPY_OK = True
    _IMPORT_ERR: Optional[Exception] = None

    # Best-effort, keep-last so we stay compatible with either a reliable or a
    # best-effort publisher on the robot, and always read the freshest value.
    _SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    # Latched (transient-local) so a nav node that starts *after* the operator
    # saved the map still receives the current obstacle set.
    _LATCHED_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    class RobotBridgeNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("robot_web_bridge")
            self._lock = threading.Lock()
            self._docking_state: Optional[str] = None
            self._odom: Optional[tuple[float, float, float]] = None  # x, y, yaw
            self._odom_source: Optional[str] = None

            # Publishers — the robot's command interface.
            self._pub_dock = self.create_publisher(String, "/dock_robot", 10)
            self._pub_abort = self.create_publisher(Bool, "/abort_docking", 10)
            self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
            self._pub_initialpose = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )
            # Admin map editor → robot: JSON list of keep-out rectangles.
            self._pub_obstacles = self.create_publisher(
                String, "/virtual_obstacles", _LATCHED_QOS
            )
            # Admin map editor → robot: JSON dock registry with pose_x/pose_y/yaw.
            self._pub_docks = self.create_publisher(
                String, "/dock_registry", _LATCHED_QOS
            )
            # Admin "reset pose": rotate to find the dock's AprilTag, then
            # /initialpose at that dock's surveyed pose (jetracer_docker owns it).
            self._pub_relocalize = self.create_publisher(
                String, "/relocalize_at_dock", 10
            )

            # Subscribers — live feedback.
            self.create_subscription(
                String, "/docking_state", self._on_docking_state, _SENSOR_QOS
            )
            # Prefer EKF-filtered odometry for map pose. Raw chassis odom remains
            # as fallback when /odometry/filtered is unavailable.
            self.create_subscription(
                Odometry, "/odometry/filtered", self._on_odom_filtered, _SENSOR_QOS
            )
            self.create_subscription(
                Odometry, "/chassis/odom", self._on_odom_raw, _SENSOR_QOS
            )
            self.get_logger().info("robot_web_bridge node up: pubs+subs registered")

        # ── subscriptions ────────────────────────────────────────────────────
        def _on_docking_state(self, msg: "String") -> None:
            with self._lock:
                self._docking_state = msg.data

        @staticmethod
        def _odom_tuple(msg: "Odometry") -> tuple[float, float, float]:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return (p.x, p.y, yaw)

        def _on_odom_filtered(self, msg: "Odometry") -> None:
            with self._lock:
                self._odom = self._odom_tuple(msg)
                self._odom_source = "filtered"

        def _on_odom_raw(self, msg: "Odometry") -> None:
            # Only use raw odom if filtered odom has not been seen yet.
            with self._lock:
                if self._odom_source != "filtered":
                    self._odom = self._odom_tuple(msg)
                    self._odom_source = "raw"

        # ── publishers (thread-safe; rclpy publish is itself safe) ────────────
        def publish_dock(self, dock_id: str) -> None:
            self._pub_dock.publish(String(data=dock_id))

        def publish_abort(self) -> None:
            self._pub_abort.publish(Bool(data=True))

        def publish_cmd_vel(self, linear: float, angular: float) -> None:
            t = Twist()
            t.linear.x = float(linear)
            t.angular.z = float(angular)
            self._pub_cmd_vel.publish(t)

        def publish_initialpose(self, x: float, y: float, yaw: float) -> None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            self._pub_initialpose.publish(msg)

        def publish_obstacles(self, payload: str) -> None:
            self._pub_obstacles.publish(String(data=payload))

        def publish_docks(self, payload: str) -> None:
            self._pub_docks.publish(String(data=payload))

        def publish_relocalize(self, dock_id: str) -> None:
            self._pub_relocalize.publish(String(data=dock_id))

        # ── snapshot ──────────────────────────────────────────────────────────
        def get_state(self) -> dict:
            with self._lock:
                return {
                    "docking_state": self._docking_state,
                    "odom": self._odom,
                    "odom_source": self._odom_source,
                }

except Exception as exc:  # pragma: no cover - depends on ROS being sourced
    RCLPY_OK = False
    _IMPORT_ERR = exc
    RobotBridgeNode = None  # type: ignore[assignment]


def start_ros_node():
    """Spin a :class:`RobotBridgeNode` on a daemon thread.

    Returns the node (with ``._executor`` attached for shutdown) or ``None`` if
    rclpy is unavailable / init fails — caller falls back to simulation.
    """
    if not RCLPY_OK:
        return None
    try:
        rclpy.init()
        node = RobotBridgeNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-spin")
        thread.start()
        node._executor = executor  # type: ignore[attr-defined]
        return node
    except Exception:  # pragma: no cover
        return None


def shutdown_ros_node(node) -> None:
    if node is None:
        return
    try:
        executor = getattr(node, "_executor", None)
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:  # pragma: no cover
        pass
