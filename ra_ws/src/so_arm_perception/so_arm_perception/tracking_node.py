# tracking_node.py
#
# Position-based visual servoing for the SO-ARM 101.
#
# Pipeline:
#   /detected_object/position (PointStamped, world frame)   ← perception_node
#        │
#        ▼  compute a grasp-approach target pose
#   /compute_ik  (MoveIt GetPositionIK, TRAC-IK position-only)
#        │
#        ▼  joint targets, eased toward each cycle
#   /arm_group_controller/joint_trajectory  (JointTrajectory)
#        │
#        ▼  JointTrajectoryController → hardware
#   sim:  topic_based_ros2_control → Isaac
#   real: swap the ros2_control hardware plugin to a Feetech interface — this
#         node, the IK, and the controller all stay identical (sim2real-portable).
#
# WHY joint-space + IK instead of MoveIt Servo twist:
#   The SO-ARM is 5-DOF. A 6-DOF Cartesian twist is structurally singular on a
#   5-DOF arm, so MoveIt Servo halts (HALT_FOR_SINGULARITY) across the workspace.
#   Position-only IK uses the 3xN position Jacobian, which is well-posed here, and
#   matches how the LeRobot/XLeRobot stack drives these arms (joint commands).

import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

# Arm joints in the order the controller expects (see ros2_controllers.yaml)
ARM_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]


def yaw_pitch_to_quat(yaw, pitch):
    """Quaternion (x,y,z,w) for a Z-yaw then Y-pitch rotation (roll=0)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    # R = Rz(yaw) * Ry(pitch)
    return (
        -sy * sp,          # x
        cy * sp,           # y
        sy * cp,           # z
        cy * cp,           # w
    )


class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("group", "arm_group")
        self.declare_parameter("ik_link", "gripper")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("object_topic", "/detected_object/position")
        self.declare_parameter("command_topic", "/arm_group_controller/joint_trajectory")
        self.declare_parameter("rate", 10.0)            # control loop Hz
        self.declare_parameter("standoff", 0.12)        # m, stand back from object along approach
        self.declare_parameter("z_offset", 0.0)         # m, raise/lower target vs object center
        self.declare_parameter("approach_pitch", 0.4)   # rad, gripper downward tilt (used by full IK)
        self.declare_parameter("gain", 0.35)            # 0..1 fraction of the way to IK solution per cycle
        self.declare_parameter("max_joint_step", 0.15)  # rad, per-cycle clamp (safety / smoothness)
        self.declare_parameter("target_timeout", 1.0)   # s, stop tracking if no fresh detection
        self.declare_parameter("ik_timeout", 0.05)      # s, per-call IK budget
        self.declare_parameter("avoid_collisions", True)
        # Anti-jitter: the detection is a noisy single-pixel unprojection, so
        # smooth it and stop commanding once we're essentially on target.
        self.declare_parameter("target_ema", 0.4)        # 0..1 weight on each new sample (lower = smoother)
        self.declare_parameter("pos_deadband", 0.01)     # m, ignore target moves smaller than this
        self.declare_parameter("joint_deadband", 0.01)   # rad, hold if IK solution is within this of current

        self.group = self.get_parameter("group").value
        self.ik_link = self.get_parameter("ik_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.standoff = self.get_parameter("standoff").value
        self.z_offset = self.get_parameter("z_offset").value
        self.approach_pitch = self.get_parameter("approach_pitch").value
        self.gain = self.get_parameter("gain").value
        self.max_step = self.get_parameter("max_joint_step").value
        self.target_timeout = self.get_parameter("target_timeout").value
        self.ik_timeout = self.get_parameter("ik_timeout").value
        self.avoid_collisions = self.get_parameter("avoid_collisions").value
        self.target_ema = self.get_parameter("target_ema").value
        self.pos_deadband = self.get_parameter("pos_deadband").value
        self.joint_deadband = self.get_parameter("joint_deadband").value
        rate = self.get_parameter("rate").value

        # ── State ───────────────────────────────────────────────────────────
        self.current_js = None          # latest JointState (all joints)
        self.target_point = None        # latest object position (world frame)
        self.target_stamp = None        # ros time of latest detection
        self._ik_busy = False           # guard: one in-flight IK call at a time

        cb = ReentrantCallbackGroup()

        # ── IO ──────────────────────────────────────────────────────────────
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=cb
        )
        self.cmd_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("command_topic").value, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10, callback_group=cb
        )
        self.create_subscription(
            PointStamped, self.get_parameter("object_topic").value,
            self._obj_cb, 10, callback_group=cb,
        )

        self.create_timer(1.0 / rate, self._loop, callback_group=cb)

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/compute_ik not available yet — will keep retrying")
        self.get_logger().info("Tracking node ready (position-based visual servoing)")

    # ── Callbacks ───────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        self.current_js = msg

    def _obj_cb(self, msg: PointStamped):
        p = msg.point
        self.target_stamp = self.get_clock().now()
        if self.target_point is None:
            self.target_point = p
            return
        # Ignore sub-deadband jitter so we don't chase detection noise near the goal.
        dx = p.x - self.target_point.x
        dy = p.y - self.target_point.y
        dz = p.z - self.target_point.z
        if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.pos_deadband:
            return
        # Exponential moving average: low-pass the noisy single-pixel unprojection.
        a = self.target_ema
        self.target_point.x += a * dx
        self.target_point.y += a * dy
        self.target_point.z += a * dz

    # ── Control loop ──────────────────────────────────────────────────────────
    def _loop(self):
        if self.current_js is None or self.target_point is None:
            return

        # Drop stale targets so the arm holds instead of chasing an old detection.
        age = (self.get_clock().now() - self.target_stamp).nanoseconds * 1e-9
        if age > self.target_timeout:
            return

        if self._ik_busy or not self.ik_client.service_is_ready():
            return

        target = self._approach_pose()

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = target
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.robot_state.joint_state = self.current_js  # seed from current pose
        req.ik_request.timeout = MsgDuration(
            sec=int(self.ik_timeout), nanosec=int((self.ik_timeout % 1) * 1e9)
        )

        self._ik_busy = True
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._ik_done)

    def _approach_pose(self) -> PoseStamped:
        """Grasp-approach target: stand off from the object along the base→object
        ray, mirroring the geometry used in mtc_node."""
        ox, oy, oz = self.target_point.x, self.target_point.y, self.target_point.z
        yaw = math.atan2(oy, ox)
        gx = ox - self.standoff * math.cos(yaw)
        gy = oy - self.standoff * math.sin(yaw)
        gz = oz + self.z_offset

        qx, qy, qz, qw = yaw_pitch_to_quat(yaw, self.approach_pitch)

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = gz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _ik_done(self, future):
        self._ik_busy = False
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"IK call failed: {e}")
            return

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"IK no solution (code {res.error_code.val})",
                throttle_duration_sec=2.0,
            )
            return

        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        cur = dict(zip(self.current_js.name, self.current_js.position))
        if not all(j in sol and j in cur for j in ARM_JOINTS):
            return

        # Convergence deadband: if every joint is essentially at the IK solution,
        # hold position instead of dithering on detection noise.
        if all(abs(sol[j] - cur[j]) < self.joint_deadband for j in ARM_JOINTS):
            return

        # Ease toward the IK solution (P-like) and clamp per-cycle step.
        cmd = []
        for j in ARM_JOINTS:
            delta = self.gain * (sol[j] - cur[j])
            delta = max(-self.max_step, min(self.max_step, delta))
            cmd.append(cur[j] + delta)

        self._publish(cmd)

    def _publish(self, positions):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        # Give the controller a short horizon so it interpolates smoothly.
        pt.time_from_start = RclpyDuration(seconds=0.2).to_msg()
        traj.points.append(pt)
        self.cmd_pub.publish(traj)


def main():
    rclpy.init()
    node = TrackingNode()
    # MultiThreadedExecutor so the async /compute_ik future resolves while the
    # timer / subscriptions keep spinning.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
