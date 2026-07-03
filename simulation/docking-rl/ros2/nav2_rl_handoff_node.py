#!/usr/bin/env python3
"""Nav2 -> RL-approach -> (external) AprilTag-docking handoff state machine.

Section 7.6 of the setup guide. Scope, deliberately narrow: this node owns exactly two
transitions -- handing off from Nav2's ``navigate_to_pose`` to the trained RL policy once the
robot is close to the staging pose, and handing off from the RL policy to whatever docks against
the AprilTag once the staging pose is reached. It does **not** implement AprilTag detection or
visual servoing itself (Section 6.4 of the guide) -- that is intentionally a separate node/system;
this RL task (see ``../source/docking_rl``) only ever targets the staging pose and has no notion
of the tag.

    IDLE -> NAV2_APPROACH (Nav2 action client drives to the staging pose)
         -> [within handoff_radius] -> cancel Nav2 goal -> RL_APPROACH (policy publishes /cmd_vel)
         -> [staging tolerance met] -> DOCKING_HANDOFF (publish a trigger for the external
            AprilTag-docking node/service; this node's job ends here) -> DONE

An independent, always-on safety stop (minimum LiDAR range check) is not learned and preempts
whichever controller is active, regardless of state.

This is a skeleton, not a finished integration: topic names, the docking-handoff trigger
mechanism, and the policy-loading code are stubbed with TODOs -- they depend on the JetRacer's
actual ROS2 topic names/message types and on how the trained skrl checkpoint gets exported for
inference, neither of which is settled yet. The ``/cmd_vel`` (Twist) output does match the
existing on-robot conversion node at
``jetracer_ws/src/ackermann_control/cmdvel_to_ackermann``, which already publishes
``AckermannDriveStamped`` from ``Twist`` -- so no new conversion node should be needed there.
"""

from __future__ import annotations

import enum
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class State(enum.Enum):
    IDLE = enum.auto()
    NAV2_APPROACH = enum.auto()
    RL_APPROACH = enum.auto()
    DOCKING_HANDOFF = enum.auto()
    DONE = enum.auto()
    SAFETY_STOP = enum.auto()


class Nav2RlHandoffNode(Node):
    def __init__(self):
        super().__init__("nav2_rl_handoff_node")

        # -- parameters -----------------------------------------------------------------------
        self.declare_parameter("staging_pose_x", 0.0)
        self.declare_parameter("staging_pose_y", 0.0)
        self.declare_parameter("staging_pose_yaw", 0.0)
        self.declare_parameter("handoff_radius_m", 1.75)  # switch Nav2 -> RL within this range
        self.declare_parameter("staging_pos_tolerance_m", 0.3)
        self.declare_parameter("staging_heading_tolerance_rad", 0.175)
        self.declare_parameter("min_safe_range_m", 0.15)  # always-on LiDAR safety stop
        self.declare_parameter("control_rate_hz", 20.0)

        # -- state --------------------------------------------------------------------------
        self._state = State.IDLE
        self._latest_pose: PoseWithCovarianceStamped | None = None
        self._latest_scan: LaserScan | None = None
        self._nav2_goal_handle = None

        # -- I/O --------------------------------------------------------------------------
        # TODO(jetracer): confirm these topic names against the real JetRacer bring-up
        # (jetracer_ws/run_workstation.sh + the on-robot launch files) before deploying.
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
        )
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # TODO(jetracer): replace with whatever trigger the AprilTag-docking node/service expects
        # (e.g. an action goal, a service call, or a simple std_msgs/Empty/Bool topic).
        self._docking_handoff_pub = self.create_publisher(Twist, "/docking/handoff_trigger", 1)

        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # TODO: load the trained skrl checkpoint / build the observation vector expected by
        # StagingDockEnvCfg's ObservationsCfg (staging_pose_error, base_lin_vel, base_ang_vel,
        # steering_angle, last_action). Needs an export step from the training checkpoint that
        # doesn't exist yet.
        self._policy = None

        rate = self.get_parameter("control_rate_hz").value
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info("nav2_rl_handoff_node started (state=IDLE)")

    # -- subscriptions ------------------------------------------------------------------------

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # -- safety (always evaluated, independent of state) --------------------------------------

    def _min_range_violated(self) -> bool:
        if self._latest_scan is None:
            return False
        min_range = self.get_parameter("min_safe_range_m").value
        ranges = [r for r in self._latest_scan.ranges if math.isfinite(r) and r > 0.0]
        return bool(ranges) and min(ranges) < min_range

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    # -- staging-pose geometry ------------------------------------------------------------------

    def _distance_and_heading_error_to_staging(self) -> tuple[float, float] | None:
        if self._latest_pose is None:
            return None
        p = self._latest_pose.pose.pose.position
        q = self._latest_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        goal_x = self.get_parameter("staging_pose_x").value
        goal_y = self.get_parameter("staging_pose_y").value
        goal_yaw = self.get_parameter("staging_pose_yaw").value

        dist = math.hypot(goal_x - p.x, goal_y - p.y)
        heading_err = math.atan2(math.sin(goal_yaw - yaw), math.cos(goal_yaw - yaw))
        return dist, heading_err

    # -- main loop ------------------------------------------------------------------------------

    def _tick(self) -> None:
        if self._min_range_violated() and self._state != State.SAFETY_STOP:
            self.get_logger().warn("min-range safety stop triggered")
            self._publish_stop()
            self._pre_safety_state = self._state
            self._state = State.SAFETY_STOP
            return
        if self._state == State.SAFETY_STOP:
            if not self._min_range_violated():
                self._state = self._pre_safety_state
            else:
                self._publish_stop()
                return

        if self._state == State.IDLE:
            self._start_nav2_approach()
        elif self._state == State.NAV2_APPROACH:
            self._tick_nav2_approach()
        elif self._state == State.RL_APPROACH:
            self._tick_rl_approach()
        elif self._state == State.DOCKING_HANDOFF:
            self._tick_docking_handoff()
        # DONE: nothing to do

    # -- state implementations -------------------------------------------------------------------

    def _start_nav2_approach(self) -> None:
        dist_heading = self._distance_and_heading_error_to_staging()
        if dist_heading is None:
            return  # wait for the first /amcl_pose
        self.get_logger().info("sending Nav2 goal toward the staging pose")
        # TODO: send a NavigateToPose goal at (staging_pose_x, staging_pose_y, staging_pose_yaw).
        # Left unimplemented -- the goal-sending/result-callback boilerplate is standard Nav2
        # client code and doesn't depend on anything specific to this task.
        self._state = State.NAV2_APPROACH

    def _tick_nav2_approach(self) -> None:
        dist_heading = self._distance_and_heading_error_to_staging()
        if dist_heading is None:
            return
        dist, _ = dist_heading
        if dist <= self.get_parameter("handoff_radius_m").value:
            self.get_logger().info(f"within handoff radius ({dist:.2f} m) -- cancelling Nav2 goal")
            if self._nav2_goal_handle is not None:
                self._nav2_goal_handle.cancel_goal_async()
            self._state = State.RL_APPROACH

    def _tick_rl_approach(self) -> None:
        dist_heading = self._distance_and_heading_error_to_staging()
        if dist_heading is None:
            return
        dist, heading_err = dist_heading
        if (
            dist < self.get_parameter("staging_pos_tolerance_m").value
            and abs(heading_err) < self.get_parameter("staging_heading_tolerance_rad").value
        ):
            self.get_logger().info("staging pose reached -- handing off to AprilTag docking")
            self._publish_stop()
            self._state = State.DOCKING_HANDOFF
            return

        if self._policy is None:
            # policy not loaded yet -- hold position rather than drive blind
            self._publish_stop()
            return

        # TODO: build the observation vector (see StagingDockEnvCfg.ObservationsCfg) from
        # (dist, heading_err, current velocity, steering angle, last action, downsampled /scan
        # per Section 7.1), run the policy, and publish the resulting Twist. Left unimplemented
        # pending the checkpoint-export step noted above.
        self._publish_stop()

    def _tick_docking_handoff(self) -> None:
        # Trigger the external AprilTag docking system and consider this node's job done. If
        # docking needs to report failure back here (e.g. tag lost -> revert to RL/backup
        # maneuver per Section 6.5), that channel isn't defined yet -- TODO.
        self._docking_handoff_pub.publish(Twist())
        self._state = State.DONE
        self.get_logger().info("docking handoff sent (state=DONE)")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2RlHandoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
