#!/usr/bin/env python3
"""Ackermann steering adapter for the opennav_docking graceful controller.

The graceful controller outputs (v, ω) assuming a diff-drive robot that can
rotate in place. The JetRacer is Ackermann (car-like): the maximum achievable
angular velocity at speed v is:

    ω_max(v) = v * tan(δ_max) / L

where L is the wheelbase and δ_max is the maximum steering angle.  Commanding
ω > ω_max(v) saturates the firmware silently, so the robot follows a larger arc
than the controller expects, causing the L-shape spiral and direction reversal.

This node:
  1. Subscribes to the docking server's raw cmd_vel (remapped topic).
  2. Clamps ω to ω_max(v) for the current linear speed.
  3. Gates ω to zero when |v| < v_min to avoid the singularity δ→90° at
     near-zero speed (would command full lock, causing the robot to spin).
  4. Republishes the corrected command on /cmd_vel.

Wire-up (nav_bringup.launch.py):
  - docking_server Node gets remappings=[('cmd_vel', '/docking/cmd_vel')]
  - this node is launched alongside it (no remapping needed)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AckermannDockFilter(Node):
    def __init__(self):
        super().__init__('ackermann_dock_filter')

        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('delta_max_deg', 30.0)
        self.declare_parameter('v_min_threshold', 0.02)
        self.declare_parameter('input_topic', '/docking/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')

        self.L = self.get_parameter('wheelbase').value
        delta_deg = self.get_parameter('delta_max_deg').value
        self.tan_delta_max = math.tan(math.radians(delta_deg))
        self.v_min = self.get_parameter('v_min_threshold').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self._cb, 10)

        self.get_logger().info(
            f'ackermann_dock_filter: L={self.L}m δ_max={delta_deg}° '
            f'v_min={self.v_min} {in_topic} → {out_topic}')

    def _cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        if abs(v) < self.v_min:
            # Near zero speed: commanding any ω would require δ→90°.
            # Drive straight and let the graceful controller converge in
            # position before trying to steer.
            w = 0.0
        else:
            # Maximum physically achievable angular velocity at this speed.
            w_max = abs(v) * self.tan_delta_max / self.L
            w = max(-w_max, min(w_max, w))

        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDockFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
