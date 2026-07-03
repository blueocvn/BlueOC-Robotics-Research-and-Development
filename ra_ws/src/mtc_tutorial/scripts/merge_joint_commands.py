#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MergeJointCommands(Node):
    def __init__(self):
        super().__init__('merge_joint_commands')
        self.arm_msg = None
        self.finger_msg = None
        self.sub = self.create_subscription(
            JointState, '/isaac_joint_commands', self.cb, 10)
        self.pub = self.create_publisher(
            JointState, '/isaac_joint_commands_merged', 10)
        self.create_timer(0.01, self.publish_merged)  # 100Hz

    def cb(self, msg):
        if 'panda_joint1' in msg.name:
            self.arm_msg = msg
        elif 'panda_finger_joint1' in msg.name:
            self.finger_msg = msg

    def publish_merged(self):
        if self.arm_msg is None or self.finger_msg is None:
            return
        merged = JointState()
        merged.header = self.arm_msg.header
        merged.name = self.arm_msg.name + self.finger_msg.name
        merged.position = list(self.arm_msg.position) + list(self.finger_msg.position)
        self.pub.publish(merged)

def main():
    rclpy.init()
    rclpy.spin(MergeJointCommands())
    rclpy.shutdown()

if __name__ == '__main__':
    main()