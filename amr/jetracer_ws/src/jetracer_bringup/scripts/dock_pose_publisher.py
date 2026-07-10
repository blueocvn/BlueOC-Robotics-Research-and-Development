#!/usr/bin/env python3
"""Republish an AprilTag dock detection as a PoseStamped for opennav_docking.

opennav_docking's SimpleChargingDock plugin (use_external_detection_pose: true)
consumes a single geometry_msgs/PoseStamped on `detected_dock_pose`. apriltag_ros
does not publish that -- it publishes an AprilTagDetectionArray (no pose) plus a
TF frame per detected tag. This node bridges the gap: it looks up the TF of each
configured dock tag relative to the camera optical frame and republishes the pose
of the currently-visible tag (the nearest one, if several are in view).

The pose is stamped in `camera_frame` on purpose: SimpleChargingDock applies its
external_detection_rotation_* / _translation_* corrections assuming the detection
arrives in the camera optical convention, then transforms it into `fixed_frame`
itself.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')

        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('dock_frames', ['dock_0', 'dock_1', 'dock_2'])
        self.declare_parameter('detection_topic', 'detected_dock_pose')
        self.declare_parameter('publish_rate', 15.0)
        # Ignore detections older than this (a tag that left the frame).
        self.declare_parameter('detection_timeout', 0.5)

        self.camera_frame = self.get_parameter('camera_frame').value
        self.dock_frames = list(self.get_parameter('dock_frames').value)
        topic = self.get_parameter('detection_topic').value
        rate = float(self.get_parameter('publish_rate').value)
        self.timeout = float(self.get_parameter('detection_timeout').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'dock_pose_publisher: camera_frame={self.camera_frame}, '
            f'dock_frames={self.dock_frames}, topic={topic}')

    def _tick(self):
        best = None  # (distance, PoseStamped)
        for frame in self.dock_frames:
            try:
                # Latest available transform of the tag in the camera frame.
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, frame, rclpy.time.Time())
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                continue

            # Reject stale detections (apriltag stops publishing TF for tags
            # that leave the view, but the last transform lingers in the buffer).
            stamp = tf.header.stamp
            age = self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
            if age > Duration(seconds=self.timeout):
                continue

            t = tf.transform.translation
            dist = (t.x * t.x + t.y * t.y + t.z * t.z) ** 0.5

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.camera_frame
            pose.pose.position.x = t.x
            pose.pose.position.y = t.y
            pose.pose.position.z = t.z
            pose.pose.orientation = tf.transform.rotation

            if best is None or dist < best[0]:
                best = (dist, pose)

        if best is not None:
            self.pub.publish(best[1])


def main(args=None):
    rclpy.init(args=args)
    node = DockPosePublisher()
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
