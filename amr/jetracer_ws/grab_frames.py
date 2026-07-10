#!/usr/bin/env python3
"""
Headless frame grabber for camera calibration.
Subscribes to an image topic and saves frames that contain a detected
checkerboard. No GUI, no display needed -- safe over pure SSH.

Usage:
    python3 grab_frames.py --topic /image_raw --cols 8 --rows 6 --out ./calib_frames

Point the camera at a checkerboard shown on a screen (or printed).
Move it around: near/far, all corners, tilted. The script auto-saves a frame
each time it finds the full board and you've moved enough since the last save.
Stop with Ctrl-C once you have ~40 saved frames spread across the view.
"""
import argparse, os, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Grabber(Node):
    def __init__(self, topic, cols, rows, out, min_move, target):
        super().__init__('calib_grabber')
        self.cols, self.rows = cols, rows
        self.out = out
        self.min_move = min_move
        self.target = target
        os.makedirs(out, exist_ok=True)
        self.bridge = CvBridge()
        self.saved = 0
        self.last_center = None
        self.sub = self.create_subscription(Image, topic, self.cb, 1)
        self.get_logger().info(f'Listening on {topic}. Looking for {cols}x{rows} inner corners.')
        self.get_logger().info(f'Saving to {out}. Target ~{target} frames. Ctrl-C when done.')

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'convert failed: {e}')
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return
        center = corners.mean(axis=0).ravel()
        if self.last_center is not None:
            if np.linalg.norm(center - self.last_center) < self.min_move:
                return  # too close to last saved pose; keep moving
        self.last_center = center
        fname = os.path.join(self.out, f'frame_{self.saved:03d}.png')
        cv2.imwrite(fname, img)
        self.saved += 1
        self.get_logger().info(f'saved {self.saved}/{self.target}: {fname}')
        if self.saved >= self.target:
            self.get_logger().info('Target reached -- you can Ctrl-C now (or keep going).')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/image_raw')
    ap.add_argument('--cols', type=int, default=8, help='inner corners across')
    ap.add_argument('--rows', type=int, default=6, help='inner corners down')
    ap.add_argument('--out', default='./calib_frames')
    ap.add_argument('--min-move', type=float, default=40.0,
                    help='min pixel move of board center before saving another frame')
    ap.add_argument('--target', type=int, default=40)
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.topic, args.cols, args.rows, args.out, args.min_move, args.target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Done. {node.saved} frames in {args.out}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
