#!/usr/bin/env python3
"""Re-initialise AMCL from the GROUND-TRUTH pose (simulation only).

Restarting navigation resets AMCL. Re-seeding it from its own previous pose would
carry the old error over and accumulate it across restarts, so the ground-truth
pose from /social_nav/ground_truth is used instead.

Usage: python3 reset_amcl.py [--check]   (--check: only report the error)
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage


class AmclReset(Node):

    def __init__(self):
        super().__init__('reset_amcl')
        self.ground_truth = {}
        self.amcl = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/social_nav/ground_truth', self._on_ground_truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def _on_ground_truth(self, msg):
        for t in msg.transforms:
            self.ground_truth[t.child_frame_id] = t.transform

    def _on_amcl(self, msg):
        self.amcl = msg

    def wait(self, seconds=30):
        # Wait for BOTH ground truth and an AMCL pose; AMCL starts later.
        start = time.time()
        while time.time() - start < seconds and (
                'my_amr' not in self.ground_truth or self.amcl is None):
            rclpy.spin_once(self, timeout_sec=0.05)
        return 'my_amr' in self.ground_truth and self.amcl is not None

    def error(self):
        t = self.ground_truth['my_amr']
        p = self.amcl.pose.pose
        return math.hypot(p.position.x - t.translation.x, p.position.y - t.translation.y)

    def seed(self):
        t = self.ground_truth['my_amr']
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = t.translation.x
        msg.pose.pose.position.y = t.translation.y
        msg.pose.pose.orientation = t.rotation
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.4)
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = AmclReset()
    try:
        if not node.wait():
            print('NO ground truth / amcl_pose')
            return 2
        before = node.error()
        if args.check:
            print(f'AMCL error: {before:.2f} m')
            return 0 if before <= 0.3 else 1
        print(f'AMCL error before seeding: {before:.2f} m')
        # Up to 3 attempts: with a large error one seed may not pull the particles back.
        for attempt in range(1, 4):
            node.seed()
            start = time.time()
            while time.time() - start < 12:
                rclpy.spin_once(node, timeout_sec=0.05)
            after = node.error()
            print(f'  attempt {attempt}: {after:.2f} m')
            if after <= 0.3:
                print('after seeding: OK')
                return 0
        print('STILL OFF after 3 attempts -- do not measure this run')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
