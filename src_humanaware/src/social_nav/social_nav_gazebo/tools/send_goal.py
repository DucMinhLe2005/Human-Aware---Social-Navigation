#!/usr/bin/env python3
"""Send the robot to one goal and wait until it arrives (or times out).

Prints the outcome: GOAL_REACHED | <seconds>, or the reason it failed.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class GoalSender(Node):

    def __init__(self):
        super().__init__('send_goal')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def run(self, x, y, yaw, timeout):
        if not self.nav.wait_for_server(timeout_sec=20.0):
            print('NO navigate_to_pose action server')
            return 2
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # bt_navigator occasionally loses the goal response ("Failed to send goal
        # response (timeout)") although the goal was accepted. Retry in that case;
        # only report a rejection when the server actually rejects the goal.
        handle = None
        for attempt in range(3):
            future = self.nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            handle = future.result()
            if handle is None:
                print(f'(goal response lost, attempt {attempt + 1}, retrying)', file=sys.stderr)
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                continue
            if not handle.accepted:
                print('GOAL_REJECTED')
                return 3
            break
        if handle is None:
            print('GOAL_RESPONSE_LOST_3_TIMES')
            return 5
        start = time.time()
        result_future = handle.get_result_async()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.3)
            if result_future.done():
                status = result_future.result().status
                succeeded = status == 4  # GoalStatus.STATUS_SUCCEEDED
                outcome = 'GOAL_REACHED' if succeeded else f'GOAL_ENDED status={status}'
                print(outcome, f'| {time.time() - start:.1f}s')
                return 0 if succeeded else 1
        print(f'TIMEOUT after {timeout:.0f}s')
        return 4


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = GoalSender()
    try:
        code = node.run(args.x, args.y, args.yaw, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
