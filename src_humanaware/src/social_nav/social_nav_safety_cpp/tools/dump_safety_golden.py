#!/usr/bin/env python3
"""Run the Python safety node on a fixed scenario and record every step.

lidar_safety is the last guard before the motors. If the C++ version diverged
from the Python one in some branch (e.g. a latch that never releases, or releases
one tick early) nothing would warn about it, so the comparison is step by step.

No ROS graph: a real node is created (rclpy is needed for declare_parameter), its
clock is replaced by a fake one, and scan_cb / raw_cmd_cb / control_loop are
called directly, so time is fully controlled and the result is reproducible.

Every float is written with float.hex() (C99 %a) and read back with std::strtod.

Run: python3 tools/dump_safety_golden.py   (overwrites test/golden/safety_trace.txt)
"""

import math
import os
import sys

import rclpy
import rclpy.node
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from social_nav_safety.lidar_safety_node1 import LidarSafetyNode

GOLDEN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test", "golden")

RAY_COUNT = 72                 # 5 degrees per beam -- enough for every branch, small enough to write out
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / RAY_COUNT
RANGE_MIN = 0.12
RANGE_MAX = 12.0


def hx(value):
    return float(value).hex()


class FakeClock:
    """Clock set by the script. The node only calls get_clock().now().nanoseconds."""

    class _Now:
        def __init__(self, nanoseconds):
            self.nanoseconds = nanoseconds

    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        return FakeClock._Now(self.nanoseconds)


def make_scan(front=None, rear=None, left=None, right=None):
    """Build one scan: a 3-beam cluster per direction, infinity elsewhere.

    front/rear/left/right are distances (m) or None if that direction is clear.
    """
    ranges = [float("inf")] * RAY_COUNT

    def place(angle, distance):
        if distance is None:
            return
        for offset in (-1, 0, 1):
            index = int(round((angle - ANGLE_MIN) / ANGLE_INCREMENT)) + offset
            index %= RAY_COUNT
            ranges[index] = float(distance)

    place(0.0, front)
    place(math.pi, rear)
    place(math.pi / 2.0, left)
    place(-math.pi / 2.0, right)
    return ranges


def build_script():
    """Scenario: each element is (time in seconds, kind, data).

    Deliberately walks through EVERY branch of control_loop:
       1. no command / scan yet        -> stale-data stop
       2. clear path                   -> passed through unchanged
       3. obstacle approaching         -> closing_speed + distance slowdown
       4. inside stop_distance         -> latch + rotate towards the escape side
       5. obstacle moves away          -> latch must be HELD for release_hold_time
       6. turn command flips every tick -> _hold_sign
       7. forward/back flips every tick -> _hold_linear_sign
       8. scans stop arriving          -> stale-data stop
       9. reverse with obstacle behind -> rear_clearance branch
      10. clear again, rotate towards a narrow side -> _gate_angular
      11. latched forward, planner asks to back off -> escape_away branch
    """
    script = []
    tick = 0.05   # 20 Hz, the default control_rate

    # (1) three ticks before any data
    t = 1.0
    for _ in range(3):
        script.append((t, "tick", None))
        t += tick

    # (2) clear path, straight ahead
    for step in range(6):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "scan", make_scan(front=6.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (3) obstacle approaching -- triggers closing_speed, then slowdown
    for distance in (3.0, 2.4, 1.8, 1.4, 1.1, 0.95, 0.85):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "scan", make_scan(front=distance, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (4) well inside stop_distance -- latch, rotate to find a way out.
    #     The left side is wider, so the escape direction must be +1 (left).
    for _ in range(8):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "scan", make_scan(front=0.55, left=5.0, right=1.2)))
        script.append((t, "tick", None))
        t += tick

    # (5) obstacle moves beyond the release threshold -- but the latch must be HELD
    #     for release_hold_time (0.30 s = 6 ticks) before it really releases
    for _ in range(10):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "scan", make_scan(front=5.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (6) turn command flips sign every tick -- _hold_sign must damp it
    for step in range(10):
        turn = 0.4 if step % 2 == 0 else -0.4
        script.append((t, "cmd", (0.0, turn)))
        script.append((t, "scan", make_scan(front=5.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (7) forward/back command flips every tick -- _hold_linear_sign must damp it
    for step in range(10):
        linear = 0.25 if step % 2 == 0 else -0.25
        script.append((t, "cmd", (linear, 0.0)))
        script.append((t, "scan", make_scan(front=5.0, rear=5.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (8) scans stop: only commands, no scan -> beyond scan_timeout (0.25 s)
    for _ in range(8):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "tick", None))
        t += tick

    # (9) reversing with an obstacle behind -- rear_clearance branch
    for distance in (2.0, 1.2, 0.7, 0.40, 0.35):
        script.append((t, "cmd", (-0.25, 0.0)))
        script.append((t, "scan", make_scan(rear=distance, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (10) clear again, rotating in place towards a narrow side -> _gate_angular
    for _ in range(8):
        script.append((t, "cmd", (0.0, 0.5)))
        script.append((t, "scan", make_scan(front=5.0, left=0.10, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    # (11) ESCAPE: a person right in front -> forward latch, then the planner asks to
    #      back off. The rear is clear, so the safety layer must ALLOW reversing and
    #      still keep the latch (escape_away branch).
    for _ in range(6):
        script.append((t, "cmd", (0.30, 0.0)))
        script.append((t, "scan", make_scan(front=0.35, rear=5.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick
    for _ in range(8):
        script.append((t, "cmd", (-0.25, 0.0)))
        script.append((t, "scan", make_scan(front=0.35, rear=5.0, left=5.0, right=5.0)))
        script.append((t, "tick", None))
        t += tick

    return script


def main():
    rclpy.init()

    # Parameters are read DIRECTLY from the deployed YAML; copying them by hand here
    # would let the two sides drift apart unnoticed.
    yaml_path = os.path.join(
        get_package_share_directory("social_nav_safety"),
        "config", "lidar_safety_params.yaml")
    with open(yaml_path) as handle:
        yaml_values = yaml.safe_load(handle)["lidar_safety_node1"]["ros__parameters"]
    # Keep bool values as bool: float(True) = 1.0 would mismatch the declared type
    # and the node would fail at start-up.
    overrides = [
        Parameter(name, value=(value if isinstance(value, bool) else float(value)))
        for name, value in yaml_values.items()
    ]

    # LidarSafetyNode.__init__ calls super().__init__("lidar_safety_node") without
    # parameter_overrides, so Node.__init__ is patched for this one construction.
    # Ugly, but the alternative would be modifying the Python reference itself.
    original_init = rclpy.node.Node.__init__

    def patched_init(self, node_name, **kwargs):
        kwargs["parameter_overrides"] = overrides
        original_init(self, node_name, **kwargs)

    rclpy.node.Node.__init__ = patched_init
    try:
        node = LidarSafetyNode()
    finally:
        rclpy.node.Node.__init__ = original_init

    # Assert that the parameters really reached the node; otherwise golden data would
    # silently be generated with hard-coded defaults.
    for name, expected in yaml_values.items():
        actual = node.get_parameter(name).value
        if abs(float(actual) - float(expected)) > 0.0:
            print(f"ERROR: parameter {name} = {actual}, YAML has {expected}")
            return 1

    clock = FakeClock()
    node.get_clock = lambda: clock

    published = []
    node.safe_cmd_pub.publish = published.append

    records = []
    for seconds, kind, payload in build_script():
        clock.nanoseconds = int(round(seconds * 1e9))

        if kind == "cmd":
            message = Twist()
            message.linear.x = float(payload[0])
            message.angular.z = float(payload[1])
            node.raw_cmd_cb(message)
            records.append({
                "t": clock.nanoseconds,
                "kind": "cmd",
                "cmd": [hx(payload[0]), hx(payload[1])],
            })

        elif kind == "scan":
            scan = LaserScan()
            scan.angle_min = ANGLE_MIN
            scan.angle_increment = ANGLE_INCREMENT
            scan.range_min = RANGE_MIN
            scan.range_max = RANGE_MAX
            scan.ranges = [float(v) for v in payload]
            node.scan_cb(scan)
            # Record the float32 value ROS actually stores, not the original Python number --
            # the C++ side reads float32 from the message, so both must match what it sees.
            records.append({
                "t": clock.nanoseconds,
                "kind": "scan",
                "geom": [hx(scan.angle_min), hx(scan.angle_increment),
                         hx(scan.range_min), hx(scan.range_max)],
                "n": len(scan.ranges),
                "r": [hx(v) for v in scan.ranges],
                "state": [hx(node.front_clearance), hx(node.rear_clearance),
                          hx(node.left_clearance), hx(node.right_clearance),
                          hx(node.closing_speed)],
            })

        elif kind == "tick":
            before = len(published)
            node.control_loop()
            if len(published) != before + 1:
                print(f"ERROR: control_loop did not publish exactly 1 message at t={seconds}")
                return 1
            out = published[-1]
            records.append({
                "t": clock.nanoseconds,
                "kind": "tick",
                "out": [hx(out.linear.x), hx(out.angular.z)],
                "latched": 1 if node.stop_latched else 0,
            })

    os.makedirs(GOLDEN_DIR, exist_ok=True)
    path = os.path.join(GOLDEN_DIR, "safety_trace.txt")
    with open(path, "w") as handle:
        for record in records:
            handle.write("--\n")
            handle.write(f"t {record['t']}\n")
            handle.write(f"kind {record['kind']}\n")
            for key in ("geom", "cmd", "out", "state", "r"):
                if key in record:
                    handle.write(f"{key} " + " ".join(record[key]) + "\n")
            for key in ("n", "latched"):
                if key in record:
                    handle.write(f"{key} {record[key]}\n")

    ticks = sum(1 for r in records if r["kind"] == "tick")
    scans = sum(1 for r in records if r["kind"] == "scan")
    print(f"safety_trace.txt: {len(records)} records ({ticks} ticks, {scans} scans)")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
