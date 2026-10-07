#!/usr/bin/env python3
"""Filter raw velocity commands using fresh LiDAR observations.

Last physical safety layer, fully independent of the AGHPM costmap, the
human-aware controller and camera perception/tracking: it only reads /scan and
/cmd_vel_raw and publishes the filtered /cmd_vel. If anything upstream is wrong,
crashes or lags, lidar still guards the robot before the command reaches the
motor driver. velocity_smoother publishes to cmd_vel_raw so this node sits
between the smoother and the driver.
"""

import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSafetyNode(Node):
    """Publish safe velocity commands after checking LiDAR freshness."""

    def __init__(self):
        """Initialize subscriptions, publisher, and safety timer."""
        super().__init__("lidar_safety_node")

        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("scan_timeout", 0.25)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("robot_front_extent", 0.18)
        self.declare_parameter("front_half_width", 0.22)
        self.declare_parameter("robot_rear_extent", 0.18)
        self.declare_parameter("rear_half_width", 0.22)
        self.declare_parameter("robot_safety_radius", 0.20)
        self.declare_parameter("turn_slow_distance", 0.15)
        self.declare_parameter("turn_stop_distance", 0.03)
        self.declare_parameter("slow_distance", 0.80)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("emergency_distance", 0.30)
        self.declare_parameter("ttc_slow", 2.0)
        self.declare_parameter("ttc_stop", 1.0)
        self.declare_parameter("closing_speed_alpha", 0.35)
        self.declare_parameter("max_closing_speed", 3.0)
        self.declare_parameter("release_distance", 0.60)
        self.declare_parameter("release_ttc", 1.50)
        self.declare_parameter("release_hold_time", 0.30)
        self.declare_parameter("escape_angular_speed", 0.3)
        self.escape_angular_speed = float(
            self.get_parameter("escape_angular_speed").value
        )

        # Escape while latched: when one direction is blocked, still allow the OPPOSITE
        # direction if its own corridor is clear. Standing still while a person
        # approaches is wrong; the robot must move forward or back off to make way.
        self.declare_parameter("escape_away_enabled", True)
        self.escape_away_enabled = bool(
            self.get_parameter("escape_away_enabled").value
        )
        self.declare_parameter("escape_away_speed", 0.15)
        self.escape_away_speed = float(
            self.get_parameter("escape_away_speed").value
        )

        self.control_rate = float(
            self.get_parameter("control_rate").value
        )
        self.scan_timeout = float(
            self.get_parameter("scan_timeout").value
        )
        self.cmd_timeout = float(
            self.get_parameter("cmd_timeout").value
        )
        self.robot_front_extent = float(
            self.get_parameter("robot_front_extent").value
        )
        self.front_half_width = float(
            self.get_parameter("front_half_width").value
        )
        self.robot_rear_extent = float(
            self.get_parameter("robot_rear_extent").value
        )
        self.rear_half_width = float(
            self.get_parameter("rear_half_width").value
        )
        self.robot_safety_radius = float(
            self.get_parameter("robot_safety_radius").value
        )
        self.turn_slow_distance = float(
            self.get_parameter("turn_slow_distance").value
        )
        self.turn_stop_distance = float(
            self.get_parameter("turn_stop_distance").value
        )
        self.slow_distance = float(
            self.get_parameter("slow_distance").value
        )
        self.stop_distance = float(
            self.get_parameter("stop_distance").value
        )
        self.emergency_distance = float(
            self.get_parameter("emergency_distance").value
        )
        self.ttc_slow = float(
            self.get_parameter("ttc_slow").value
        )
        self.ttc_stop = float(
            self.get_parameter("ttc_stop").value
        )
        self.closing_speed_alpha = float(
            self.get_parameter("closing_speed_alpha").value
        )
        self.max_closing_speed = float(
            self.get_parameter("max_closing_speed").value
        )
        self.release_distance = float(
            self.get_parameter("release_distance").value
        )
        self.release_ttc = float(
            self.get_parameter("release_ttc").value
        )
        self.release_hold_time = float(
            self.get_parameter("release_hold_time").value
        )

        if self.release_distance <= self.stop_distance:
            raise ValueError(
                "release_distance must be greater than stop_distance"
            )

        if self.release_ttc <= self.ttc_stop:
            raise ValueError(
                "release_ttc must be greater than ttc_stop"
            )

        if not 0.0 < self.ttc_stop < self.ttc_slow:
            raise ValueError("Require 0 < ttc_stop < ttc_slow")

        if not 0.0 <= self.closing_speed_alpha <= 1.0:
            raise ValueError("closing_speed_alpha must be in [0, 1]")

        if not (
            0.0 <= self.emergency_distance
            <= self.stop_distance
            < self.slow_distance
        ):
            raise ValueError(
                "Require emergency_distance <= stop_distance "
                "< slow_distance"
            )

        if not 0.0 <= self.turn_stop_distance < self.turn_slow_distance:
            raise ValueError(
                "Require 0 <= turn_stop_distance < turn_slow_distance"
            )

        self.latest_raw_cmd = Twist()
        self.last_cmd_ns = None
        self.last_scan_ns = None
        self.front_clearance = float("inf")
        self.rear_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.previous_clearance = None
        self.previous_scan_ns = None
        self.closing_speed = 0.0
        self.stop_latched = False
        self.release_safe_since_ns = None
        self._escape_direction_sign = 0.0
        self._escape_committed_since_ns = None
        self._last_output_sign = 0.0
        self._sign_committed_since_ns = None
        self._last_linear_output_sign = 0.0
        self._linear_sign_committed_since_ns = None
        # Direction (+1 front, -1 rear, 0 unknown/freshness-triggered)
        # requested at the moment stop_latched was freshly set. The
        # release check below must stay anchored to this, not to whatever
        # direction is currently requested -- otherwise an upstream
        # sign-flip (e.g. DWA alternating forward/reverse) can satisfy the
        # release condition for a hazard it never actually cleared.
        self._latch_direction_sign = 0.0

        self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.raw_cmd_cb,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            qos_profile_sensor_data,
        )

        self.safe_cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        period = 1.0 / max(self.control_rate, 1e-3)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            "LiDAR safety node started"
        )

    def raw_cmd_cb(self, msg: Twist):
        """Store the latest requested velocity command."""
        self.latest_raw_cmd = msg
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def scan_cb(self, msg: LaserScan):
        """Measure the nearest obstacle in front, behind, and on each side."""
        minimum_clearance = float("inf")
        rear_clearance_min = float("inf")
        left_clearance = float("inf")
        right_clearance = float("inf")

        for index, measurement in enumerate(msg.ranges):
            if not math.isfinite(measurement):
                continue
            if measurement < msg.range_min:
                continue
            if measurement > msg.range_max:
                continue

            angle = (
                float(msg.angle_min)
                + index * float(msg.angle_increment)
            )
            point_x = float(measurement) * math.cos(angle)
            point_y = float(measurement) * math.sin(angle)

            if point_x > 0.0 and abs(point_y) <= self.front_half_width:
                clearance = point_x - self.robot_front_extent
                minimum_clearance = min(
                    minimum_clearance,
                    clearance,
                )

            if point_x < 0.0 and abs(point_y) <= self.rear_half_width:
                rear_clearance = -point_x - self.robot_rear_extent
                rear_clearance_min = min(
                    rear_clearance_min,
                    rear_clearance,
                )

            side_clearance = float(measurement) - self.robot_safety_radius
            if point_y > 0.0:
                left_clearance = min(left_clearance, side_clearance)
            elif point_y < 0.0:
                right_clearance = min(right_clearance, side_clearance)

        now_ns = self.get_clock().now().nanoseconds
        current_clearance = max(0.0, minimum_clearance)
        current_rear_clearance = (
            max(0.0, rear_clearance_min)
            if math.isfinite(rear_clearance_min)
            else float("inf")
        )
        self.left_clearance = (
            max(0.0, left_clearance)
            if math.isfinite(left_clearance)
            else float("inf")
        )
        self.right_clearance = (
            max(0.0, right_clearance)
            if math.isfinite(right_clearance)
            else float("inf")
        )

        valid_history = (
            self.previous_clearance is not None
            and self.previous_scan_ns is not None
            and math.isfinite(current_clearance)
        )

        if valid_history:
            dt = (now_ns - self.previous_scan_ns) / 1e9

            if 0.01 < dt < 0.5:
                raw_closing_speed = (
                    self.previous_clearance - current_clearance
                ) / dt
                raw_closing_speed = max(
                    0.0,
                    min(self.max_closing_speed, raw_closing_speed),
                )

                alpha = self.closing_speed_alpha
                self.closing_speed = (
                    alpha * raw_closing_speed
                    + (1.0 - alpha) * self.closing_speed
                )
            else:
                self.closing_speed = 0.0
        else:
            self.closing_speed = 0.0

        self.front_clearance = current_clearance
        self.rear_clearance = current_rear_clearance
        self.previous_clearance = (
            current_clearance
            if math.isfinite(current_clearance)
            else None
        )
        self.previous_scan_ns = now_ns
        self.last_scan_ns = now_ns

    def _gate_angular(self, angular_z: float) -> float:
        """Allow rotation when the side being turned toward remains clear."""
        if abs(angular_z) <= 1e-6:
            return 0.0

        side_clearance = min(
            self.left_clearance,
            self.right_clearance,
        )
        span = self.turn_slow_distance - self.turn_stop_distance
        side_scale = (
            side_clearance - self.turn_stop_distance
        ) / span
        side_scale = max(0.0, min(1.0, side_scale))
        return angular_z * side_scale

    def _hold_sign(self, angular_z: float, now_ns: int) -> float:
        """Suppress a turn-direction reversal for release_hold_time.

        Dampens an upstream command that itself oscillates in sign every
        cycle (e.g. DWA/global-path chatter) -- gate_angular alone has no
        memory and would relay any sign flip untouched.
        """
        if abs(angular_z) <= 1e-6:
            self._last_output_sign = 0.0
            self._sign_committed_since_ns = None
            return angular_z

        sign = 1.0 if angular_z > 0.0 else -1.0

        if (
            self._last_output_sign == 0.0
            or self._sign_committed_since_ns is None
            or sign == self._last_output_sign
        ):
            self._last_output_sign = sign
            self._sign_committed_since_ns = now_ns
            return angular_z

        held_seconds = (
            now_ns - self._sign_committed_since_ns
        ) / 1e9
        if held_seconds < self.release_hold_time:
            return 0.0

        self._last_output_sign = sign
        self._sign_committed_since_ns = now_ns
        return angular_z

    def _finalize_angular(self, requested_turn: float, now_ns: int) -> float:
        return self._hold_sign(
            self._gate_angular(requested_turn), now_ns
        )

    def _hold_linear_sign(self, linear_x: float, now_ns: int) -> float:
        """Suppress a forward/reverse reversal for release_hold_time.

        Same last-resort damper as _hold_sign, mirrored onto linear.x: an
        upstream command alternating forward/reverse every cycle (e.g. a
        DWA emergency-branch tie with no continuity term) would otherwise
        be relayed untouched, since scaling alone has no memory.
        """
        if abs(linear_x) <= 1e-6:
            self._last_linear_output_sign = 0.0
            self._linear_sign_committed_since_ns = None
            return linear_x

        sign = 1.0 if linear_x > 0.0 else -1.0

        if (
            self._last_linear_output_sign == 0.0
            or self._linear_sign_committed_since_ns is None
            or sign == self._last_linear_output_sign
        ):
            self._last_linear_output_sign = sign
            self._linear_sign_committed_since_ns = now_ns
            return linear_x

        held_seconds = (
            now_ns - self._linear_sign_committed_since_ns
        ) / 1e9
        if held_seconds < self.release_hold_time:
            return 0.0

        self._last_linear_output_sign = sign
        self._linear_sign_committed_since_ns = now_ns
        return linear_x

    def control_loop(self):
        """Apply freshness and forward-clearance safety limits."""
        now_ns = self.get_clock().now().nanoseconds

        cmd_missing = self.last_cmd_ns is None
        scan_missing = self.last_scan_ns is None

        cmd_stale = (
            not cmd_missing
            and (now_ns - self.last_cmd_ns) / 1e9
            > self.cmd_timeout
        )
        scan_stale = (
            not scan_missing
            and (now_ns - self.last_scan_ns) / 1e9
            > self.scan_timeout
        )

        if cmd_missing or scan_missing or cmd_stale or scan_stale:
            if not self.stop_latched:
                self._latch_direction_sign = 0.0
            self.stop_latched = True
            self.release_safe_since_ns = None
            self.safe_cmd_pub.publish(Twist())
            return

        raw_cmd = self.latest_raw_cmd
        safe_cmd = Twist()
        safe_cmd.linear.x = float(raw_cmd.linear.x)
        safe_cmd.angular.z = float(raw_cmd.angular.z)

        if safe_cmd.linear.x > 0.0:
            clearance = self.front_clearance
        elif safe_cmd.linear.x < 0.0:
            clearance = self.rear_clearance
        else:
            clearance = float("inf")

        ttc = float("inf")
        if (
            safe_cmd.linear.x > 0.0
            and self.closing_speed > 1e-3
            and math.isfinite(clearance)
        ):
            ttc = clearance / self.closing_speed

        # No hard stop for side obstacles here, on purpose: when a person comes close to
        # one side, the robot must keep moving -- stopping abruptly in front of an
        # approaching person causes the collision instead of avoiding it. The robot
        # only stops for obstacles inside its forward/backward corridor.
        # emergency_distance is still declared and validated but not used, so existing
        # YAML files keep working. The C++ implementation (social_nav_safety_cpp) has a
        # test that locks this behaviour.

        stop_required = (
            clearance <= self.stop_distance
            or ttc <= self.ttc_stop
        )
        if stop_required:
            if not self.stop_latched:
                self._latch_direction_sign = (
                    1.0 if safe_cmd.linear.x > 0.0
                    else -1.0 if safe_cmd.linear.x < 0.0
                    else 0.0
                )
            self.stop_latched = True
            self.release_safe_since_ns = None

            blocked = Twist()
            requested_turn = safe_cmd.angular.z

            # The planner still asks to drive straight although lidar blocks it. Rotate in
            # place towards the side with more free space, keeping the chosen direction for
            # release_hold_time to avoid flipping on sensor noise.
            if (
                safe_cmd.linear.x > 0.0
                and abs(requested_turn) <= 1e-6
            ):
                commit_expired = (
                    self._escape_committed_since_ns is None
                    or (now_ns - self._escape_committed_since_ns) / 1e9
                    >= self.release_hold_time
                )
                if self._escape_direction_sign == 0.0 or commit_expired:
                    self._escape_direction_sign = (
                        1.0
                        if self.left_clearance > self.right_clearance
                        else -1.0
                    )
                    self._escape_committed_since_ns = now_ns
                requested_turn = (
                    self._escape_direction_sign * self.escape_angular_speed
                )
            else:
                self._escape_direction_sign = 0.0
                self._escape_committed_since_ns = None

            blocked.angular.z = self._finalize_angular(
                requested_turn, now_ns
            )
            self.safe_cmd_pub.publish(blocked)
            return

        if self.stop_latched:
            # ESCAPE. Latched because one direction is blocked, but upstream now asks to
            # move the OTHER way. Moving away from the hazard is never worse than standing
            # still. The opposite corridor must still be genuinely clear, and the latch is
            # kept, so the robot cannot re-enter the blocked direction.
            if (
                self.escape_away_enabled
                and self._latch_direction_sign != 0.0
                and abs(safe_cmd.linear.x) > 1e-6
            ):
                request_sign = 1.0 if safe_cmd.linear.x > 0.0 else -1.0
                away_clearance = (
                    self.front_clearance
                    if request_sign > 0.0
                    else self.rear_clearance
                )
                if (
                    request_sign == -self._latch_direction_sign
                    and away_clearance >= self.release_distance
                ):
                    escape = Twist()
                    escape.linear.x = max(
                        -self.escape_away_speed,
                        min(self.escape_away_speed, safe_cmd.linear.x),
                    )
                    escape.angular.z = self._finalize_angular(
                        safe_cmd.angular.z, now_ns
                    )
                    # Record the emitted sign so _hold_linear_sign blocks
                    # further flapping; bypassed here because this IS the
                    # intentional reversal.
                    self._last_linear_output_sign = request_sign
                    self._linear_sign_committed_since_ns = now_ns
                    self.release_safe_since_ns = None
                    self.safe_cmd_pub.publish(escape)
                    return

            # Anchor the release check to the direction that actually triggered the latch,
            # not to whatever raw_cmd.linear.x currently requests -- otherwise an upstream
            # sign flip (e.g. alternating forward/reverse) could release a latch whose
            # hazard was never cleared (the rear looks clear while the front hazard that
            # caused the stop is still there).
            if self._latch_direction_sign > 0.0:
                release_clearance = self.front_clearance
                release_ttc = (
                    release_clearance / self.closing_speed
                    if (
                        self.closing_speed > 1e-3
                        and math.isfinite(release_clearance)
                    )
                    else float("inf")
                )
            elif self._latch_direction_sign < 0.0:
                release_clearance = self.rear_clearance
                release_ttc = float("inf")
            else:
                release_clearance = clearance
                release_ttc = ttc

            release_safe = (
                release_clearance >= self.release_distance
                and release_ttc >= self.release_ttc
            )

            if not release_safe:
                self.release_safe_since_ns = None
                blocked = Twist()
                blocked.angular.z = self._finalize_angular(
                    safe_cmd.angular.z, now_ns
                )
                self.safe_cmd_pub.publish(blocked)
                return

            if self.release_safe_since_ns is None:
                self.release_safe_since_ns = now_ns
                blocked = Twist()
                blocked.angular.z = self._finalize_angular(
                    safe_cmd.angular.z, now_ns
                )
                self.safe_cmd_pub.publish(blocked)
                return

            safe_duration = (
                now_ns - self.release_safe_since_ns
            ) / 1e9

            if safe_duration < self.release_hold_time:
                blocked = Twist()
                blocked.angular.z = self._finalize_angular(
                    safe_cmd.angular.z, now_ns
                )
                self.safe_cmd_pub.publish(blocked)
                return

            self.stop_latched = False
            self.release_safe_since_ns = None
            self._latch_direction_sign = 0.0

        self._escape_direction_sign = 0.0
        self._escape_committed_since_ns = None

        scale = 1.0

        if math.isfinite(clearance) and clearance < self.slow_distance:
            distance_scale = (
                (clearance - self.stop_distance)
                / (self.slow_distance - self.stop_distance)
            )
            scale = min(
                scale,
                max(0.0, min(1.0, distance_scale)),
            )

        if ttc < self.ttc_slow:
            ttc_scale = (
                (ttc - self.ttc_stop)
                / (self.ttc_slow - self.ttc_stop)
            )
            scale = min(
                scale,
                max(0.0, min(1.0, ttc_scale)),
            )

        safe_cmd.linear.x = self._hold_linear_sign(
            safe_cmd.linear.x * scale, now_ns
        )
        safe_cmd.angular.z = self._finalize_angular(
            safe_cmd.angular.z, now_ns
        )
        self.safe_cmd_pub.publish(safe_cmd)


def main(args=None):
    """Run the LiDAR safety node."""
    rclpy.init(args=args)
    node = LidarSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
