#!/usr/bin/env python3
# Social-navigation metrics in Gazebo, computed from GROUND-TRUTH positions.
#   ros2 run social_nav_gazebo social_nav_metrics --ros-args -p use_sim_time:=true
#
# Three groups:
#   1. COLLISIONS: clearance between the robot body rectangle and each person's
#      shape (not centre-to-centre distance). A new collision is only counted
#      after the clearance rises above collision_release, so one contact is not
#      counted many times. Also minimum clearance and time inside the social zone.
#   2. ROBOT TIME: time actually moving, distance, and each Nav2 goal
#      (/navigate_to_pose).
#   3. PEOPLE SEEN: every person published by the tracker (/planning/tracked_humans,
#      odom frame) is matched with the nearest REAL person within match_radius,
#      split into "any sensor" and "camera"; unmatched tracks count as false ids.
#
# Ground truth comes from the GroundTruthPosePublisher1 plugin (dymap.sdf) via
# /social_nav/ground_truth; all times are sim time from its stamps.
#
# Ctrl+C prints a summary and writes JSON to output_dir.
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from tf2_msgs.msg import TFMessage
from thesis_msgs.msg import TrackedHuman, TrackedHumans

GOAL_RESULT = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


# ---------------------------------------------------------------- 2D geometry
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_world(x, y, yaw, px, py):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def box_polygon(x, y, yaw, box):
    xmin, xmax, ymin, ymax = box
    return [to_world(x, y, yaw, px, py)
            for px, py in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))]


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else max(
        0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def edges(poly):
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def inside_convex(p, poly):
    sign = 0
    for a, b in edges(poly):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if cross != 0.0:
            if sign == 0:
                sign = 1 if cross > 0 else -1
            elif (cross > 0) != (sign > 0):
                return False
    return True


def segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return orient(a, b, c) * orient(a, b, d) < 0 and orient(c, d, a) * orient(c, d, b) < 0


def clearance_polygon_circle(poly, center, radius):
    """Clearance (m). Negative = overlapping."""
    d = min(point_segment_distance(center, a, b) for a, b in edges(poly))
    return -(d + radius) if inside_convex(center, poly) else d - radius


def clearance_polygon_polygon(p, q):
    """Clearance (m). 0 = touching or overlapping (penetration depth not computed)."""
    if any(inside_convex(v, q) for v in p) or any(inside_convex(v, p) for v in q):
        return 0.0
    if any(segments_cross(a, b, c, d) for a, b in edges(p) for c, d in edges(q)):
        return 0.0
    return min(
        min(point_segment_distance(v, a, b) for v in p for a, b in edges(q)),
        min(point_segment_distance(v, a, b) for v in q for a, b in edges(p)))


def parse_shape(spec):
    """'name=circle:r' or 'name=box:xmin,xmax,ymin,ymax' (model frame)."""
    name, shape = spec.split('=', 1)
    kind, args = shape.split(':', 1)
    values = [float(v) for v in args.split(',')]
    if kind == 'circle' and len(values) == 1:
        return name.strip(), ('circle', values[0])
    if kind == 'box' and len(values) == 4:
        return name.strip(), ('box', tuple(values))
    raise ValueError(f'Invalid person shape: {spec!r}')


# ---------------------------------------------------------------------- node
class Person:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape
        self.pose = None            # (x, y, yaw) in world
        self.clearance = math.inf
        self.min_clearance = math.inf
        self.in_collision = False
        self.collisions = 0
        self.social_zone_time = 0.0
        self.seen_time = 0.0        # seen by the tracker (any source)
        self.camera_time = 0.0      # seen by the tracker AND from the camera
        self.first_seen = None
        self.first_camera = None
        self.seen_now = False
        self.camera_now = False

    def center(self):
        x, y, yaw = self.pose
        if self.shape[0] == 'circle':
            return x, y
        xmin, xmax, ymin, ymax = self.shape[1]
        return to_world(x, y, yaw, (xmin + xmax) / 2.0, (ymin + ymax) / 2.0)


class SocialNavMetrics(Node):
    def __init__(self):
        super().__init__('social_nav_metrics')
        declare = self.declare_parameter
        self.robot_name = declare('robot_model', 'my_amr').value
        # my_amr base_link collision box (my_amr_properties.urdf.xacro), centred on base_footprint.
        length = declare('robot_length', 0.27).value
        width = declare('robot_width', 0.24).value
        self.robot_box = (-length / 2.0, length / 2.0, -width / 2.0, width / 2.0)
        # actor_collision_proxy: cylinder r=0.28 (dymap.sdf). Nurse: bounding box of
        # Nurse_Col.obj in the model frame -- OFFSET from the model origin.
        people_specs = declare('people', [
            'actor_collision_proxy=circle:0.28',
            'Nurse=box:-0.111,0.511,-0.246,0.200',
            'Nurse_clone=box:-0.111,0.511,-0.246,0.200',
        ]).value
        self.social_zone = declare('social_zone', 0.5).value
        self.collision_release = declare('collision_release', 0.05).value
        self.match_radius = declare('match_radius', 0.7).value
        self.max_observation_age = declare('max_observation_age', 1.0).value
        self.status_period = declare('status_period', 5.0).value
        self.moving_speed = declare('moving_speed_threshold', 0.02).value
        self.moving_yaw_rate = declare('moving_yaw_rate_threshold', 0.1).value
        # A step faster than this is a JUMP (set_pose / world reset), not driving:
        # excluded from distance and time.
        self.max_plausible_speed = declare('max_plausible_speed', 2.0).value
        self.teleports = 0
        self.output_dir = os.path.expanduser(
            declare('output_dir', '~/.ros/social_nav_metrics').value)
        self.odom_frame = declare('odom_frame', 'odom').value
        self.base_frame = declare('base_frame', 'base_footprint').value

        self.people = [Person(*parse_shape(s)) for s in people_specs]
        self.by_name = {p.name: p for p in self.people}

        self.robot_pose = None
        self.start_t = None
        self.last_t = None
        self.last_status_t = None
        self.distance = 0.0
        self.moving_time = 0.0
        self.min_clearance = math.inf
        self.min_clearance_who = '-'

        self.goals = {}             # goal id -> running goal dict
        self.finished_goals = []

        self.last_tracks_t = None
        self.track_ids = set()
        self.false_track_ids = set()
        self.tracks_received = 0
        self.tf_errors = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(TFMessage, '/social_nav/ground_truth', self.on_ground_truth, 10)
        self.create_subscription(TrackedHumans, '/planning/tracked_humans', self.on_tracks, 10)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_goal_status, status_qos)

        self.get_logger().info(
            f'Measuring: robot={self.robot_name}, people={[p.name for p in self.people]}, '
            f'social zone<{self.social_zone} m. Ctrl+C prints the summary.')

    # ------------------------------------------------------- ground truth
    def on_ground_truth(self, msg):
        poses = {}
        stamp = None
        for tf in msg.transforms:
            tr, q = tf.transform.translation, tf.transform.rotation
            poses[tf.child_frame_id] = (tr.x, tr.y, yaw_of(q))
            stamp = tf.header.stamp
        if self.robot_name not in poses or stamp is None:
            return
        t = stamp.sec + stamp.nanosec * 1e-9

        # World reset (sim time went backwards) -> start counting again.
        if self.last_t is not None and t < self.last_t:
            self.get_logger().warn('Sim time went backwards (world reset) -- restarting the clock.')
            self.last_t = None
        if self.start_t is None:
            self.start_t = t
            self.last_status_t = t

        new_pose = poses[self.robot_name]
        if self.last_t is not None and self.robot_pose is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:
                step = math.hypot(new_pose[0] - self.robot_pose[0], new_pose[1] - self.robot_pose[1])
                dyaw = math.atan2(math.sin(new_pose[2] - self.robot_pose[2]),
                                  math.cos(new_pose[2] - self.robot_pose[2]))
                if step / dt > self.max_plausible_speed:
                    self.teleports += 1
                    self.get_logger().warn(
                        f'Robot JUMPED {step:.2f} m in {dt * 1000:.0f} ms (set_pose/reset?) -- '
                        'not counted as distance')
                elif step / dt > self.moving_speed or abs(dyaw) / dt > self.moving_yaw_rate:
                    self.moving_time += dt
                    self.distance += step
                for person in self.people:
                    if person.pose is not None and person.clearance < self.social_zone:
                        person.social_zone_time += dt
        self.robot_pose = new_pose
        self.last_t = t

        robot_poly = box_polygon(*new_pose, self.robot_box)
        for person in self.people:
            if person.name not in poses:
                continue
            person.pose = poses[person.name]
            if person.shape[0] == 'circle':
                c = clearance_polygon_circle(robot_poly, person.pose[:2], person.shape[1])
            else:
                c = clearance_polygon_polygon(robot_poly, box_polygon(*person.pose, person.shape[1]))
            person.clearance = c
            person.min_clearance = min(person.min_clearance, c)
            if c < self.min_clearance:
                self.min_clearance, self.min_clearance_who = c, person.name
            self.update_collision(person, t)

        if t - self.last_status_t >= self.status_period:
            self.last_status_t = t
            self.print_status(t)

    def update_collision(self, person, t):
        if not person.in_collision and person.clearance <= 0.0:
            person.in_collision = True
            person.collisions += 1
            self.get_logger().error(
                f'[COLLISION #{self.total_collisions()}] with {person.name} at t={self.rel(t):.1f}s, '
                f'robot at ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f}), '
                f'clearance {person.clearance:+.2f} m')
        elif person.in_collision and person.clearance > self.collision_release:
            person.in_collision = False
            self.get_logger().warn(f'Collision with {person.name} ended at t={self.rel(t):.1f}s')

    def rel(self, t):
        """All printed times are relative to the first ground-truth message."""
        return t - self.start_t if self.start_t is not None else t

    def total_collisions(self):
        return sum(p.collisions for p in self.people)

    # ------------------------------------------------------ people seen
    def on_tracks(self, msg):
        if self.robot_pose is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            self.tf_errors += 1
            return
        # world_T_odom = world_T_base(true) * inv(odom_T_base). Independent of map/AMCL.
        ox, oy = tf.transform.translation.x, tf.transform.translation.y
        oyaw = yaw_of(tf.transform.rotation)
        rx, ry, ryaw = self.robot_pose
        wyaw = ryaw - oyaw
        c, s = math.cos(wyaw), math.sin(wyaw)
        wx, wy = rx - (c * ox - s * oy), ry - (s * ox + c * oy)

        dt = 0.0
        if self.last_tracks_t is not None and 0.0 < t - self.last_tracks_t < 1.0:
            dt = t - self.last_tracks_t
        self.last_tracks_t = t
        self.tracks_received += 1

        for person in self.people:
            person.seen_now = person.camera_now = False
        for human in msg.humans:
            self.track_ids.add(human.id)
            if human.observation_age > self.max_observation_age:
                continue
            hx, hy = wx + c * human.x - s * human.y, wy + s * human.x + c * human.y
            best, best_d = None, self.match_radius
            for person in self.people:
                if person.pose is None:
                    continue
                px, py = person.center()
                d = math.hypot(hx - px, hy - py)
                if d < best_d:
                    best, best_d = person, d
            if best is None:
                self.false_track_ids.add(human.id)
                continue
            if not best.seen_now:
                best.seen_now = True
                best.seen_time += dt
                if best.first_seen is None:
                    best.first_seen = t
                    self.get_logger().info(f'First seen {best.name} at t={self.rel(t):.1f}s (track id {human.id})')
            if human.source == TrackedHuman.SOURCE_CAMERA and not best.camera_now:
                best.camera_now = True
                best.camera_time += dt
                if best.first_camera is None:
                    best.first_camera = t

    # ---------------------------------------------------------------- goals
    def on_goal_status(self, msg):
        now = self.last_t
        if now is None:
            return
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid).hex()[:8]
            if status.status == GoalStatus.STATUS_EXECUTING and gid not in self.goals \
                    and all(g['id'] != gid for g in self.finished_goals):
                self.goals[gid] = {
                    'id': gid, 'start': now, 'distance0': self.distance,
                    'moving0': self.moving_time, 'collisions0': self.total_collisions(),
                }
                self.get_logger().info(f'Goal {gid} started at t={self.rel(now):.1f}s')
            elif status.status in GOAL_RESULT and gid in self.goals:
                g = self.goals.pop(gid)
                g.update({
                    'result': GOAL_RESULT[status.status],
                    'duration_s': round(now - g['start'], 2),
                    'moving_s': round(self.moving_time - g.pop('moving0'), 2),
                    'distance_m': round(self.distance - g.pop('distance0'), 2),
                    'collisions': self.total_collisions() - g.pop('collisions0'),
                })
                self.finished_goals.append(g)
                self.get_logger().info(
                    f"Goal {gid} {g['result']}: {g['duration_s']:.1f}s, {g['distance_m']:.2f} m, "
                    f"collisions {g['collisions']}")

    # ------------------------------------------------------------- report
    def print_status(self, t):
        seen_now = sum(p.seen_now for p in self.people)
        camera_now = sum(p.camera_now for p in self.people)
        seen_ever = sum(p.first_seen is not None for p in self.people)
        nearest = min((p for p in self.people if p.pose is not None),
                      key=lambda p: p.clearance, default=None)
        near_txt = f'{nearest.name} {nearest.clearance:.2f} m' if nearest else '-'
        self.get_logger().info(
            f'[t={self.rel(t):6.0f}s] collisions={self.total_collisions()} | nearest: {near_txt} | '
            f'seen now {seen_now}/{len(self.people)} (camera {camera_now}) | '
            f'seen ever {seen_ever}/{len(self.people)} | robot {self.distance:.1f} m, moving {self.moving_time:.0f}s')

    def summary(self):
        elapsed = (self.last_t - self.start_t) if self.start_t is not None else 0.0
        return {
            'sim_duration_s': round(elapsed, 2),
            'collisions_total': self.total_collisions(),
            'min_clearance_m': None if math.isinf(self.min_clearance) else round(self.min_clearance, 3),
            'min_clearance_with': self.min_clearance_who,
            'robot_moving_s': round(self.moving_time, 2),
            'robot_distance_m': round(self.distance, 2),
            'robot_teleports_ignored': self.teleports,
            'people_seen': sum(p.first_seen is not None for p in self.people),
            'people_seen_by_camera': sum(p.first_camera is not None for p in self.people),
            'people_total': len(self.people),
            'tracker_unique_ids': len(self.track_ids),
            'tracker_false_ids': len(self.false_track_ids),
            'tracker_messages': self.tracks_received,
            'goals': self.finished_goals,
            'goals_unfinished': len(self.goals),
            'people': [{
                'name': p.name,
                'collisions': p.collisions,
                'min_clearance_m': None if math.isinf(p.min_clearance) else round(p.min_clearance, 3),
                'social_zone_s': round(p.social_zone_time, 2),
                'seen': p.first_seen is not None,
                'seen_s': round(p.seen_time, 2),
                'seen_by_camera': p.first_camera is not None,
                'camera_s': round(p.camera_time, 2),
            } for p in self.people],
        }

    def finish(self):
        s = self.summary()
        lines = [
            '',
            '================ SOCIAL NAV SUMMARY =================',
            f"Simulated time      : {s['sim_duration_s']:.1f} s",
            f"Collisions          : {s['collisions_total']}",
            f"Minimum clearance   : {s['min_clearance_m']} m (with {s['min_clearance_with']})",
            f"Robot moving        : {s['robot_moving_s']:.1f} s, {s['robot_distance_m']:.2f} m",
            f"People seen         : {s['people_seen']}/{s['people_total']} "
            f"(camera: {s['people_seen_by_camera']}/{s['people_total']})",
            f"Tracker tracks      : {s['tracker_unique_ids']} ids, {s['tracker_false_ids']} ids not matching a real person",
            '--- Per person ---',
        ]
        for p in s['people']:
            lines.append(
                f"  {p['name']:<22} collisions {p['collisions']}, min clearance {p['min_clearance_m']} m, "
                f"social zone {p['social_zone_s']:.1f}s, seen {p['seen_s']:.1f}s (camera {p['camera_s']:.1f}s)")
        lines.append(f"--- Nav2 goals ({len(s['goals'])} finished, {s['goals_unfinished']} running) ---")
        for g in s['goals']:
            lines.append(
                f"  {g['id']} {g['result']:<8} {g['duration_s']:.1f}s (moving {g['moving_s']:.1f}s), "
                f"{g['distance_m']:.2f} m, collisions {g['collisions']}")
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            path = os.path.join(self.output_dir, time.strftime('run_%Y%m%d_%H%M%S.json'))
            with open(path, 'w') as f:
                json.dump(s, f, indent=2, ensure_ascii=False)
            lines.append(f'Written: {path}')
        except OSError as exc:
            lines.append(f'Cannot write the result file: {exc}')
        lines.append('=====================================================')
        # print instead of the logger: after Ctrl+C the rclpy context may be gone.
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = SocialNavMetrics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
