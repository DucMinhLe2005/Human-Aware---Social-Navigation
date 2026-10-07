#!/usr/bin/env python3
"""Thin node: takes human detections (camera) and scans (lidar), runs HumanTracker
and publishes thesis_msgs/TrackedHumans.

Tracks are kept and published in the "odom" frame:
  1. The local costmap (where the AGHPM layer reads TrackedHumans) uses
     global_frame=odom, so no map->odom lookup per track is needed every cycle.
  2. With AMCL running, map and odom really differ; the controller and the
     local costmap both work in odom.
  3. The person detector (yolo_pose_node1.py) already publishes in odom, so
     camera detections normally need no transform (only a fallback if
     frame_id differs).
scan_cb transforms lidar to odom for clustering. Only the static-map filter
needs the map frame, and it transforms just the few cluster centres
(person_centers_odom) to map, filters them against the static grid and maps the
result back to the original odom coordinates.
"""
from threading import Lock

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from thesis_msgs.msg import MultiplePoseResult3D, PoseResult3D, TrackedHuman, TrackedHumans
from visualization_msgs.msg import Marker, MarkerArray

from social_nav_tracking.human_tracking import (
    HumanTracker,
    HumanTrackerParams,
    cluster_lidar_person_centers,
    filter_lidar_person_centers_by_exclusion_zones,
    filter_lidar_person_centers_by_static_map,
)


def _quaternion_to_yaw(quaternion) -> float:
    return float(
        np.arctan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
    )


def _normalize_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))



def _pose_to_xyyaw(pose_result: PoseResult3D):
    """Extract a human center and body yaw from pose keypoints."""
    if not pose_result.keypoints:
        return None

    keypoints = {
        keypoint.id: keypoint
        for keypoint in pose_result.keypoints
        if keypoint.confidence > 0.3
    }
    if not keypoints:
        return None

    torso_points = [
        (keypoints[keypoint_id].x, keypoints[keypoint_id].y)
        for keypoint_id in (5, 6, 11, 12)
        if keypoint_id in keypoints
    ]

    if torso_points:
        human_x = float(np.mean([point[0] for point in torso_points]))
        human_y = float(np.mean([point[1] for point in torso_points]))
    else:
        first_keypoint = next(iter(keypoints.values()))
        human_x = float(first_keypoint.x)
        human_y = float(first_keypoint.y)

    human_yaw = 0.0
    if 5 in keypoints and 6 in keypoints:
        left, right = keypoints[5], keypoints[6]
        human_yaw = np.arctan2(right.y - left.y, right.x - left.x) + np.pi / 2.0
    elif 11 in keypoints and 12 in keypoints:
        left, right = keypoints[11], keypoints[12]
        human_yaw = np.arctan2(right.y - left.y, right.x - left.x) + np.pi / 2.0
    elif 0 in keypoints and (11 in keypoints or 12 in keypoints):
        nose = keypoints[0]
        hip = keypoints.get(11) or keypoints[12]
        human_yaw = np.arctan2(nose.y - hip.y, nose.x - hip.x)

    return human_x, human_y, _normalize_angle(float(human_yaw))


class HumanTrackerNode1(Node):
    def __init__(self):
        super().__init__("human_tracker_node1")

        self.declare_parameter("update_rate", 4.0)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lidar_min_range", 0.15)
        self.declare_parameter("lidar_max_range", 8.0)
        self.declare_parameter("lidar_observation_timeout", 0.5)
        self.declare_parameter("publish_tracking_markers", True)
        self.declare_parameter("human_lidar_fusion_radius", 0.4)
        self.declare_parameter("human_lidar_sync_tolerance", 0.10)
        self.declare_parameter("lidar_person_cluster_min_points", 3)
        self.declare_parameter("lidar_person_cluster_max_gap", 0.15)
        self.declare_parameter("lidar_person_cluster_max_diameter", 0.75)
        self.declare_parameter("lidar_person_radius", 0.28)
        self.declare_parameter("lidar_static_rejection_radius", 0.15)
        # Do NOT publish unconfirmed lidar-only tracks (see _filter_publishable_tracks).
        # Set True to publish everything when debugging.
        self.declare_parameter("publish_unconfirmed_lidar_only", False)
        self.declare_parameter(
            "lidar_static_exclusion_zones", [0.0] * 0,
            ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter("track_association_gate", 0.75)
        self.declare_parameter("track_association_gate_speed_factor", 0.5)
        self.declare_parameter("track_min_hits_for_velocity", 3)
        self.declare_parameter("track_max_plausible_speed", 2.0)
        self.declare_parameter("track_moving_speed_threshold", 0.20)
        self.declare_parameter("track_moving_exit_speed_threshold", 0.10)
        self.declare_parameter("track_moving_hold_time", 0.5)
        self.declare_parameter("track_timeout", 0.35)
        self.declare_parameter("track_kf_process_noise_std", 1.0)
        # Reversal handling -- see HumanTrackerParams.reversal_* in human_tracking.py
        self.declare_parameter("track_reversal_speed_threshold", 0.25)
        self.declare_parameter("track_reversal_innovation_threshold", 0.10)
        self.declare_parameter("track_reversal_covariance_boost", 9.0)
        self.declare_parameter("track_kf_measurement_noise_std", 0.15)
        self.declare_parameter("track_kf_lidar_measurement_noise_std", 0.20)
        self.declare_parameter("track_min_hits_to_confirm", 3)
        self.declare_parameter("track_lidar_coast_gate", 0.5)
        self.declare_parameter("track_coast_match_timeout", 0.5)
        self.declare_parameter("track_coast_timeout", 3.0)
        self.declare_parameter("lidar_only_association_gate", 0.35)
        self.declare_parameter("lidar_only_min_hits", 4)
        self.declare_parameter("lidar_only_min_displacement", 0.12)
        self.declare_parameter("lidar_only_moving_speed_threshold", 0.15)
        # See HumanTrackerParams in human_tracking.py.
        #   lidar_only_enabled=False -> disable the lidar-only branch (camera only).
        #   lidar_only_confirm_decay_sec -> revoke the "person" label when a lidar-only
        #     track stays still longer than this and is not confirmed by the camera.
        #     0.0 keeps the label forever.
        self.declare_parameter("lidar_only_enabled", True)
        self.declare_parameter("lidar_only_confirm_decay_sec", 3.0)
        # Consecutive co-located camera+lidar ticks needed to confirm a person for good
        # (camera -> lidar handover). See fresh_camera_matches in human_tracking.py.
        # 0 disables it.
        self.declare_parameter("fusion_confirm_min_hits", 3)
        self.declare_parameter("lidar_only_track_timeout", 0.5)
        # Spatial person memory. See HumanTrackerParams in human_tracking.py.
        self.declare_parameter("person_memory_ttl", 5.0)
        self.declare_parameter("person_memory_radius_base", 0.5)
        self.declare_parameter("person_memory_radius_max", 2.5)
        self.declare_parameter("person_memory_growth_speed", 1.5)
        self.declare_parameter("ghost_publish_min_speed", 0.3)

        self.update_rate = float(self.get_parameter("update_rate").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.lidar_min_range = float(self.get_parameter("lidar_min_range").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)
        self.lidar_observation_timeout = float(
            self.get_parameter("lidar_observation_timeout").value
        )
        self.publish_tracking_markers = bool(
            self.get_parameter("publish_tracking_markers").value
        )
        self.human_lidar_fusion_radius = float(
            self.get_parameter("human_lidar_fusion_radius").value
        )
        self.human_lidar_sync_tolerance = float(
            self.get_parameter("human_lidar_sync_tolerance").value
        )
        self.lidar_person_cluster_min_points = int(
            self.get_parameter("lidar_person_cluster_min_points").value
        )
        self.lidar_person_cluster_max_gap = float(
            self.get_parameter("lidar_person_cluster_max_gap").value
        )
        self.lidar_person_cluster_max_diameter = float(
            self.get_parameter("lidar_person_cluster_max_diameter").value
        )
        self.lidar_person_radius = float(self.get_parameter("lidar_person_radius").value)
        self.lidar_static_rejection_radius = float(
            self.get_parameter("lidar_static_rejection_radius").value
        )
        self.publish_unconfirmed_lidar_only = bool(
            self.get_parameter("publish_unconfirmed_lidar_only").value
        )
        # `or []`: ROS 2 cannot infer the type of an EMPTY list in YAML (`key: []`), so
        # `.value` returns None and the loop below would raise TypeError at start-up.
        # To disable exclusion zones, remove the key from the YAML file.
        static_zone_values = [
            float(v)
            for v in (self.get_parameter("lidar_static_exclusion_zones").value or [])
        ]
        self.lidar_static_exclusion_zones = [
            tuple(static_zone_values[index:index + 3])
            for index in range(0, len(static_zone_values) - 2, 3)
        ]

        self.human_tracker = HumanTracker(
            HumanTrackerParams(
                association_gate=float(self.get_parameter("track_association_gate").value),
                association_gate_speed_factor=float(
                    self.get_parameter("track_association_gate_speed_factor").value
                ),
                min_hits_for_velocity=int(
                    self.get_parameter("track_min_hits_for_velocity").value
                ),
                max_plausible_speed=float(
                    self.get_parameter("track_max_plausible_speed").value
                ),
                moving_speed_threshold=float(
                    self.get_parameter("track_moving_speed_threshold").value
                ),
                moving_exit_speed_threshold=max(
                    0.0,
                    float(self.get_parameter("track_moving_exit_speed_threshold").value),
                ),
                moving_hold_time=max(
                    0.0, float(self.get_parameter("track_moving_hold_time").value)
                ),
                track_timeout=float(self.get_parameter("track_timeout").value),
                reversal_speed_threshold=float(
                    self.get_parameter("track_reversal_speed_threshold").value
                ),
                reversal_innovation_threshold=float(
                    self.get_parameter("track_reversal_innovation_threshold").value
                ),
                reversal_covariance_boost=float(
                    self.get_parameter("track_reversal_covariance_boost").value
                ),
                kf_process_noise_std=float(
                    self.get_parameter("track_kf_process_noise_std").value
                ),
                kf_measurement_noise_std=float(
                    self.get_parameter("track_kf_measurement_noise_std").value
                ),
                kf_lidar_measurement_noise_std=float(
                    self.get_parameter("track_kf_lidar_measurement_noise_std").value
                ),
                track_min_hits_to_confirm=int(
                    self.get_parameter("track_min_hits_to_confirm").value
                ),
                track_lidar_coast_gate=float(
                    self.get_parameter("track_lidar_coast_gate").value
                ),
                track_coast_match_timeout=float(
                    self.get_parameter("track_coast_match_timeout").value
                ),
                track_coast_timeout=float(self.get_parameter("track_coast_timeout").value),
                lidar_only_association_gate=float(
                    self.get_parameter("lidar_only_association_gate").value
                ),
                lidar_only_min_hits=int(self.get_parameter("lidar_only_min_hits").value),
                lidar_only_min_displacement=float(
                    self.get_parameter("lidar_only_min_displacement").value
                ),
                lidar_only_moving_speed_threshold=float(
                    self.get_parameter("lidar_only_moving_speed_threshold").value
                ),
                lidar_only_enabled=bool(
                    self.get_parameter("lidar_only_enabled").value
                ),
                lidar_only_confirm_decay_sec=float(
                    self.get_parameter("lidar_only_confirm_decay_sec").value
                ),
                lidar_only_track_timeout=float(
                    self.get_parameter("lidar_only_track_timeout").value
                ),
                fusion_confirm_min_hits=int(
                    self.get_parameter("fusion_confirm_min_hits").value
                ),
                person_memory_ttl=float(
                    self.get_parameter("person_memory_ttl").value
                ),
                person_memory_radius_base=float(
                    self.get_parameter("person_memory_radius_base").value
                ),
                person_memory_radius_max=float(
                    self.get_parameter("person_memory_radius_max").value
                ),
                person_memory_growth_speed=float(
                    self.get_parameter("person_memory_growth_speed").value
                ),
                ghost_publish_min_speed=float(
                    self.get_parameter("ghost_publish_min_speed").value
                ),
            )
        )

        self.map_msg = None
        self.static_grid_cache = None
        self.static_clearance_cache = None
        self.map_lock = Lock()

        self.lidar_points_odom = np.empty((0, 2), dtype=np.float64)
        self.lidar_person_centers_odom = np.empty((0, 2), dtype=np.float64)
        self.last_lidar_update_ns = 0
        self.lidar_generation = 0
        self.last_coasted_lidar_generation = -1
        self.last_lidar_stamp_ns = 0
        self.last_fused_lidar_generation = -1
        self.lidar_lock = Lock()
        self.tracker_lock = Lock()

        self._last_prune_log_ns = 0
        self._last_lidar_tf_warning_ns = 0
        self._last_human_tf_warning_ns = 0
        self._last_static_filter_tf_warning_ns = 0
        self._last_human_measurement_ns = 0
        self._last_clock_ns = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS must be TRANSIENT_LOCAL to match map_server: /map is published once at
        # start-up, so with the default VOLATILE profile the map never arrives when
        # this node starts later, and the static-obstacle filter silently does nothing.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, map_qos)
        self.create_subscription(
            MultiplePoseResult3D,
            "/perception/yolov8_multiple_pose_results_3d_ori",
            self.pose_cb,
            1,
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data
        )

        self.tracked_humans_pub = self.create_publisher(
            TrackedHumans, "/planning/tracked_humans", 10
        )
        self.tracked_humans_markers_pub = self.create_publisher(
            MarkerArray, "/planning/local_debug/tracked_humans", 10
        )
        self.lidar_person_centers_pub = self.create_publisher(
            PoseArray, "/planning/local_debug/lidar_person_centers", 10
        )

        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_cb)
        self.get_logger().info(
            f"human_tracker_node1 started: scan_topic={self.scan_topic!r}, "
            f"update_rate={self.update_rate:.1f} Hz, tracker frame=odom "
            "(see module docstring)"
        )

    # ---- map (static) -------------------------------------------------

    def map_cb(self, msg: OccupancyGrid):
        width = int(msg.info.width)
        height = int(msg.info.height)
        valid_map = (
            width > 0
            and height > 0
            and float(msg.info.resolution) > 0.0
            and len(msg.data) == width * height
        )
        if not valid_map:
            return

        static_grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        static_obstacles = static_grid >= self.occupied_threshold
        if np.any(static_obstacles):
            static_clearance = (
                distance_transform_edt(~static_obstacles) * float(msg.info.resolution)
            )
        else:
            static_clearance = np.full(static_grid.shape, np.inf, dtype=np.float64)

        with self.map_lock:
            self.map_msg = msg
            self.static_grid_cache = static_grid
            self.static_clearance_cache = static_clearance

    # ---- TF helpers -----------------------------------------------------

    def _lookup_2d_transform(self, target_frame: str, source_frame: str, stamp, timeout_sec: float):
        """Return (tx, ty, yaw) of source_frame in target_frame, or None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp) if hasattr(stamp, "sec") else stamp,
                timeout=Duration(seconds=timeout_sec),
            )
        except tf2_ros.TransformException:
            return None
        yaw = _quaternion_to_yaw(transform.transform.rotation)
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )

    def _transform_xyyaw(self, x, y, yaw, source_frame, target_frame, stamp):
        if not source_frame or source_frame == target_frame:
            return x, y, yaw
        looked_up = self._lookup_2d_transform(target_frame, source_frame, stamp, 0.05)
        if looked_up is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_human_tf_warning_ns > 5_000_000_000:
                self.get_logger().warn(
                    f"Cannot transform human pose {source_frame} -> {target_frame}"
                )
                self._last_human_tf_warning_ns = now_ns
            return None
        tx, ty, tyaw = looked_up
        cos_yaw, sin_yaw = np.cos(tyaw), np.sin(tyaw)
        out_x = tx + cos_yaw * x - sin_yaw * y
        out_y = ty + sin_yaw * x + cos_yaw * y
        out_yaw = _normalize_angle(yaw + tyaw)
        return out_x, out_y, out_yaw

    def _snap_to_nearest_lidar_point(self, x, y, points, excluded_indices):
        candidate_mask = np.ones(points.shape[0], dtype=bool)
        if excluded_indices:
            candidate_mask[list(excluded_indices)] = False
        if not np.any(candidate_mask):
            return x, y, None

        candidate_indices = np.nonzero(candidate_mask)[0]
        distances = np.hypot(
            points[candidate_indices, 0] - x, points[candidate_indices, 1] - y
        )
        best = int(np.argmin(distances))
        if distances[best] <= self.human_lidar_fusion_radius:
            matched_index = int(candidate_indices[best])
            return (
                float(points[matched_index, 0]),
                float(points[matched_index, 1]),
                matched_index,
            )
        return x, y, None

    def _snapshot_lidar(self, now_ns: int):
        with self.lidar_lock:
            points = self.lidar_points_odom
            person_centers = self.lidar_person_centers_odom
            last_update_ns = self.last_lidar_update_ns
            lidar_stamp_ns = self.last_lidar_stamp_ns
            lidar_generation = self.lidar_generation

        receipt_age = (
            (now_ns - last_update_ns) / 1e9 if last_update_ns > 0 else float("inf")
        )
        is_fresh = (
            last_update_ns > 0
            and 0.0 <= receipt_age <= self.lidar_observation_timeout
            and points.size > 0
        )
        return points, is_fresh, lidar_generation, lidar_stamp_ns, person_centers

    def _handle_clock_reset(self, now_ns: int) -> None:
        clock_moved_backwards = self._last_clock_ns > 0 and now_ns < self._last_clock_ns
        self._last_clock_ns = now_ns
        if not clock_moved_backwards:
            return
        with self.tracker_lock:
            self.human_tracker.reset()
        with self.lidar_lock:
            self.lidar_points_odom = np.empty((0, 2), dtype=np.float64)
            self.lidar_person_centers_odom = np.empty((0, 2), dtype=np.float64)
            self.last_lidar_update_ns = 0
            self.last_lidar_stamp_ns = 0
            self.lidar_generation += 1
        self.last_coasted_lidar_generation = -1
        self.last_fused_lidar_generation = -1
        self._last_human_measurement_ns = 0
        self.get_logger().warn("ROS clock moved backwards; cleared human tracks and LiDAR cache")

    # ---- camera detections ----------------------------------------------

    def pose_cb(self, msg: MultiplePoseResult3D):
        now_ns = self.get_clock().now().nanoseconds
        self._handle_clock_reset(now_ns)

        measurement_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if measurement_ns <= 0:
            measurement_ns = now_ns
        if measurement_ns <= self._last_human_measurement_ns:
            return
        self._last_human_measurement_ns = measurement_ns

        lidar_points, lidar_fresh, lidar_generation, lidar_stamp_ns, _ = self._snapshot_lidar(
            now_ns
        )
        lidar_time_error = (
            abs(lidar_stamp_ns - measurement_ns) / 1e9 if lidar_stamp_ns > 0 else float("inf")
        )
        can_fuse_lidar = (
            lidar_fresh
            and lidar_generation != self.last_fused_lidar_generation
            and lidar_time_error <= self.human_lidar_sync_tolerance
        )

        used_lidar_indices = set()
        humans = []
        for pose in msg.poses_result_3d:
            human = _pose_to_xyyaw(pose)
            if human is None:
                continue
            source_frame = pose.header.frame_id or msg.header.frame_id or "odom"
            human_odom = self._transform_xyyaw(
                human[0], human[1], human[2], source_frame, "odom", msg.header.stamp
            )
            if human_odom is None:
                continue
            human_x, human_y, human_yaw = human_odom
            if can_fuse_lidar:
                _, _, matched_index = self._snap_to_nearest_lidar_point(
                    human_x, human_y, lidar_points, used_lidar_indices
                )
                if matched_index is not None:
                    used_lidar_indices.add(matched_index)
            humans.append((human_x, human_y, human_yaw))

        if can_fuse_lidar and used_lidar_indices:
            self.last_fused_lidar_generation = lidar_generation

        with self.tracker_lock:
            self.human_tracker.update(humans, measurement_ns)

    # ---- lidar ------------------------------------------------------------

    def scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

        sensor_max = float(msg.range_max) if np.isfinite(msg.range_max) else self.lidar_max_range
        minimum = max(float(msg.range_min), self.lidar_min_range)
        maximum = min(sensor_max, self.lidar_max_range)
        valid = np.isfinite(ranges) & (ranges >= minimum) & (ranges <= maximum)

        now_ns = self.get_clock().now().nanoseconds
        self._handle_clock_reset(now_ns)

        scan_time = Time.from_msg(msg.header.stamp)
        scan_stamp_ns = scan_time.nanoseconds
        if scan_stamp_ns <= 0:
            scan_stamp_ns = now_ns
            scan_time = Time(nanoseconds=scan_stamp_ns)

        if not np.any(valid):
            with self.lidar_lock:
                self.lidar_points_odom = np.empty((0, 2), dtype=np.float64)
                self.lidar_person_centers_odom = np.empty((0, 2), dtype=np.float64)
                self.last_lidar_update_ns = now_ns
                self.last_lidar_stamp_ns = scan_stamp_ns
                self.lidar_generation += 1
            return

        source_frame = msg.header.frame_id or "lidar_link"
        looked_up = self._lookup_2d_transform(
            "odom", source_frame, msg.header.stamp, 0.1
        )
        if looked_up is None:
            if now_ns - self._last_lidar_tf_warning_ns > 5_000_000_000:
                self.get_logger().warn(f"Cannot transform LiDAR {source_frame} -> odom")
                self._last_lidar_tf_warning_ns = now_ns
            return
        tx_o, ty_o, yaw_o = looked_up

        indices = np.nonzero(valid)[0]
        angles = float(msg.angle_min) + indices * float(msg.angle_increment)
        valid_ranges = ranges[valid]
        sensor_x = valid_ranges * np.cos(angles)
        sensor_y = valid_ranges * np.sin(angles)

        cos_yaw, sin_yaw = np.cos(yaw_o), np.sin(yaw_o)
        odom_x = tx_o + cos_yaw * sensor_x - sin_yaw * sensor_y
        odom_y = ty_o + sin_yaw * sensor_x + cos_yaw * sensor_y
        odom_points = np.column_stack((odom_x, odom_y))

        person_centers_odom = cluster_lidar_person_centers(
            odom_points,
            indices,
            (tx_o, ty_o),
            self.lidar_person_cluster_min_points,
            self.lidar_person_cluster_max_gap,
            self.lidar_person_cluster_max_diameter,
            self.lidar_person_radius,
        )
        person_centers_odom = self._filter_by_static_map_via_odom(
            person_centers_odom, msg.header.stamp
        )

        centers_msg = PoseArray()
        centers_msg.header.stamp = msg.header.stamp
        centers_msg.header.frame_id = "odom"
        for center_x, center_y in person_centers_odom:
            pose = Pose()
            pose.position.x = float(center_x)
            pose.position.y = float(center_y)
            pose.orientation.w = 1.0
            centers_msg.poses.append(pose)
        self.lidar_person_centers_pub.publish(centers_msg)

        with self.lidar_lock:
            self.lidar_points_odom = odom_points
            self.lidar_person_centers_odom = np.asarray(
                person_centers_odom, dtype=np.float64
            ).reshape((-1, 2))
            self.last_lidar_update_ns = now_ns
            self.last_lidar_stamp_ns = scan_stamp_ns
            self.lidar_generation += 1

    def _filter_by_static_map_via_odom(self, centers_odom, stamp):
        """Filter clusters against the static map (map frame) and return the result in odom.

        The static map is always in the map frame, so one extra transform is needed,
        but only for a few cluster centres (usually < 5), not for the whole scan.
        """
        if not centers_odom:
            return centers_odom

        with self.map_lock:
            map_msg = self.map_msg
            static_grid = self.static_grid_cache
            static_clearance = self.static_clearance_cache
        if map_msg is None or static_grid is None or static_clearance is None:
            return centers_odom

        looked_up = self._lookup_2d_transform("map", "odom", stamp, 0.05)
        if looked_up is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_static_filter_tf_warning_ns > 5_000_000_000:
                self.get_logger().warn(
                    "Cannot transform odom -> map for static-lidar filter; "
                    "skipping the static map filter this time"
                )
                self._last_static_filter_tf_warning_ns = now_ns
            return centers_odom
        tx_m, ty_m, yaw_m = looked_up
        cos_yaw, sin_yaw = np.cos(yaw_m), np.sin(yaw_m)

        centers_map = [
            (
                tx_m + cos_yaw * cx - sin_yaw * cy,
                ty_m + sin_yaw * cx + cos_yaw * cy,
            )
            for cx, cy in centers_odom
        ]

        origin = map_msg.info.origin
        kept_map = filter_lidar_person_centers_by_static_map(
            centers_map,
            static_grid,
            static_clearance,
            float(map_msg.info.resolution),
            float(origin.position.x),
            float(origin.position.y),
            _quaternion_to_yaw(origin.orientation),
            self.lidar_static_rejection_radius,
        )
        kept_map = filter_lidar_person_centers_by_exclusion_zones(
            kept_map, self.lidar_static_exclusion_zones
        )

        # Map back to odom by order (the filter keeps the relative order and does not
        # change values), avoiding matching by float value.
        kept_set = set(kept_map)
        result = []
        for (mx, my), (ox, oy) in zip(centers_map, centers_odom):
            if (mx, my) in kept_set:
                result.append((ox, oy))
        return result

    # ---- publish ------------------------------------------------------

    def _track_position_at(self, track, now_ns: int):
        params = self.human_tracker.params
        moving = track.is_moving(
            params.min_hits_for_velocity,
            params.moving_speed_threshold,
            now_ns,
            params.moving_hold_time,
            exit_speed_threshold=params.moving_exit_speed_threshold,
        )
        dt = max(0.0, (now_ns - track.last_update_ns) / 1e9)
        dt = min(dt, params.track_coast_timeout)
        if not moving:
            return track.x, track.y, moving, dt
        current_x, current_y = track.predict(dt)
        return current_x, current_y, moving, dt

    def _filter_publishable_tracks(self, tracks):
        """Remove unconfirmed lidar-only tracks from the published message.

        lidar_only_min_hits / lidar_only_min_displacement only set the
        `lidar_only_confirmed` flag, while every lidar-born track without camera hits is
        labelled LIDAR_ONLY whether confirmed or not, and get_tracks() returns them all.
        Without this filter a fresh wall cluster would be published as a person.

        Kept:
          - camera-born tracks (lidar_origin_x is None),
          - tracks confirmed by the camera (camera_hits > 0, including coasting ones),
          - lidar-only tracks that passed all three thresholds (lidar_only_confirmed).
        A real walking person passes the thresholds after ~1-2 s, so they only appear
        slightly later; they are not lost.
        """
        if self.publish_unconfirmed_lidar_only:
            return tracks
        return [
            track for track in tracks
            if track.lidar_origin_x is None
            or track.camera_hits > 0
            or track.lidar_only_confirmed
        ]

    def _publish_tracked_humans(self, tracks, now) -> None:
        msg = TrackedHumans()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        for track in tracks:
            current_x, current_y, moving, observation_age = self._track_position_at(
                track, now.nanoseconds
            )
            position_covariance = track.covariance_at(
                observation_age, self.human_tracker.params.kf_process_noise_std
            )
            msg.humans.append(
                TrackedHuman(
                    id=track.track_id,
                    x=current_x,
                    y=current_y,
                    yaw=track.yaw,
                    vx=track.vx,
                    vy=track.vy,
                    speed=track.speed,
                    is_moving=moving,
                    hits=track.hits,
                    source=track.source,
                    mode=track.mode,
                    covariance_xx=float(position_covariance[0, 0]),
                    covariance_xy=float(position_covariance[0, 1]),
                    covariance_yy=float(position_covariance[1, 1]),
                    observation_age=observation_age,
                )
            )
        self.tracked_humans_pub.publish(msg)

    def _publish_tracked_human_markers(self, tracks, now) -> None:
        stamp = now.to_msg()
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.stamp = stamp
        delete_marker.header.frame_id = "odom"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for track in tracks:
            current_x, current_y, moving, _ = self._track_position_at(
                track, now.nanoseconds
            )

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = "odom"
            sphere.ns = "tracked_humans_id"
            sphere.id = track.track_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = current_x
            sphere.pose.position.y = current_y
            sphere.pose.position.z = 0.9
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color.a = 0.8
            sphere.color.r = 1.0 if moving else 0.2
            sphere.color.g = 0.2 if moving else 1.0
            sphere.color.b = 0.2
            sphere.lifetime.sec = 1
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = "odom"
            text.ns = "tracked_humans_id"
            text.id = 10000 + track.track_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = current_x
            text.pose.position.y = current_y
            text.pose.position.z = 1.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = text.color.g = text.color.b = 1.0
            text.text = f"H{track.track_id} v={track.speed:.2f}m/s src={track.source} mode={track.mode}"
            text.lifetime.sec = 1
            marker_array.markers.append(text)

        self.tracked_humans_markers_pub.publish(marker_array)

    def timer_cb(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        self._handle_clock_reset(now_ns)

        _, lidar_fresh, lidar_generation, lidar_stamp_ns, lidar_person_centers = (
            self._snapshot_lidar(now_ns)
        )

        with self.tracker_lock:
            new_lidar_scan = lidar_fresh and lidar_generation != self.last_coasted_lidar_generation
            if new_lidar_scan:
                coast_points = [
                    (float(point[0]), float(point[1])) for point in lidar_person_centers
                ]
                self.human_tracker.coast_with_lidar(coast_points, lidar_stamp_ns)
                self.human_tracker.tick(now_ns)
                self.last_coasted_lidar_generation = lidar_generation
            else:
                self.human_tracker.tick(now_ns)
            tracks = self.human_tracker.get_tracks()

        tracks = self._filter_publishable_tracks(tracks)

        self._publish_tracked_humans(tracks, now)
        if self.publish_tracking_markers:
            self._publish_tracked_human_markers(tracks, now)

        self._log_prune_reasons(now_ns)

    def _log_prune_reasons(self, now_ns: int) -> None:
        """Every 10 s, report which branch removed tracks and how much memory is held.

        Shows whether id switches mostly come from lidar_only_track_timeout or from the
        coast branch, so timeouts can be tuned on data.
        """
        if now_ns - self._last_prune_log_ns < 10_000_000_000:
            return
        self._last_prune_log_ns = now_ns
        with self.tracker_lock:
            reasons = dict(self.human_tracker.prune_reasons)
            memory_size = len(self.human_tracker._memory)
        if reasons:
            self.get_logger().info(
                f"removed tracks by branch (cumulative): {reasons}; "
                f"tracks held in memory: {memory_size}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerNode1()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
