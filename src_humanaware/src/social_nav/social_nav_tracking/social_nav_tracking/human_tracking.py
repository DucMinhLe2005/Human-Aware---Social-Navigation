"""Multi-frame human tracker: Hungarian (optimal) association + a
constant-velocity Kalman filter. No rclpy import on purpose, so this
module can be unit-tested without a ROS runtime/Docker/GPU -- numpy/scipy
are fine here since they're already hard dependencies of this package
(see setup.py) and of the node that owns this tracker.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

TRACK_SOURCE_CAMERA = 0
TRACK_SOURCE_LIDAR_COAST = 1
TRACK_SOURCE_LIDAR_ONLY = 2

TRACK_MODE_FUSION = 0
TRACK_MODE_CAMERA_ONLY = 1
TRACK_MODE_LIDAR_ONLY = 2
TRACK_MODE_PREDICTION = 3

# State vector is [x, y, vx, vy]; only (x, y) are observed.
_STATE_DIM = 4
_H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
_IDENTITY4 = np.eye(_STATE_DIM)
# Upper bound on person-memory records. A real room rarely has this many people
# lost at once; the cap prevents the whole room from being remembered as people.
_MEMORY_MAX_ENTRIES = 8


def directional_position_variances(
    covariance_xx: float,
    covariance_xy: float,
    covariance_yy: float,
    yaw: float,
) -> Tuple[float, float]:
    """Project map-frame position covariance onto forward/side axes."""
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))

    variance_forward = (
        cos_yaw * cos_yaw * covariance_xx
        + 2.0 * cos_yaw * sin_yaw * covariance_xy
        + sin_yaw * sin_yaw * covariance_yy
    )

    variance_side = (
        sin_yaw * sin_yaw * covariance_xx
        - 2.0 * sin_yaw * cos_yaw * covariance_xy
        + cos_yaw * cos_yaw * covariance_yy
    )

    return (
        max(0.0, float(variance_forward)),
        max(0.0, float(variance_side)),
    )


def cluster_lidar_person_centers(
    points: np.ndarray,
    scan_indices: np.ndarray,
    sensor_origin: Tuple[float, float],
    min_points: int,
    max_point_gap: float,
    max_diameter: float,
    person_radius: float,
) -> List[Tuple[float, float]]:
    """Estimate person centres from compact, contiguous LaserScan clusters.

    Scan endpoints lie on the visible surface rather than at the person's
    centre. For each compact cluster, estimate a robust radial surface point
    and move it away from the sensor by an approximate body radius. Long
    clusters (normally walls) and isolated returns are rejected before
    tracker association.
    """
    points_array = np.asarray(points, dtype=np.float64)
    indices_array = np.asarray(scan_indices, dtype=np.int64)
    if (
        points_array.ndim != 2
        or points_array.shape[1] != 2
        or points_array.shape[0] != indices_array.size
        or points_array.shape[0] == 0
    ):
        return []

    finite = np.all(np.isfinite(points_array), axis=1)
    finite &= np.isfinite(indices_array)
    points_array = points_array[finite]
    indices_array = indices_array[finite]
    if points_array.shape[0] == 0:
        return []

    order = np.argsort(indices_array)
    points_array = points_array[order]
    indices_array = indices_array[order]

    required_points = max(1, int(min_points))
    point_gap = max(1e-6, float(max_point_gap))
    diameter_limit = max(1e-6, float(max_diameter))
    radius = max(0.0, float(person_radius))
    sensor = np.asarray(sensor_origin, dtype=np.float64)
    if sensor.shape != (2,) or not np.all(np.isfinite(sensor)):
        return []

    split_after = []
    for index in range(1, points_array.shape[0]):
        scan_gap = indices_array[index] - indices_array[index - 1]
        spatial_gap = float(
            np.linalg.norm(points_array[index] - points_array[index - 1])
        )
        # Permit one invalid/missing beam inside a person cluster. The spatial
        # gate still prevents two separated objects from being joined.
        if scan_gap > 2 or spatial_gap > point_gap:
            split_after.append(index)

    cluster_bounds = [0, *split_after, points_array.shape[0]]
    centers: List[Tuple[float, float]] = []
    for start, end in zip(cluster_bounds[:-1], cluster_bounds[1:]):
        cluster = points_array[start:end]
        if cluster.shape[0] < required_points:
            continue

        extent = np.ptp(cluster, axis=0)
        cluster_diameter = float(np.hypot(extent[0], extent[1]))
        if cluster_diameter > diameter_limit:
            continue

        radial_vectors = cluster - sensor
        ranges = np.linalg.norm(radial_vectors, axis=1)
        valid_ranges = ranges > 1e-6
        if not np.any(valid_ranges):
            continue

        # A single shortest return is vulnerable to range speckle and beam
        # switching. Median direction plus a low range percentile retains the
        # visible front surface while rejecting one short-range outlier.
        radial_vectors = radial_vectors[valid_ranges]
        ranges = ranges[valid_ranges]
        direction = np.median(
            radial_vectors / ranges[:, np.newaxis],
            axis=0,
        )
        direction_length = float(np.linalg.norm(direction))
        if direction_length <= 1e-6:
            continue

        direction /= direction_length
        surface_range = float(np.percentile(ranges, 20.0))
        center = sensor + (surface_range + radius) * direction
        centers.append((float(center[0]), float(center[1])))

    return centers


def filter_lidar_person_centers_by_static_map(
    centers: Sequence[Tuple[float, float]],
    static_grid: np.ndarray,
    static_clearance_m: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    rejection_radius: float,
) -> List[Tuple[float, float]]:
    """Remove LiDAR person candidates that coincide with the static map.

    The cluster centre is expressed in the map frame. Candidates outside the
    map, inside unknown space, or no farther than ``rejection_radius`` from a
    static occupied cell are rejected. The map-origin yaw is handled so this
    helper also works for OccupancyGrid maps whose axes are rotated.
    """
    centers_array = np.asarray(centers, dtype=np.float64)
    if centers_array.size == 0:
        return []
    if centers_array.ndim != 2 or centers_array.shape[1] != 2:
        return []

    grid = np.asarray(static_grid)
    clearance = np.asarray(static_clearance_m, dtype=np.float64)
    map_resolution = float(resolution)
    if (
        grid.ndim != 2
        or clearance.shape != grid.shape
        or not np.isfinite(map_resolution)
        or map_resolution <= 0.0
    ):
        return []

    finite = np.all(np.isfinite(centers_array), axis=1)
    dx = np.zeros(centers_array.shape[0], dtype=np.float64)
    dy = np.zeros(centers_array.shape[0], dtype=np.float64)
    dx[finite] = centers_array[finite, 0] - float(origin_x)
    dy[finite] = centers_array[finite, 1] - float(origin_y)
    cos_yaw = np.cos(float(origin_yaw))
    sin_yaw = np.sin(float(origin_yaw))
    map_x = cos_yaw * dx + sin_yaw * dy
    map_y = -sin_yaw * dx + cos_yaw * dy
    cells_x = np.floor(map_x / map_resolution).astype(np.int64)
    cells_y = np.floor(map_y / map_resolution).astype(np.int64)

    height, width = grid.shape
    in_bounds = (
        finite
        & (cells_x >= 0)
        & (cells_x < width)
        & (cells_y >= 0)
        & (cells_y < height)
    )
    keep = np.zeros(centers_array.shape[0], dtype=bool)
    valid_indices = np.nonzero(in_bounds)[0]
    if valid_indices.size:
        valid_x = cells_x[valid_indices]
        valid_y = cells_y[valid_indices]
        min_clearance = max(0.0, float(rejection_radius))
        keep[valid_indices] = (
            (grid[valid_y, valid_x] >= 0)
            & (clearance[valid_y, valid_x] > min_clearance)
        )

    return [
        (float(center[0]), float(center[1]))
        for center in centers_array[keep]
    ]


def filter_lidar_person_centers_by_exclusion_zones(
    centers: Sequence[Tuple[float, float]],
    exclusion_zones: Sequence[Tuple[float, float, float]],
) -> List[Tuple[float, float]]:
    """Remove centres inside configured circular static-object zones."""
    filtered = []
    valid_zones = [
        (float(zone[0]), float(zone[1]), max(0.0, float(zone[2])))
        for zone in exclusion_zones
        if len(zone) >= 3
        and np.all(np.isfinite(zone[:3]))
    ]
    for center in centers:
        if len(center) < 2 or not np.all(np.isfinite(center[:2])):
            continue
        center_x, center_y = float(center[0]), float(center[1])
        inside_static_zone = any(
            math.hypot(center_x - zone_x, center_y - zone_y)
            <= radius
            for zone_x, zone_y, radius in valid_zones
        )
        if not inside_static_zone:
            filtered.append((center_x, center_y))
    return filtered


@dataclass
class HumanTrack:
    track_id: int
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    camera_hits: int = 1
    lidar_hits: int = 0
    created_ns: int = 0
    last_update_ns: int = 0
    last_camera_update_ns: int = 0
    last_lidar_update_ns: Optional[int] = None
    # Last measurement time at which filtered speed was above
    # moving_speed_threshold.
    last_moving_ns: Optional[int] = None
    # Last time the track exceeded the lidar-only speed threshold. Kept separate from
    # `last_moving_ns`, which is_moving() resets to None after the hold time
    # (~0.5 s) and therefore cannot serve as the 3 s demotion timer.
    last_lidar_motion_ns: Optional[int] = None
    last_motion_yaw: Optional[float] = None
    source: int = TRACK_SOURCE_CAMERA
    mode: int = TRACK_MODE_CAMERA_ONLY
    P: Optional[np.ndarray] = None
    # Deadline (ns) by which a coasting track must be re-matched (camera or
    # lidar) or be pruned. Set fresh the moment the track enters coasting
    # (see _reclassify_and_prune) and refreshed on every successful lidar
    # match -- deliberately NOT derived from last_update_ns at that moment,
    # since last_update_ns still reflects the last CAMERA hit, already
    # `track_timeout` stale by definition, which would blow through a
    # shorter track_coast_match_timeout before coasting gets one try.
    coast_deadline_ns: Optional[int] = None
    # LiDAR-only tracks remain private until multi-frame motion confirms that
    # the compact cluster is not stationary, unmapped furniture.
    lidar_origin_x: Optional[float] = None
    lidar_origin_y: Optional[float] = None
    lidar_max_displacement: float = 0.0
    lidar_only_confirmed: bool = False
    # Consecutive ticks in which camera and lidar report the person at the SAME
    # position (fresh_camera_matches branch). Counted separately from lidar_hits,
    # which only increments when the lidar stamp is newer than last_update_ns; with
    # a faster camera that rarely happens.
    fusion_hits: int = 0
    # Number of revivals from person memory and camera_hits at the last revival.
    # Prevents chains: a track revived by lidar and not re-confirmed by the camera
    # may not enter the memory again, otherwise a "person" label could hop from one
    # wall cluster to the next forever.
    memory_revivals: int = 0
    camera_hits_at_revival: int = 0

    @property
    def speed(self) -> float:
        return (self.vx**2 + self.vy**2) ** 0.5

    @property
    def is_coasting(self) -> bool:
        return self.source == TRACK_SOURCE_LIDAR_COAST

    @property
    def is_lidar_only(self) -> bool:
        return self.source == TRACK_SOURCE_LIDAR_ONLY

    def is_moving(
        self,
        min_hits: int,
        speed_threshold: float,
        now_ns: Optional[int] = None,
        hold_time: float = 0.0,
        exit_speed_threshold: Optional[float] = None,
    ) -> bool:
        if max(self.camera_hits, self.lidar_hits) < min_hits:
            return False

        enter_threshold = max(0.0, float(speed_threshold))

        if exit_speed_threshold is None:
            exit_threshold = enter_threshold
        else:
            exit_threshold = min(
                enter_threshold,
                max(0.0, float(exit_speed_threshold)),
            )

        # A stationary track must cross the higher threshold to enter moving.
        if self.speed >= enter_threshold:
            return True

        # No active moving history means speeds between exit and enter
        # cannot start the moving state.
        if self.last_moving_ns is None:
            return False

        # Once moving, remain moving while above the lower exit threshold.
        if self.speed >= exit_threshold:
            return True

        if now_ns is None:
            return False

        moving_age = (now_ns - self.last_moving_ns) / 1e9
        held = 0.0 <= moving_age <= max(0.0, hold_time)

        if not held:
            # Clear the latch. The track must cross enter_threshold again.
            self.last_moving_ns = None

        return held

    def predict(self, dt: float) -> Tuple[float, float]:
        """Constant-velocity extrapolation: p(t) = p_now + v * t."""
        return self.x + self.vx * dt, self.y + self.vy * dt

    def covariance_at(
        self,
        dt: float,
        process_noise_std: float,
    ) -> np.ndarray:
        """Predict state covariance dt seconds after the last KF update."""
        if self.P is None:
            return np.zeros((4, 4), dtype=np.float64)

        prediction_dt = max(0.0, float(dt))
        F, Q = _cv_transition(
            prediction_dt,
            process_noise_std,
        )

        predicted = F @ self.P @ F.T + Q

        # Symmetrise to remove round-off asymmetry in the covariance.
        return 0.5 * (predicted + predicted.T)


@dataclass
class HumanTrackerParams:
    association_gate: float = 0.75
    # Widens the association gate per track by this factor times the track's own
    # filtered speed times the time since its last update. A fixed gate assumes
    # every track is re-detected each frame; a slow inference tick or a change of
    # speed/direction moves the predicted anchor outside it and spawns a new track.
    association_gate_speed_factor: float = 0.5
    min_hits_for_velocity: int = 3
    max_plausible_speed: float = 2.0
    moving_speed_threshold: float = 0.20
    moving_exit_speed_threshold: float = 0.10
    moving_hold_time: float = 0.5
    track_timeout: float = 0.35
    # Kalman filter noise model (constant-velocity state [x, y, vx, vy]).
    kf_process_noise_std: float = 1.0
    kf_measurement_noise_std: float = 0.15
    kf_lidar_measurement_noise_std: float = 0.20
    # --- Reversal handling ----------------------------------------------------
    # A constant-velocity Kalman filter carries the old momentum, so when a person
    # turns around the track is dragged past them and the id can be lost. When a
    # measurement falls BEHIND the prediction along the walking direction, the
    # VELOCITY covariance is multiplied by `reversal_covariance_boost` so the filter
    # trusts the measurement and the velocity flips within 1-2 cycles, keeping the id.
    # boost = 1.0 disables it.
    reversal_speed_threshold: float = 0.25
    reversal_innovation_threshold: float = 0.10
    reversal_covariance_boost: float = 9.0
    # A new track is not exposed (published/used by the costmap) until it
    # has this many total hits. It still ages out via the normal
    # track_timeout/coasting rules below while unconfirmed -- a missed
    # frame doesn't kill it outright -- so a brief detection dropout
    # (e.g. one dropped YOLO frame under load) doesn't restart its ID,
    # while a one-off false-positive detection still expires quietly
    # without ever being exposed.
    track_min_hits_to_confirm: int = 3
    # LiDAR-only "coasting": keep a track alive briefly after camera
    # confirmation lapses, as long as raw LiDAR keeps re-matching it.
    track_lidar_coast_gate: float = 0.5
    track_coast_match_timeout: float = 0.5
    track_coast_timeout: float = 3.0
    # Lidar-only confirmation thresholds. Looser values let cluster jitter (wall
    # corners, mis-clustered static objects) satisfy them within a few cycles and
    # label ghosts as people. A real walking person still passes them within 1 s.
    lidar_only_association_gate: float = 0.25
    lidar_only_min_hits: int = 8
    # Seconds a lidar-only track must have existed before it may be published.
    # When > 0 this REPLACES lidar_only_min_hits; <= 0 keeps the hit count. A hit
    # count depends on update_rate (15 hits = 3.75 s at 4 Hz but 1.5 s at 10 Hz),
    # so raising the rate would silently loosen the anti-ghost filter.
    lidar_only_min_duration: float = 0.0
    # Let LIDAR keep alive a track the CAMERA has already confirmed, without
    # requiring lidar_only_confirmed. Default False.
    #
    # lidar_only_confirmed needs displacement >= 0.60 m AND speed >= 0.20 m/s,
    # which a person STANDING STILL never satisfies. Once the camera loses them the
    # track dies, new lidar tracks there are filtered out, and the standing person
    # vanishes from /planning/tracked_humans with no social zone. This does not
    # loosen the anti-ghost filter: the camera must still label the person first.
    camera_vouched_lidar_hold: bool = False
    # Publish ghost tracks for people who have STOPPED, not only moving ones.
    # The stationary are normally skipped ("a person who stops and is lost goes
    # quiet"), but the most dangerous case falls exactly there: someone walks
    # past the robot, STOPS and TURNS AROUND to come back. While stopped their
    # speed is ~0 so they vanish, the robot reads that space as free and drives
    # through, and they walk into it.
    # Still bounded by person_memory_ttl, and memory still requires the camera to
    # have vouched for the person first, so ghost rejection is not weakened.
    ghost_publish_stationary: bool = False
    lidar_only_min_displacement: float = 0.30
    lidar_only_moving_speed_threshold: float = 0.20
    # --- Furniture labelled as people ------------------------------------------
    # The tracker works in `odom`, and on a real robot the robot's OWN motion makes
    # the centroid of a static object's lidar cluster slide (front leg, then back
    # leg of a chair) far enough to pass the confirmation threshold. Each ghost
    # gets a 0.8 m social zone that can block a corridor and abort the goal.
    #
    # `lidar_only_enabled=False` disables the lidar-only branch entirely (camera
    # only) -- for clean navigation tests or very cluttered environments.
    lidar_only_enabled: bool = True
    # Revoke the "person" label when a lidar-only track stays still longer than
    # this. Without it lidar_only_confirmed is never cleared. Only applies to tracks
    # not confirmed by the camera (a real standing person is still seen by the
    # camera). 0.0 disables demotion.
    lidar_only_confirm_decay_sec: float = 3.0
    # Co-located camera + lidar ticks needed to confirm a person for good.
    fusion_confirm_min_hits: int = 3
    lidar_only_track_timeout: float = 0.5
    # --- Spatial person memory --------------------------------------------------
    # Without it an expired track is deleted and the reappearing cluster gets a NEW
    # id with vx = vy = 0, while the controller predicts people from vx, vy.
    #
    # A track confirmed as a person is moved to memory when it dies, keeping its id,
    # velocity and covariance. The memory zone grows with time
    # (r = base + growth_speed * dt, capped at radius_max) and is matched against
    # both the last observed position and its constant-velocity extrapolation, so
    # a person who stopped and one who kept walking are both recovered.
    person_memory_ttl: float = 5.0
    person_memory_radius_base: float = 0.5
    person_memory_radius_max: float = 2.5
    person_memory_growth_speed: float = 1.5
    # While a person is lost, only MOVING tracks (speed at death >= this) keep being
    # published as a ghost in PREDICTION mode, so AGHPM keeps a social zone that
    # fades with the covariance. A lost standing person is only remembered.
    # 0.0 disables ghosts.
    ghost_publish_min_speed: float = 0.3


def _cv_transition(
    dt: float,
    process_noise_std: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Constant-velocity transition F and discretized white-noise-
    acceleration process noise Q for elapsed time dt. State layout is
    [x, y, vx, vy]: x couples to vx, y couples to vy -- x and y themselves
    are independent of each other."""
    F = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    q = process_noise_std**2
    q_block = q * np.array(
        [
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2],
        ]
    )
    Q = np.zeros((_STATE_DIM, _STATE_DIM))
    for i, j in ((0, 2), (1, 3)):  # (x, vx) and (y, vy) pairs
        Q[i, i] = q_block[0, 0]
        Q[i, j] = q_block[0, 1]
        Q[j, i] = q_block[1, 0]
        Q[j, j] = q_block[1, 1]
    return F, Q


def _optimal_match(
    anchors: List[Tuple[float, float]],
    candidates: List[Tuple[float, float]],
    gates: Sequence[float],
    fallback_anchors: Optional[Sequence[Tuple[float, float]]] = None,
) -> Dict[int, int]:
    """Globally optimal (Hungarian) matching of anchors to candidates.

    Minimises the total distance, restricted to pairs within each anchor's own gate
    (``gates[i]`` for ``anchors[i]``). Returns {anchor_index: candidate_index}.

    ``fallback_anchors`` optionally gives each track a second anchor (distance = min
    of the two). Every current caller passes None.
    """
    if not anchors or not candidates:
        return {}

    max_gate = max(gates) if gates else 0.0
    sentinel = max_gate * 1e4 + 1.0
    cost = np.full((len(anchors), len(candidates)), sentinel)
    true_distance = np.full((len(anchors), len(candidates)), sentinel)
    for i, (ax, ay) in enumerate(anchors):
        gate = gates[i]
        fx, fy = (
            fallback_anchors[i]
            if fallback_anchors is not None and i < len(fallback_anchors)
            else (ax, ay)
        )
        for j, (cx, cy) in enumerate(candidates):
            distance = min(
                math.hypot(ax - cx, ay - cy),
                math.hypot(fx - cx, fy - cy),
            )
            true_distance[i, j] = distance
            if distance <= gate:
                cost[i, j] = distance

    row_indices, col_indices = linear_sum_assignment(cost)
    matches: Dict[int, int] = {}
    for row, col in zip(row_indices, col_indices):
        if true_distance[row, col] <= gates[row]:
            matches[int(row)] = int(col)
    return matches


class HumanTracker:
    """Tracks people across frames from (x, y, yaw) detections in a single frame.

    This project uses the odom frame. Not thread-safe: the caller must lock around
    update() and coast_with_lidar().
    """

    def __init__(self, params: Optional[HumanTrackerParams] = None):
        self.params = params or HumanTrackerParams()
        self._tracks: Dict[int, HumanTrack] = {}
        self._next_id = 0
        # Person memory: tracks once confirmed as people that expired less than
        # person_memory_ttl ago. Values are (track, died_ns).
        self._memory: Dict[int, Tuple[HumanTrack, int]] = {}
        # Time of the latest prune -- get_tracks() receives no now_ns, so ghosts use
        # this to know whether they expired.
        self._last_now_ns = 0
        # Diagnostics: count removed tracks per branch to tell which timeout to tune.
        self.prune_reasons: Counter = Counter()

    def reset(self) -> None:
        """Clear all tracks after a simulation clock reset."""
        self._tracks.clear()
        self._memory.clear()
        self._next_id = 0
        self._last_now_ns = 0
        self.prune_reasons.clear()

    def update(
        self,
        detections: List[Tuple[float, float, float]],
        now_ns: int,
    ) -> List[HumanTrack]:
        matches, unmatched_detections = self._associate(detections, now_ns)

        for track_id, detection_index in matches.items():
            self._update_track(
                self._tracks[track_id],
                detections[detection_index],
                now_ns,
                is_camera=True,
            )

        for detection_index in unmatched_detections:
            detection = detections[detection_index]
            # A person leaving and re-entering the view gets their old id back instead of
            # becoming a new person with zero velocity.
            if self._revive_from_memory(detection, now_ns, is_camera=True):
                continue
            track = self._create_track(detection, now_ns)
            self._tracks[track.track_id] = track

        self._reclassify_and_prune(now_ns)
        return self._confirmed_tracks()

    def coast_with_lidar(
        self,
        lidar_points: List[Tuple[float, float]],
        now_ns: int,
    ) -> List[HumanTrack]:
        """Update camera-coast and motion-confirmed LiDAR-only tracks.

        Fresh camera tracks reserve their nearby LiDAR centre so the same
        person cannot create a duplicate LiDAR track. Remaining centres are
        associated over time and stay private until their measured motion is
        sufficient to reject stationary clutter.
        """
        lidar_points = [
            (float(point[0]), float(point[1]))
            for point in lidar_points
            if len(point) >= 2
            and np.isfinite(point[0])
            and np.isfinite(point[1])
        ]
        confirmation_threshold = (
            self.params.track_min_hits_to_confirm
        )
        available_indices = set(range(len(lidar_points)))

        def match_available(track_ids, gate):
            candidate_indices = sorted(available_indices)
            anchors = [
                self._tracks[track_id].predict(
                    max(
                        0.0,
                        (
                            now_ns
                            - self._tracks[track_id].last_update_ns
                        )
                        / 1e9,
                    )
                )
                for track_id in track_ids
            ]
            candidates = [
                lidar_points[index]
                for index in candidate_indices
            ]
            matches = _optimal_match(
                anchors,
                candidates,
                [gate] * len(anchors),
            )
            resolved = [
                (
                    track_ids[anchor_index],
                    candidate_indices[candidate_index],
                )
                for anchor_index, candidate_index in matches.items()
            ]
            for _, point_index in resolved:
                available_indices.discard(point_index)
            return resolved

        fresh_camera_track_ids = [
            track_id
            for track_id, track in self._tracks.items()
            if (
                track.camera_hits > 0
                and (now_ns - track.last_camera_update_ns) / 1e9
                <= self.params.track_timeout
            )
        ]
        # Match camera-fresh tracks first so the same LiDAR centre cannot
        # create a duplicate track. A newer LiDAR scan then performs the
        # second asynchronous KF correction for the same person.
        fresh_camera_matches = match_available(
            fresh_camera_track_ids,
            self.params.track_lidar_coast_gate,
        )

        for track_id, point_index in fresh_camera_matches:
            track = self._tracks[track_id]
            lidar_x, lidar_y = lidar_points[point_index]

            self._update_track(
                track,
                (lidar_x, lidar_y, track.yaw),
                now_ns,
                is_camera=False,
            )

            # If the lidar timestamp equals the camera timestamp, _update_track() does not
            # rewind the filter, but the lidar observation of the track is still recorded.
            track.last_lidar_update_ns = now_ns

            # --- Camera -> lidar handover -----------------------------------------------
            # Here the camera sees this person fresh (fresh_camera_track_ids) and lidar has
            # a cluster at the same position (within track_lidar_coast_gate): two
            # independent sensors agree that this is a person.
            #
            # Counted here unconditionally, because _update_track can return early (camera
            # stamp newer than lidar stamp) and then lidar_hits does not increase.
            #
            # After fusion_confirm_min_hits agreeing ticks, set lidar_only_confirmed: when
            # the person walks into the camera's blind spot the track continues as
            # SOURCE_LIDAR_ONLY for as long as lidar follows it, instead of coasting and
            # being removed by track_coast_timeout. Ghosts (never confirmed by the camera,
            # camera_hits == 0) cannot enter this branch and stay subject to the three
            # lidar_only_* thresholds.
            track.fusion_hits += 1
            if track.fusion_hits >= self.params.fusion_confirm_min_hits:
                track.lidar_only_confirmed = True

        coast_track_ids = [
            track_id
            for track_id, track in self._tracks.items()
            if (
                not track.lidar_only_confirmed
                and track.camera_hits >= confirmation_threshold
                and (now_ns - track.last_camera_update_ns) / 1e9
                > self.params.track_timeout
            )
        ]
        for track_id, point_index in match_available(
            coast_track_ids,
            self.params.track_lidar_coast_gate,
        ):
            track = self._tracks[track_id]
            matched_x, matched_y = lidar_points[point_index]
            self._update_track(
                track,
                (matched_x, matched_y, track.yaw),
                now_ns,
                is_camera=False,
            )

        lidar_only_track_ids = [
            track_id
            for track_id, track in self._tracks.items()
            if (
                track.lidar_origin_x is not None
                and track_id not in fresh_camera_track_ids
                and track_id not in coast_track_ids
            )
        ]
        for track_id, point_index in match_available(
            lidar_only_track_ids,
            self.params.lidar_only_association_gate,
        ):
            track = self._tracks[track_id]
            matched_x, matched_y = lidar_points[point_index]
            self._update_track(
                track,
                (matched_x, matched_y, track.yaw),
                now_ns,
                is_camera=False,
            )

        leftover_indices = sorted(available_indices)
        revived_points = self._revive_unambiguous_lidar(
            [lidar_points[index] for index in leftover_indices],
            now_ns,
        )
        for offset, point_index in enumerate(leftover_indices):
            if offset in revived_points:
                continue
            point_x, point_y = lidar_points[point_index]
            track = self._create_lidar_track(
                point_x,
                point_y,
                now_ns,
            )
            self._tracks[track.track_id] = track

        self._reclassify_and_prune(now_ns)
        return self._confirmed_tracks()

    def get_tracks(self) -> List[HumanTrack]:
        """Return the current confirmed tracks without aging them."""
        return self._confirmed_tracks()

    def tick(self, now_ns: int) -> List[HumanTrack]:
        """Age and prune tracks even when no sensor callback arrives."""
        self._reclassify_and_prune(now_ns)
        return self._confirmed_tracks()

    def _confirmed_tracks(self) -> List[HumanTrack]:
        threshold = self.params.track_min_hits_to_confirm
        tracks = [
            track
            for track in self._tracks.values()
            if (
                track.camera_hits >= threshold
                or track.lidar_only_confirmed
            )
        ]
        tracks.extend(self._ghost_tracks())
        return tracks

    def _refresh_track_mode(
        self,
        track: HumanTrack,
        now_ns: int,
    ) -> None:
        camera_fresh = (
            track.camera_hits > 0
            and (now_ns - track.last_camera_update_ns) / 1e9
            <= self.params.track_timeout
        )

        lidar_fresh = (
            track.last_lidar_update_ns is not None
            and (now_ns - track.last_lidar_update_ns) / 1e9
            <= self.params.lidar_only_track_timeout
        )

        if camera_fresh and lidar_fresh:
            track.mode = TRACK_MODE_FUSION
        elif camera_fresh:
            track.mode = TRACK_MODE_CAMERA_ONLY
        elif lidar_fresh:
            track.mode = TRACK_MODE_LIDAR_ONLY
        else:
            track.mode = TRACK_MODE_PREDICTION

    def _associate(
        self,
        detections: List[Tuple[float, float, float]],
        now_ns: int,
    ) -> Tuple[Dict[int, int], List[int]]:
        track_ids = list(self._tracks.keys())
        dts = [
            max(0.0, (now_ns - self._tracks[track_id].last_update_ns) / 1e9)
            for track_id in track_ids
        ]
        anchors = [
            self._tracks[track_id].predict(dt)
            for track_id, dt in zip(track_ids, dts)
        ]
        gates = [
            self.params.association_gate
            + self.params.association_gate_speed_factor
            * self._tracks[track_id].speed
            * dt
            for track_id, dt in zip(track_ids, dts)
        ]
        # A dual anchor (adding the last measured position as a second anchor) was
        # tried and dropped: it widened the effective gate of every track, so tracks
        # competed for detections and a stale standing track stole the walking
        # person's detection (more identity switches than the extrapolated anchor alone).
        candidates = [(det[0], det[1]) for det in detections]
        raw_matches = _optimal_match(anchors, candidates, gates)

        matches = {
            track_ids[anchor_index]: detection_index
            for anchor_index, detection_index in raw_matches.items()
        }
        matched_detections = set(matches.values())
        unmatched_detections = [
            index
            for index in range(len(detections))
            if index not in matched_detections
        ]
        return matches, unmatched_detections

    def _update_track(
        self,
        track: HumanTrack,
        detection: Tuple[float, float, float],
        now_ns: int,
        is_camera: bool,
    ) -> None:
        if now_ns <= track.last_update_ns:
            # A detector result can arrive after a newer LiDAR scan because
            # inference is slower than the scan callback. Never rewind the
            # Kalman state, but do let a successfully associated camera
            # observation re-confirm the track. Otherwise repeated delayed
            # images could leave it stuck in LiDAR-coasting forever.
            if is_camera and now_ns > track.last_camera_update_ns:
                track.yaw = detection[2]
                track.hits += 1
                track.camera_hits += 1
                track.last_camera_update_ns = now_ns
                track.coast_deadline_ns = None
            return

        det_x, det_y, det_yaw = detection
        dt = max(0.0, (now_ns - track.last_update_ns) / 1e9)

        F, Q = _cv_transition(dt, self.params.kf_process_noise_std)
        state = np.array([track.x, track.y, track.vx, track.vy])
        state_pred = F @ state
        P_pred = F @ track.P @ F.T + Q

        measurement_std = (
            self.params.kf_measurement_noise_std
            if is_camera
            else self.params.kf_lidar_measurement_noise_std
        )
        measurement_noise = max(1e-6, measurement_std) ** 2
        R = measurement_noise * np.eye(2)
        innovation = np.array([det_x, det_y]) - _H @ state_pred

        # Reversal: the measurement is behind the prediction -> boost the velocity
        # covariance. See HumanTrackerParams.reversal_*.
        speed_pred = math.hypot(float(state_pred[2]), float(state_pred[3]))
        if speed_pred >= self.params.reversal_speed_threshold:
            along = (
                float(innovation[0]) * float(state_pred[2])
                + float(innovation[1]) * float(state_pred[3])
            ) / speed_pred
            if along <= -self.params.reversal_innovation_threshold:
                P_pred[2:4, 2:4] = (
                    P_pred[2:4, 2:4] * self.params.reversal_covariance_boost
                )

        S = _H @ P_pred @ _H.T + R
        K = P_pred @ _H.T @ np.linalg.inv(S)
        state_new = state_pred + K @ innovation
        P_new = (_IDENTITY4 - K @ _H) @ P_pred

        vx, vy = float(state_new[2]), float(state_new[3])
        speed = math.hypot(vx, vy)
        if speed > self.params.max_plausible_speed and speed > 0.0:
            scale = self.params.max_plausible_speed / speed
            vx *= scale
            vy *= scale

        track.x, track.y = float(state_new[0]), float(state_new[1])
        track.vx, track.vy = vx, vy
        track.P = P_new
        track.yaw = det_yaw
        track.hits += 1
        track.last_update_ns = now_ns

        if is_camera:
            track.camera_hits += 1
            track.last_camera_update_ns = now_ns
        else:
            track.lidar_hits += 1
            track.last_lidar_update_ns = now_ns
            if track.lidar_origin_x is None:
                track.lidar_origin_x = det_x
                track.lidar_origin_y = det_y
            else:
                displacement = math.hypot(
                    det_x - track.lidar_origin_x,
                    det_y - track.lidar_origin_y,
                )
                track.lidar_max_displacement = max(
                    track.lidar_max_displacement,
                    displacement,
                )
            track.coast_deadline_ns = now_ns + int(
                (
                    self.params.lidar_only_track_timeout
                    if (
                        track.camera_hits == 0
                        or track.lidar_only_confirmed
                    )
                    else self.params.track_coast_match_timeout
                )
                * 1e9
            )
        enough_observations = (
            max(track.camera_hits, track.lidar_hits)
            >= self.params.min_hits_for_velocity
        )

        if (
            enough_observations
            and track.speed
            >= self.params.moving_speed_threshold
        ):
            track.last_moving_ns = now_ns
            track.last_motion_yaw = math.atan2(
                track.vy,
                track.vx,
            )
            if track.lidar_origin_x is not None and not is_camera:
                track.yaw = track.last_motion_yaw

        if track.speed >= self.params.lidar_only_moving_speed_threshold:
            track.last_lidar_motion_ns = now_ns

        # "Has lived long enough": measured in SECONDS when
        # lidar_only_min_duration > 0, otherwise the old hit count. See the
        # parameter's comment for why.
        if self.params.lidar_only_min_duration > 0.0:
            da_du_lau = (
                (now_ns - track.created_ns) / 1e9
                >= self.params.lidar_only_min_duration
            )
        else:
            da_du_lau = track.lidar_hits >= self.params.lidar_only_min_hits

        if (
            self.params.lidar_only_enabled
            and track.lidar_origin_x is not None
            and da_du_lau
            and track.lidar_max_displacement
            >= self.params.lidar_only_min_displacement
            and track.speed
            >= self.params.lidar_only_moving_speed_threshold
        ):
            track.lidar_only_confirmed = True
        elif (
            track.lidar_only_confirmed
            and self.params.lidar_only_confirm_decay_sec > 0.0
            and track.camera_hits < self.params.track_min_hits_to_confirm
            and track.fusion_hits == 0
            and track.last_lidar_motion_ns is not None
            and (now_ns - track.last_lidar_motion_ns)
            > self.params.lidar_only_confirm_decay_sec * 1e9
        ):
            # Still for too long and never confirmed by the camera: not a person. The track
            # is kept (still useful as an obstacle), it is just no longer published as a PERSON.
            track.lidar_only_confirmed = False

    def _create_track(
        self, detection: Tuple[float, float, float], now_ns: int
    ) -> HumanTrack:
        det_x, det_y, det_yaw = detection
        position_variance = self.params.kf_measurement_noise_std**2
        velocity_variance = self.params.max_plausible_speed**2
        P = np.diag(
            [
                position_variance,
                position_variance,
                velocity_variance,
                velocity_variance,
            ]
        )
        track = HumanTrack(
            track_id=self._next_id,
            x=det_x,
            y=det_y,
            yaw=det_yaw,
            hits=1,
            camera_hits=1,
            created_ns=now_ns,
            last_update_ns=now_ns,
            last_camera_update_ns=now_ns,
            mode=TRACK_MODE_CAMERA_ONLY,
            last_lidar_update_ns=None,
            source=TRACK_SOURCE_CAMERA,
            P=P,
        )
        self._next_id += 1
        return track

    def _create_lidar_track(
        self,
        x: float,
        y: float,
        now_ns: int,
    ) -> HumanTrack:
        position_variance = (
            self.params.kf_lidar_measurement_noise_std**2
        )
        velocity_variance = self.params.max_plausible_speed**2
        track = HumanTrack(
            track_id=self._next_id,
            x=x,
            y=y,
            yaw=0.0,
            hits=1,
            camera_hits=0,
            lidar_hits=1,
            created_ns=now_ns,
            last_update_ns=now_ns,
            last_camera_update_ns=0,
            mode=TRACK_MODE_LIDAR_ONLY,
            last_lidar_update_ns=now_ns,
            source=TRACK_SOURCE_LIDAR_ONLY,
            P=np.diag(
                [
                    position_variance,
                    position_variance,
                    velocity_variance,
                    velocity_variance,
                ]
            ),
            coast_deadline_ns=(
                now_ns
                + int(self.params.lidar_only_track_timeout * 1e9)
            ),
            lidar_origin_x=x,
            lidar_origin_y=y,
        )
        self._next_id += 1
        return track

    def _reclassify_and_prune(self, now_ns: int) -> None:
        """Derive source state and prune tracks using source-aware timeouts.

        Camera-origin tracks retain the hard coasting ceiling unless coherent
        LiDAR motion promotes them to LiDAR-only. Tentative and confirmed
        LiDAR-only tracks instead require a recent LiDAR match.
        """
        self._last_now_ns = now_ns
        self._expire_memory(now_ns)
        stale_ids = []
        for track_id, track in self._tracks.items():
            self._refresh_track_mode(track, now_ns)

            camera_age = (
                (now_ns - track.last_camera_update_ns) / 1e9
                if track.camera_hits > 0
                else float("inf")
            )
            if (
                track.camera_hits > 0
                and camera_age <= self.params.track_timeout
            ):
                track.source = TRACK_SOURCE_CAMERA
                track.coast_deadline_ns = None
                continue

            # The camera already labelled this track a person, so lidar may
            # keep it alive without lidar_only_confirmed -- a person STANDING
            # STILL can never meet those three thresholds. See the parameter's
            # comment for the measurement.
            camera_da_bao_lanh = (
                self.params.camera_vouched_lidar_hold
                and track.camera_hits >= self.params.track_min_hits_to_confirm
            )

            if (
                track.lidar_origin_x is not None
                and (
                    track.camera_hits == 0
                    or track.lidar_only_confirmed
                    or camera_da_bao_lanh
                )
            ):
                track.source = TRACK_SOURCE_LIDAR_ONLY
                lidar_age = (now_ns - track.last_update_ns) / 1e9
                if lidar_age > self.params.lidar_only_track_timeout:
                    stale_ids.append((track_id, "lidar_only_track_timeout"))
                continue

            if track.source == TRACK_SOURCE_CAMERA:
                # Just became coast-eligible this tick: start a fresh grace
                # window from now rather than from last_update_ns.
                track.coast_deadline_ns = now_ns + int(
                    self.params.track_coast_match_timeout * 1e9
                )
            track.source = TRACK_SOURCE_LIDAR_COAST

            if now_ns > track.coast_deadline_ns:
                stale_ids.append((track_id, "coast_deadline"))
            elif camera_age > self.params.track_coast_timeout:
                stale_ids.append((track_id, "track_coast_timeout"))

        for track_id, reason in stale_ids:
            self._retire_track(track_id, now_ns, reason)

    # ---- Spatial person memory ------------------------------------------------

    def _retire_track(self, track_id: int, now_ns: int, reason: str) -> None:
        """Retire an expired track: CONFIRMED people go to memory, everything else is deleted.

        Only confirmed people are remembered (enough camera hits, or
        lidar_only_confirmed); remembering unconfirmed flickering clusters would let wall
        clusters be revived whenever they reappear.
        """
        track = self._tracks.pop(track_id, None)
        if track is None:
            return
        self.prune_reasons[reason] += 1
        if self.params.person_memory_ttl <= 0.0:
            return

        # CONDITION 1 -- the label must come from the CAMERA. `lidar_only_confirmed` is
        # not enough: wall clusters sliding past a moving robot can satisfy the three
        # lidar thresholds. Only tracks confirmed by the camera (or by camera+lidar
        # agreement through fusion_hits) may leave a memory record.
        labelled_by_camera = (
            track.camera_hits >= self.params.track_min_hits_to_confirm
            or track.fusion_hits > 0
        )
        # CONDITION 2 -- no chains: a track revived by lidar and never seen again by
        # the camera before dying may not leave a record. A real person seen again by
        # the camera regains that right immediately.
        not_chaining = (
            track.memory_revivals == 0
            or track.camera_hits > track.camera_hits_at_revival
        )
        if not (labelled_by_camera and not_chaining):
            return

        self._memory[track_id] = (track, now_ns)
        # Size cap: keep the newest records so the whole room is not remembered as people.
        if len(self._memory) > _MEMORY_MAX_ENTRIES:
            oldest = min(self._memory.items(), key=lambda item: item[1][1])[0]
            del self._memory[oldest]

    def _expire_memory(self, now_ns: int) -> None:
        ttl = self.params.person_memory_ttl
        expired = [
            track_id
            for track_id, (_, died_ns) in self._memory.items()
            if not 0.0 <= (now_ns - died_ns) / 1e9 <= ttl
        ]
        for track_id in expired:
            del self._memory[track_id]

    def _memory_radius(self, dt: float) -> float:
        """Memory zone radius dt seconds after the person was lost.

        Grows with the maximum walking speed: after dt seconds the person cannot be
        farther than base + growth_speed * dt. Capped at radius_max.
        """
        return min(
            self.params.person_memory_radius_max,
            self.params.person_memory_radius_base
            + self.params.person_memory_growth_speed * max(0.0, dt),
        )

    def _memory_lookup(
        self,
        x: float,
        y: float,
        now_ns: int,
    ) -> Optional[int]:
        """Find the memory record that best matches a new observation at (x, y).

        Matches against both the last observed position and its constant-velocity
        extrapolation, so a person who stopped and one who kept walking are both
        recovered. Scores are normalised by radius, so a newer record (narrow zone)
        beats an older one (wide zone) when both cover the point.
        """
        best_id: Optional[int] = None
        best_score: Optional[float] = None
        for track_id, (track, died_ns) in self._memory.items():
            dt = (now_ns - died_ns) / 1e9
            if not 0.0 <= dt <= self.params.person_memory_ttl:
                continue
            radius = self._memory_radius(dt)
            predicted_x, predicted_y = track.predict(dt)
            distance = min(
                math.hypot(x - track.x, y - track.y),
                math.hypot(x - predicted_x, y - predicted_y),
            )
            if distance > radius:
                continue
            score = distance / radius
            if best_score is None or score < best_score:
                best_id, best_score = track_id, score
        return best_id

    def _revive_from_memory(
        self,
        detection: Tuple[float, float, float],
        now_ns: int,
        is_camera: bool,
        only_id: Optional[int] = None,
    ) -> Optional[HumanTrack]:
        """Revive an old id if the new observation falls inside a memory zone.

        Returns the old HumanTrack object itself, so id, vx, vy, hits and covariance are
        kept. _update_track() handles the Kalman part (F, Q for the elapsed dt), so the
        uncertainty grows with the time the person was lost.
        """
        if self.params.person_memory_ttl <= 0.0 or not self._memory:
            return None
        det_x, det_y, det_yaw = detection
        track_id = (
            only_id
            if only_id is not None
            else self._memory_lookup(det_x, det_y, now_ns)
        )
        if track_id is None or track_id not in self._memory:
            return None
        track, _ = self._memory.pop(track_id)
        # The "person" label is inherited: the cluster does not have to pass the three
        # lidar_only_* thresholds again, and the track stays in the LIDAR_ONLY branch of
        # _reclassify_and_prune instead of the shorter coast branch.
        track.lidar_only_confirmed = True
        track.coast_deadline_ns = None
        track.memory_revivals += 1
        track.camera_hits_at_revival = track.camera_hits
        self._tracks[track_id] = track
        self._update_track(track, (det_x, det_y, det_yaw), now_ns, is_camera)
        return track

    def _revive_unambiguous_lidar(
        self,
        points: Sequence[Tuple[float, float]],
        now_ns: int,
    ) -> set:
        """Revive an old id from a leftover cluster ONLY when it is unambiguous.

        Taking simply the nearest record let a person's label jump to an adjacent wall
        cluster. Lidar cannot tell people from walls, so when a memory zone covers more
        than one cluster (or a cluster lies in several zones), do not guess: the cluster
        follows the normal path and must pass the three lidar_only_* thresholds.

        Returns the set of indices (into `points`) that were revived.
        """
        revived: set = set()
        if self.params.person_memory_ttl <= 0.0 or not self._memory or not points:
            return revived

        # Auxiliary table: which clusters lie in which memory zone.
        candidates: Dict[int, List[int]] = {}
        owners: Dict[int, List[int]] = {}
        for point_index, (point_x, point_y) in enumerate(points):
            for track_id, (track, died_ns) in self._memory.items():
                dt = (now_ns - died_ns) / 1e9
                if not 0.0 <= dt <= self.params.person_memory_ttl:
                    continue
                radius = self._memory_radius(dt)
                predicted_x, predicted_y = track.predict(dt)
                distance = min(
                    math.hypot(point_x - track.x, point_y - track.y),
                    math.hypot(point_x - predicted_x, point_y - predicted_y),
                )
                if distance <= radius:
                    candidates.setdefault(track_id, []).append(point_index)
                    owners.setdefault(point_index, []).append(track_id)

        for track_id, point_indices in candidates.items():
            if len(point_indices) != 1:
                continue  # one remembered person, several clusters -> do not guess
            point_index = point_indices[0]
            if len(owners.get(point_index, [])) != 1:
                continue  # one cluster, several remembered people -> do not guess
            if track_id not in self._memory:
                continue
            point_x, point_y = points[point_index]
            if self._revive_from_memory(
                (point_x, point_y, 0.0), now_ns, is_camera=False, only_id=track_id
            ):
                revived.add(point_index)
        return revived

    def _ghost_tracks(self) -> List[HumanTrack]:
        """Ghost tracks for MOVING people just lost by both camera and lidar.

        A lost standing person is only remembered. A walking person is still published
        in PREDICTION mode with a growing covariance; AghpmLayer1 (with
        social_covariance_enabled) then draws a social zone that widens and fades.
        """
        min_speed = self.params.ghost_publish_min_speed
        include_stationary = self.params.ghost_publish_stationary
        if not self._memory or (min_speed <= 0.0 and not include_stationary):
            return []
        ghosts = []
        for track, died_ns in self._memory.values():
            dt = (self._last_now_ns - died_ns) / 1e9
            if not 0.0 <= dt <= self.params.person_memory_ttl:
                continue
            # A person who has STOPPED is kept when ghost_publish_stationary
            # is on -- that is the most dangerous moment (stopping to turn).
            if not include_stationary and track.speed < min_speed:
                continue
            track.source = TRACK_SOURCE_LIDAR_COAST
            track.mode = TRACK_MODE_PREDICTION
            ghosts.append(track)
        return ghosts
