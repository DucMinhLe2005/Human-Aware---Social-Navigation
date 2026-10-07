#!/usr/bin/env python3
"""Record the COMPLETE run trace of the Python tracker for the C++ replay test.

The strongest equivalence evidence without a robot: the same call sequence
(update / coast_with_lidar / tick) with the same inputs and timestamps, comparing
the WHOLE track table after EVERY call -- every field, including the 4x4
covariance and the timestamps. No timers, TF or threads are involved, so the
results must match almost exactly; any mismatch is a real bug.

The scenarios exercise the hard branches: people walking straight, people
REVERSING, people leaving and returning (person memory), static clutter whose
cluster slides (lidar-only demotion), and camera timestamps arriving LATER than
the latest lidar scan.

Run: python3 tools/dump_tracker_trace.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_golden import _load_python_tracker, hx, write  # noqa: E402

GOLDEN_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "test", "golden")

PARAM_FIELDS = [
    "association_gate", "association_gate_speed_factor", "min_hits_for_velocity",
    "max_plausible_speed", "moving_speed_threshold", "moving_exit_speed_threshold",
    "moving_hold_time", "track_timeout", "kf_process_noise_std",
    "kf_measurement_noise_std", "kf_lidar_measurement_noise_std",
    "reversal_speed_threshold", "reversal_innovation_threshold",
    "reversal_covariance_boost",
    "track_min_hits_to_confirm", "track_lidar_coast_gate", "track_coast_match_timeout",
    "track_coast_timeout", "lidar_only_association_gate", "lidar_only_min_hits",
    "lidar_only_min_duration", "camera_vouched_lidar_hold",
    "ghost_publish_stationary",
    "lidar_only_min_displacement", "lidar_only_moving_speed_threshold",
    "lidar_only_enabled", "lidar_only_confirm_decay_sec", "fusion_confirm_min_hits",
    "lidar_only_track_timeout", "person_memory_ttl", "person_memory_radius_base",
    "person_memory_radius_max", "person_memory_growth_speed", "ghost_publish_min_speed",
]

INT_FIELDS = ["track_id", "hits", "camera_hits", "lidar_hits", "source", "mode",
              "fusion_hits", "memory_revivals", "camera_hits_at_revival"]
FLOAT_FIELDS = ["x", "y", "yaw", "vx", "vy", "lidar_max_displacement"]
TIME_FIELDS = ["created_ns", "last_update_ns", "last_camera_update_ns"]
OPT_TIME_FIELDS = ["last_lidar_update_ns", "last_moving_ns", "last_lidar_motion_ns",
                   "coast_deadline_ns"]
OPT_FLOAT_FIELDS = ["last_motion_yaw", "lidar_origin_x", "lidar_origin_y"]


def _track_tokens(track):
    tokens = []
    for name in INT_FIELDS:
        tokens.append(str(int(getattr(track, name))))
    tokens.append("1" if track.lidar_only_confirmed else "0")
    for name in FLOAT_FIELDS:
        tokens.append(hx(getattr(track, name)))
    for name in TIME_FIELDS:
        tokens.append(str(int(getattr(track, name))))
    for name in OPT_TIME_FIELDS:
        value = getattr(track, name)
        tokens.append("none" if value is None else str(int(value)))
    for name in OPT_FLOAT_FIELDS:
        value = getattr(track, name)
        tokens.append("none" if value is None else hx(value))
    if track.P is None:
        tokens.append("noP")
    else:
        for value in np.asarray(track.P, dtype=np.float64).ravel():
            tokens.append(hx(value))
    return tokens


def _snapshot(tracker, returned, row):
    row["returned"] = [str(int(t.track_id)) for t in returned] or ["skip"]
    row["ntracks"] = len(tracker._tracks)
    for index, track in enumerate(tracker._tracks.values()):
        row[f"t{index}"] = _track_tokens(track)
    row["memory"] = [str(int(k)) for k in tracker._memory.keys()] or ["skip"]
    row["next_id"] = int(tracker._next_id)


def _walk_scenario(rng, ht, params, n_steps):
    """One scenario: a few people walking back and forth plus sliding static clutter."""
    tracker = ht.HumanTracker(params)
    rows = []

    n_people = int(rng.integers(1, 3))
    people = []
    for _ in range(n_people):
        people.append({
            "x": float(rng.uniform(-3.0, 3.0)),
            "y": float(rng.uniform(-3.0, 3.0)),
            "heading": float(rng.uniform(-math.pi, math.pi)),
            "speed": float(rng.uniform(0.2, 1.0)),
        })
    # Static clutter: its cluster centroid slides as the robot moves (the "chair
    # labelled as a person" case seen on the real robot).
    clutter = {"x": float(rng.uniform(-4.0, 4.0)), "y": float(rng.uniform(-4.0, 4.0))}

    now_ns = int(rng.integers(0, 10 ** 9))
    for step in range(n_steps):
        dt_ns = int(rng.uniform(0.05, 0.35) * 1e9)
        now_ns += dt_ns

        for person in people:
            # Occasional reversal -- where the extrapolated anchor drifts the farthest.
            if rng.uniform() < 0.08:
                person["heading"] += math.pi + float(rng.normal(0.0, 0.3))
            person["x"] += person["speed"] * dt_ns / 1e9 * math.cos(person["heading"])
            person["y"] += person["speed"] * dt_ns / 1e9 * math.sin(person["heading"])
        clutter["x"] += float(rng.normal(0.0, 0.05))
        clutter["y"] += float(rng.normal(0.0, 0.05))

        choice = rng.uniform()
        row = {}
        if choice < 0.45:
            detections = []
            for person in people:
                # The camera sometimes loses the person (occlusion) -> exercises the person memory.
                if rng.uniform() < 0.25:
                    continue
                detections.append((
                    person["x"] + float(rng.normal(0.0, 0.02)),
                    person["y"] + float(rng.normal(0.0, 0.02)),
                    person["heading"] + float(rng.normal(0.0, 0.1)),
                ))
            # Camera timestamp LATER than the latest lidar scan: exercises the
            # "do not rewind the Kalman filter" branch of _update_track.
            stamp_ns = now_ns - (int(rng.uniform(0.0, 0.4) * 1e9) if rng.uniform() < 0.3 else 0)
            row["call"] = ["update"]
            row["now"] = stamp_ns
            flat = []
            for det in detections:
                flat.extend([hx(det[0]), hx(det[1]), hx(det[2])])
            row["det"] = flat or ["skip"]
            returned = tracker.update(detections, stamp_ns)
        elif choice < 0.85:
            centers = []
            for person in people:
                if rng.uniform() < 0.15:
                    continue
                centers.append((
                    person["x"] + float(rng.normal(0.0, 0.03)),
                    person["y"] + float(rng.normal(0.0, 0.03)),
                ))
            centers.append((clutter["x"], clutter["y"]))
            row["call"] = ["coast"]
            row["now"] = now_ns
            flat = []
            for center in centers:
                flat.extend([hx(center[0]), hx(center[1])])
            row["lidar"] = flat or ["skip"]
            returned = tracker.coast_with_lidar(centers, now_ns)
        else:
            row["call"] = ["tick"]
            row["now"] = now_ns
            returned = tracker.tick(now_ns)

        _snapshot(tracker, returned, row)
        rows.append(row)
        _ = step
    return rows


def main():
    ht = _load_python_tracker()
    rng = np.random.default_rng(913202609)
    all_rows = []

    for scenario in range(18):
        params = ht.HumanTrackerParams()
        # Some scenarios use the deployed parameters (YAML), others tightened ones to reach
        # the lidar-only confirmation/demotion branches in fewer steps.
        if scenario % 3 == 1:
            params.track_timeout = 1.0
            params.track_coast_match_timeout = 2.0
            params.track_coast_timeout = 6.0
            params.lidar_only_association_gate = 0.35
            params.lidar_only_min_hits = 15
            params.lidar_only_min_displacement = 0.60
            params.lidar_only_track_timeout = 1.5
            params.person_memory_ttl = 3.0
            params.person_memory_radius_max = 1.2
            params.person_memory_growth_speed = 1.0
        elif scenario % 3 == 2:
            params.lidar_only_min_hits = 3
            params.lidar_only_min_displacement = 0.20
            params.lidar_only_confirm_decay_sec = 1.0
            params.min_hits_for_velocity = 2
            # Some scenarios use the TIME threshold instead of the hit count, so the replay
            # contract also covers lidar_only_min_duration.
            if scenario % 6 == 5:
                params.lidar_only_min_duration = 1.2
                # Cover camera_vouched_lidar_hold in the replay contract.
                params.camera_vouched_lidar_hold = True
                params.ghost_publish_stationary = True

        rows = _walk_scenario(rng, ht, params, n_steps=60)
        header = {"scenario": scenario}
        for name in PARAM_FIELDS:
            value = getattr(params, name)
            if isinstance(value, bool):
                header[name] = 1 if value else 0
            elif isinstance(value, int):
                header[name] = value
            else:
                header[name] = hx(value)
        header["nsteps"] = len(rows)
        all_rows.append(header)
        all_rows.extend(rows)

    write("tracker_trace.txt", all_rows)


if __name__ == "__main__":
    main()
