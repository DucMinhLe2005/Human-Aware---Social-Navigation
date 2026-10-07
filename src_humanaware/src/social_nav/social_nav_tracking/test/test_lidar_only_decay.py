"""A standing lidar-only track must lose its "person" label.

Reproduces a case seen on the real robot: a chair published as a person for
834 cycles. The tracker works in `odom`, and the robot's OWN motion makes the
centroid of a static object's lidar cluster slide (front leg, then back leg)
beyond the 0.60 m confirmation threshold. Without demotion the label is
permanent, and AGHPM draws a 0.8 m social zone around the ghost that can block a
corridor and abort the goal.
"""
import pytest

from social_nav_tracking.human_tracking import HumanTracker, HumanTrackerParams

NS_PER_S = 1_000_000_000


def _tracker(**overrides):
    params = HumanTrackerParams(
        lidar_only_min_hits=3,
        lidar_only_min_displacement=0.30,
        lidar_only_moving_speed_threshold=0.20,
        min_hits_for_velocity=2,
        moving_speed_threshold=0.20,
        **overrides,
    )
    return HumanTracker(params), params


def _drive(tracker, xs, t0, dt=0.1, track=None):
    """Feed a sequence of positions through the LIDAR path into one track.

    `track` must be passed to keep updating that track; None creates a new track.
    """
    t = t0
    for x in xs:
        if track is None:
            track = tracker._create_track((x, 0.0, 0.0), t)
            tracker._tracks[track.track_id] = track
        else:
            tracker._update_track(track, (x, 0.0, 0.0), t, is_camera=False)
        t += int(dt * NS_PER_S)
    return track, t


# An object that slides 0.88 m -- above the confirmation threshold, like the chair.
SLIDING = [i * 0.08 for i in range(12)]


def test_sliding_object_gets_confirmed_as_person():
    """Locks the ORIGINAL behaviour (the problem being fixed), not a desired one."""
    tracker, _ = _tracker()
    track, _ = _drive(tracker, SLIDING, 0)
    assert track.lidar_only_confirmed
    assert track.lidar_max_displacement >= 0.30


def test_still_too_long_loses_label():
    tracker, params = _tracker()
    track, t = _drive(tracker, SLIDING, 0)
    assert track.lidar_only_confirmed

    # still for 6 s > lidar_only_confirm_decay_sec (3 s)
    _drive(tracker, [SLIDING[-1]] * 60, t, track=track)

    assert not track.lidar_only_confirmed
    assert track.track_id not in [t.track_id for t in tracker.get_tracks()]


def test_lidar_only_branch_can_be_disabled():
    tracker, _ = _tracker(lidar_only_enabled=False)
    track, _ = _drive(tracker, SLIDING, 0)
    assert not track.lidar_only_confirmed


def test_zero_decay_keeps_original_behaviour():
    tracker, _ = _tracker(lidar_only_confirm_decay_sec=0.0)
    track, t = _drive(tracker, SLIDING, 0)
    _drive(tracker, [SLIDING[-1]] * 60, t, track=track)
    assert track.lidar_only_confirmed, "decay=0 must keep the label forever"


def test_standing_person_seen_by_camera_keeps_label():
    """A real standing person must NOT be demoted -- the camera still confirms them."""
    tracker, params = _tracker()
    track, t = _drive(tracker, SLIDING, 0)
    # seen by the camera often enough to confirm
    for _ in range(params.track_min_hits_to_confirm + 1):
        tracker._update_track(track, (SLIDING[-1], 0.0, 0.0), t, is_camera=True)
        t += int(0.1 * NS_PER_S)
    _drive(tracker, [SLIDING[-1]] * 60, t, track=track)
    assert track.lidar_only_confirmed
