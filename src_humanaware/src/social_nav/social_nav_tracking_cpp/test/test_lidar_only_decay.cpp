// A standing lidar-only track must lose its "person" label.
//
// Ported from social_nav_tracking/test/test_lidar_only_decay.py (5 cases).
//
// Reproduces a case seen on the real robot: a chair published as a person for
// 834 cycles. The tracker works in `odom`, and the ROBOT'S OWN MOTION makes the
// centroid of a static object's lidar cluster slide (front leg, then back leg)
// beyond the 0.60 m confirmation threshold. Without demotion the label is
// permanent, and AGHPM draws a 0.8 m social zone around it that can block a
// corridor and abort the goal.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "social_nav_tracking_cpp/human_tracking.hpp"

using social_nav_tracking_cpp::Detection;
using social_nav_tracking_cpp::HumanTracker;
using social_nav_tracking_cpp::HumanTrackerParams;
using social_nav_tracking_cpp::TrackPtr;

namespace
{

constexpr int64_t kNsPerSecond = 1000000000;

HumanTrackerParams baseParams()
{
  HumanTrackerParams params;
  params.lidar_only_min_hits = 3;
  params.lidar_only_min_displacement = 0.30;
  params.lidar_only_moving_speed_threshold = 0.20;
  params.min_hits_for_velocity = 2;
  params.moving_speed_threshold = 0.20;
  return params;
}

/// Feeds a sequence of positions through the LIDAR path into one track.
///
/// `track` must be passed to keep updating that track; leaving it empty
/// creates a new track instead.
TrackPtr drive(
  HumanTracker & tracker, const std::vector<double> & xs, int64_t t0,
  int64_t & t_out, double dt = 0.1, TrackPtr track = nullptr)
{
  int64_t t = t0;
  for (const double x : xs) {
    if (track == nullptr) {
      track = tracker.createTrack(Detection{x, 0.0, 0.0}, t);
      tracker.adoptTrack(track);
    } else {
      tracker.updateTrack(track, Detection{x, 0.0, 0.0}, t, false);
    }
    t += static_cast<int64_t>(dt * static_cast<double>(kNsPerSecond));
  }
  t_out = t;
  return track;
}

/// An object that slides 0.88 m -- above the confirmation threshold, like the chair.
std::vector<double> sliding()
{
  std::vector<double> xs;
  for (int i = 0; i < 12; ++i) {
    xs.push_back(i * 0.08);
  }
  return xs;
}

bool containsTrackId(const std::vector<TrackPtr> & tracks, int track_id)
{
  return std::any_of(
    tracks.begin(), tracks.end(),
    [track_id](const TrackPtr & track) {return track->track_id == track_id;});
}

}  // namespace

TEST(LidarOnlyDecay, SlidingObjectGetsConfirmedAsPerson)
{
  // Locks the ORIGINAL behaviour (the problem being fixed), not a desired one.
  HumanTracker tracker(baseParams());
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);
  EXPECT_TRUE(track->lidar_only_confirmed);
  EXPECT_GE(track->lidar_max_displacement, 0.30);
}

TEST(LidarOnlyDecay, StillTooLongLosesLabel)
{
  HumanTracker tracker(baseParams());
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);
  ASSERT_TRUE(track->lidar_only_confirmed);

  // still for 6 s > lidar_only_confirm_decay_sec (3 s)
  const std::vector<double> still(60, sliding().back());
  int64_t t_end = 0;
  drive(tracker, still, t, t_end, 0.1, track);

  EXPECT_FALSE(track->lidar_only_confirmed);
  EXPECT_FALSE(containsTrackId(tracker.getTracks(), track->track_id));
}

TEST(LidarOnlyDecay, LidarOnlyBranchCanBeDisabled)
{
  HumanTrackerParams params = baseParams();
  params.lidar_only_enabled = false;
  HumanTracker tracker(params);
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);
  EXPECT_FALSE(track->lidar_only_confirmed);
}

TEST(LidarOnlyDecay, ZeroDecayKeepsOriginalBehaviour)
{
  HumanTrackerParams params = baseParams();
  params.lidar_only_confirm_decay_sec = 0.0;
  HumanTracker tracker(params);
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);

  const std::vector<double> still(60, sliding().back());
  int64_t t_end = 0;
  drive(tracker, still, t, t_end, 0.1, track);

  EXPECT_TRUE(track->lidar_only_confirmed)
    << "decay=0 must keep the label forever, as originally";
}

TEST(LidarOnlyDecay, StandingPersonSeenByCameraKeepsLabel)
{
  // A real standing person must NOT be demoted -- the camera still confirms them.
  const HumanTrackerParams params = baseParams();
  HumanTracker tracker(params);
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);

  // seen by the camera often enough to confirm
  for (int i = 0; i < params.track_min_hits_to_confirm + 1; ++i) {
    tracker.updateTrack(track, Detection{sliding().back(), 0.0, 0.0}, t, true);
    t += static_cast<int64_t>(0.1 * static_cast<double>(kNsPerSecond));
  }

  const std::vector<double> still(60, sliding().back());
  int64_t t_end = 0;
  drive(tracker, still, t, t_end, 0.1, track);

  EXPECT_TRUE(track->lidar_only_confirmed);
}

// ---------------------------------------------------------------------------
// The cases below are not in the Python tests. They lock details that a port
// can easily get wrong and that the Python tests do not check.
// ---------------------------------------------------------------------------

TEST(LidarOnlyDecay, IsMovingClearsLatchAsSideEffect)
{
  // isMoving() CLEARS last_moving_ns after the hold time. The node calls it TWICE
  // per tick (publishing and markers), so the second call can see the cleared
  // state. If the C++ version returned tracks BY VALUE instead of a shared
  // pointer, this side effect would vanish and tracks would stay "moving" longer.
  HumanTracker tracker(baseParams());
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);
  ASSERT_TRUE(track->last_moving_ns.has_value());

  // Speed drops to 0 but the latch remains -> still moving during the hold time.
  track->vx = 0.0;
  track->vy = 0.0;
  const int64_t within_hold = track->last_moving_ns.value() +
    static_cast<int64_t>(0.2 * static_cast<double>(kNsPerSecond));
  EXPECT_TRUE(track->isMoving(2, 0.20, within_hold, 0.5, 0.10));
  EXPECT_TRUE(track->last_moving_ns.has_value());

  // After the hold time: returns false AND clears the latch.
  const int64_t past_hold = track->last_moving_ns.value() +
    static_cast<int64_t>(1.0 * static_cast<double>(kNsPerSecond));
  EXPECT_FALSE(track->isMoving(2, 0.20, past_hold, 0.5, 0.10));
  EXPECT_FALSE(track->last_moving_ns.has_value());
}

TEST(LidarOnlyDecay, LateCameraStampDoesNotRewindKalman)
{
  // A YOLO result can arrive AFTER a newer lidar scan (inference is slower than
  // the scan callback). The camera bookkeeping must be updated WITHOUT touching
  // the Kalman state.
  HumanTracker tracker(baseParams());
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);

  const double x_before = track->x;
  const double y_before = track->y;
  const double vx_before = track->vx;
  const double vy_before = track->vy;
  const Eigen::Matrix4d P_before = track->P.value();
  const int hits_before = track->hits;
  const int camera_hits_before = track->camera_hits;

  const int64_t stale_ns = track->last_update_ns - static_cast<int64_t>(0.05 * 1e9);
  tracker.updateTrack(track, Detection{99.0, 99.0, 1.23}, stale_ns, true);

  EXPECT_DOUBLE_EQ(track->x, x_before);
  EXPECT_DOUBLE_EQ(track->y, y_before);
  EXPECT_DOUBLE_EQ(track->vx, vx_before);
  EXPECT_DOUBLE_EQ(track->vy, vy_before);
  EXPECT_TRUE(track->P.value() == P_before);
  EXPECT_EQ(track->hits, hits_before + 1);
  EXPECT_EQ(track->camera_hits, camera_hits_before + 1);
  EXPECT_DOUBLE_EQ(track->yaw, 1.23);
}

TEST(LidarOnlyDecay, TwoMotionTimestampsAreIndependent)
{
  // The 3 s demotion timer counts from `last_lidar_motion_ns`, NOT
  // `last_moving_ns`, which clears itself after ~0.5 s.
  HumanTracker tracker(baseParams());
  int64_t t = 0;
  const TrackPtr track = drive(tracker, sliding(), 0, t);
  ASSERT_TRUE(track->last_moving_ns.has_value());
  ASSERT_TRUE(track->last_lidar_motion_ns.has_value());

  const int64_t lidar_motion_before = track->last_lidar_motion_ns.value();

  // Force isMoving to release the latch.
  track->vx = 0.0;
  track->vy = 0.0;
  const int64_t past_hold = track->last_moving_ns.value() +
    static_cast<int64_t>(1.0 * static_cast<double>(kNsPerSecond));
  EXPECT_FALSE(track->isMoving(2, 0.20, past_hold, 0.5, 0.10));
  EXPECT_FALSE(track->last_moving_ns.has_value());

  // The lidar timestamp is STILL there -- this is what the demotion timer uses.
  ASSERT_TRUE(track->last_lidar_motion_ns.has_value());
  EXPECT_EQ(track->last_lidar_motion_ns.value(), lidar_motion_before);
}

TEST(LidarOnlyDecay, ParameterDefaultsDoNotDrift)
{
  // Locks the defaults of the HumanTrackerParams dataclass in Python. The
  // deployed YAML overrides most of them, but they apply if the YAML is missing.
  const HumanTrackerParams p;
  EXPECT_DOUBLE_EQ(p.association_gate, 0.75);
  EXPECT_DOUBLE_EQ(p.association_gate_speed_factor, 0.5);
  EXPECT_EQ(p.min_hits_for_velocity, 3);
  EXPECT_DOUBLE_EQ(p.max_plausible_speed, 2.0);
  EXPECT_DOUBLE_EQ(p.moving_speed_threshold, 0.20);
  EXPECT_DOUBLE_EQ(p.moving_exit_speed_threshold, 0.10);
  EXPECT_DOUBLE_EQ(p.moving_hold_time, 0.5);
  EXPECT_DOUBLE_EQ(p.track_timeout, 0.35);
  EXPECT_DOUBLE_EQ(p.kf_process_noise_std, 1.0);
  EXPECT_DOUBLE_EQ(p.kf_measurement_noise_std, 0.15);
  EXPECT_DOUBLE_EQ(p.kf_lidar_measurement_noise_std, 0.20);
  EXPECT_EQ(p.track_min_hits_to_confirm, 3);
  EXPECT_DOUBLE_EQ(p.track_lidar_coast_gate, 0.5);
  EXPECT_DOUBLE_EQ(p.track_coast_match_timeout, 0.5);
  EXPECT_DOUBLE_EQ(p.track_coast_timeout, 3.0);
  EXPECT_DOUBLE_EQ(p.lidar_only_association_gate, 0.25);
  EXPECT_EQ(p.lidar_only_min_hits, 8);
  EXPECT_DOUBLE_EQ(p.lidar_only_min_displacement, 0.30);
  EXPECT_DOUBLE_EQ(p.lidar_only_moving_speed_threshold, 0.20);
  EXPECT_TRUE(p.lidar_only_enabled);
  EXPECT_DOUBLE_EQ(p.lidar_only_confirm_decay_sec, 3.0);
  EXPECT_EQ(p.fusion_confirm_min_hits, 3);
  EXPECT_DOUBLE_EQ(p.lidar_only_track_timeout, 0.5);
  EXPECT_DOUBLE_EQ(p.person_memory_ttl, 5.0);
  EXPECT_DOUBLE_EQ(p.person_memory_radius_base, 0.5);
  EXPECT_DOUBLE_EQ(p.person_memory_radius_max, 2.5);
  EXPECT_DOUBLE_EQ(p.person_memory_growth_speed, 1.5);
  EXPECT_DOUBLE_EQ(p.ghost_publish_min_speed, 0.3);
}
