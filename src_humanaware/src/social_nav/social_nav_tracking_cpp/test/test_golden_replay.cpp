// Replays the COMPLETE run trace of the Python tracker and compares every field.
//
// This is the main evidence of equivalence: same call sequence, same inputs,
// same timestamps -- no timers, no TF, no threads -- so the results must match
// almost exactly. A mismatch is a real bug, not noise.
//
// After every call: the returned id list (INCLUDING order), the number of
// tracks, and for every track all 26 fields plus the 4x4 covariance matrix.

#include <gtest/gtest.h>

#include <cstddef>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "social_nav_tracking_cpp/human_tracking.hpp"
#include "golden_reader.hpp"

using social_nav_tracking_cpp::Detection;
using social_nav_tracking_cpp::HumanTracker;
using social_nav_tracking_cpp::HumanTrackerParams;
using social_nav_tracking_cpp::Point2D;
using social_nav_tracking_cpp::TrackPtr;

namespace
{

// Largest relative difference observed over the whole trace, printed at the end.
double g_max_relative_difference = 0.0;

/// Compares a CONTINUOUS quantity.
///
/// Not bit-for-bit: Python multiplies matrices with numpy (BLAS), C++ with
/// Eigen, and the different summation order changes the last digit (observed
/// around 1e-15 relative). The 1e-12 threshold is ~1000x above that and many
/// orders of magnitude below any meaningful physical quantity. All DISCRETE
/// fields (ids, hits, source/mode, timestamps, iteration order) must still be
/// exactly equal, so if this tolerance ever flipped a decision the test would
/// fail right there.
void expectClose(double actual, double expected, const std::string & what)
{
  const double scale = std::max(1.0, std::fabs(expected));
  const double difference = std::fabs(actual - expected);
  g_max_relative_difference = std::max(g_max_relative_difference, difference / scale);
  EXPECT_NEAR(actual, expected, 1e-12 * scale) << what;
}

HumanTrackerParams paramsFromRecord(const golden::Record & record)
{
  HumanTrackerParams p;
  p.association_gate = record.scalar("association_gate");
  p.association_gate_speed_factor = record.scalar("association_gate_speed_factor");
  p.min_hits_for_velocity = record.integer("min_hits_for_velocity");
  p.max_plausible_speed = record.scalar("max_plausible_speed");
  p.moving_speed_threshold = record.scalar("moving_speed_threshold");
  p.moving_exit_speed_threshold = record.scalar("moving_exit_speed_threshold");
  p.moving_hold_time = record.scalar("moving_hold_time");
  p.track_timeout = record.scalar("track_timeout");
  p.kf_process_noise_std = record.scalar("kf_process_noise_std");
  p.reversal_speed_threshold = record.scalar("reversal_speed_threshold");
  p.reversal_innovation_threshold = record.scalar("reversal_innovation_threshold");
  p.reversal_covariance_boost = record.scalar("reversal_covariance_boost");
  p.kf_measurement_noise_std = record.scalar("kf_measurement_noise_std");
  p.kf_lidar_measurement_noise_std = record.scalar("kf_lidar_measurement_noise_std");
  p.track_min_hits_to_confirm = record.integer("track_min_hits_to_confirm");
  p.track_lidar_coast_gate = record.scalar("track_lidar_coast_gate");
  p.track_coast_match_timeout = record.scalar("track_coast_match_timeout");
  p.track_coast_timeout = record.scalar("track_coast_timeout");
  p.lidar_only_association_gate = record.scalar("lidar_only_association_gate");
  p.lidar_only_min_hits = record.integer("lidar_only_min_hits");
  p.lidar_only_min_duration = record.scalar("lidar_only_min_duration");
  p.camera_vouched_lidar_hold = record.integer("camera_vouched_lidar_hold") != 0;
  p.ghost_publish_stationary = record.integer("ghost_publish_stationary") != 0;
  p.lidar_only_min_displacement = record.scalar("lidar_only_min_displacement");
  p.lidar_only_moving_speed_threshold = record.scalar("lidar_only_moving_speed_threshold");
  p.lidar_only_enabled = record.integer("lidar_only_enabled") != 0;
  p.lidar_only_confirm_decay_sec = record.scalar("lidar_only_confirm_decay_sec");
  p.fusion_confirm_min_hits = record.integer("fusion_confirm_min_hits");
  p.lidar_only_track_timeout = record.scalar("lidar_only_track_timeout");
  p.person_memory_ttl = record.scalar("person_memory_ttl");
  p.person_memory_radius_base = record.scalar("person_memory_radius_base");
  p.person_memory_radius_max = record.scalar("person_memory_radius_max");
  p.person_memory_growth_speed = record.scalar("person_memory_growth_speed");
  p.ghost_publish_min_speed = record.scalar("ghost_publish_min_speed");
  return p;
}

int64_t asOptionalTime(const std::string & token, bool & present)
{
  present = token != "none";
  return present ? static_cast<int64_t>(std::strtoll(token.c_str(), nullptr, 10)) : 0;
}

// Compares one track with the token sequence written by Python. The token
// order must match tools/dump_tracker_trace.py.
void expectTrackMatches(
  const TrackPtr & track, const std::vector<std::string> & tokens,
  std::size_t call_index, std::size_t track_index)
{
  const std::string where =
    " [loi goi " + std::to_string(call_index) + ", track " + std::to_string(track_index) + "]";
  ASSERT_GE(tokens.size(), 26u) << where;

  std::size_t k = 0;
  EXPECT_EQ(track->track_id, std::atoi(tokens[k++].c_str())) << "track_id" << where;
  EXPECT_EQ(track->hits, std::atoi(tokens[k++].c_str())) << "hits" << where;
  EXPECT_EQ(track->camera_hits, std::atoi(tokens[k++].c_str())) << "camera_hits" << where;
  EXPECT_EQ(track->lidar_hits, std::atoi(tokens[k++].c_str())) << "lidar_hits" << where;
  EXPECT_EQ(track->source, std::atoi(tokens[k++].c_str())) << "source" << where;
  EXPECT_EQ(track->mode, std::atoi(tokens[k++].c_str())) << "mode" << where;
  EXPECT_EQ(track->fusion_hits, std::atoi(tokens[k++].c_str())) << "fusion_hits" << where;
  EXPECT_EQ(track->memory_revivals, std::atoi(tokens[k++].c_str())) << "memory_revivals" << where;
  EXPECT_EQ(track->camera_hits_at_revival, std::atoi(tokens[k++].c_str()))
    << "camera_hits_at_revival" << where;
  EXPECT_EQ(track->lidar_only_confirmed, std::atoi(tokens[k++].c_str()) != 0)
    << "lidar_only_confirmed" << where;

  expectClose(track->x, std::strtod(tokens[k++].c_str(), nullptr), "x" + where);
  expectClose(track->y, std::strtod(tokens[k++].c_str(), nullptr), "y" + where);
  expectClose(track->yaw, std::strtod(tokens[k++].c_str(), nullptr), "yaw" + where);
  expectClose(track->vx, std::strtod(tokens[k++].c_str(), nullptr), "vx" + where);
  expectClose(track->vy, std::strtod(tokens[k++].c_str(), nullptr), "vy" + where);
  expectClose(
    track->lidar_max_displacement, std::strtod(tokens[k++].c_str(), nullptr),
    "lidar_max_displacement" + where);

  EXPECT_EQ(track->created_ns, std::strtoll(tokens[k++].c_str(), nullptr, 10))
    << "created_ns" << where;
  EXPECT_EQ(track->last_update_ns, std::strtoll(tokens[k++].c_str(), nullptr, 10))
    << "last_update_ns" << where;
  EXPECT_EQ(track->last_camera_update_ns, std::strtoll(tokens[k++].c_str(), nullptr, 10))
    << "last_camera_update_ns" << where;

  const char * optional_time_names[] = {
    "last_lidar_update_ns", "last_moving_ns", "last_lidar_motion_ns", "coast_deadline_ns"};
  const std::optional<int64_t> * optional_times[] = {
    &track->last_lidar_update_ns, &track->last_moving_ns,
    &track->last_lidar_motion_ns, &track->coast_deadline_ns};
  for (std::size_t i = 0; i < 4; ++i) {
    bool present = false;
    const int64_t value = asOptionalTime(tokens[k++], present);
    ASSERT_EQ(optional_times[i]->has_value(), present) << optional_time_names[i] << where;
    if (present) {
      EXPECT_EQ(optional_times[i]->value(), value) << optional_time_names[i] << where;
    }
  }

  const char * optional_float_names[] = {"last_motion_yaw", "lidar_origin_x", "lidar_origin_y"};
  const std::optional<double> * optional_floats[] = {
    &track->last_motion_yaw, &track->lidar_origin_x, &track->lidar_origin_y};
  for (std::size_t i = 0; i < 3; ++i) {
    const std::string & token = tokens[k++];
    const bool present = token != "none";
    ASSERT_EQ(optional_floats[i]->has_value(), present) << optional_float_names[i] << where;
    if (present) {
      expectClose(
        optional_floats[i]->value(), std::strtod(token.c_str(), nullptr),
        std::string(optional_float_names[i]) + where);
    }
  }

  if (tokens[k] == "noP") {
    EXPECT_FALSE(track->P.has_value()) << "P" << where;
    return;
  }
  ASSERT_TRUE(track->P.has_value()) << "P" << where;
  ASSERT_EQ(tokens.size() - k, 16u) << "number of P elements" << where;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      expectClose(
        track->P.value()(row, col), std::strtod(tokens[k++].c_str(), nullptr),
        "P(" + std::to_string(row) + "," + std::to_string(col) + ")" + where);
    }
  }
}

std::vector<Detection> detectionsFrom(const golden::Record & record)
{
  std::vector<Detection> detections;
  if (!record.has("det") || record.tokens("det")[0] == "skip") {
    return detections;
  }
  const auto flat = record.doubles("det");
  for (std::size_t i = 0; i + 2 < flat.size(); i += 3) {
    detections.push_back(Detection{flat[i], flat[i + 1], flat[i + 2]});
  }
  return detections;
}

std::vector<Point2D> lidarFrom(const golden::Record & record)
{
  std::vector<Point2D> points;
  if (!record.has("lidar") || record.tokens("lidar")[0] == "skip") {
    return points;
  }
  const auto flat = record.doubles("lidar");
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    points.push_back(Point2D{flat[i], flat[i + 1]});
  }
  return points;
}

}  // namespace

TEST(GoldenReplay, MatchesPythonOverWholeTrace)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/tracker_trace.txt");
  ASSERT_FALSE(records.empty());

  std::unique_ptr<HumanTracker> tracker;
  std::size_t calls_checked = 0;
  std::size_t tracks_checked = 0;
  std::size_t scenarios = 0;

  for (std::size_t index = 0; index < records.size(); ++index) {
    const auto & record = records[index];

    if (record.has("scenario")) {
      tracker = std::make_unique<HumanTracker>(paramsFromRecord(record));
      ++scenarios;
      continue;
    }

    ASSERT_NE(tracker, nullptr) << "call record before any scenario header";
    const std::string call = record.tokens("call")[0];
    const int64_t now_ns = static_cast<int64_t>(
      std::strtoll(record.tokens("now")[0].c_str(), nullptr, 10));

    std::vector<TrackPtr> returned;
    if (call == "update") {
      returned = tracker->update(detectionsFrom(record), now_ns);
    } else if (call == "coast") {
      returned = tracker->coastWithLidar(lidarFrom(record), now_ns);
    } else {
      returned = tracker->tick(now_ns);
    }

    // 1. Returned id list, INCLUDING order (the order itself does not matter, but it
    //    is the cheapest sign that the internal iteration order diverged).
    std::vector<std::string> expected_returned;
    if (record.tokens("returned")[0] != "skip") {
      expected_returned = record.tokens("returned");
    }
    ASSERT_EQ(returned.size(), expected_returned.size())
      << "so track tra ve khac nhau [loi goi " << index << ", " << call << "]";
    for (std::size_t i = 0; i < returned.size(); ++i) {
      EXPECT_EQ(returned[i]->track_id, std::atoi(expected_returned[i].c_str()))
        << "returned id #" << i << " [loi goi " << index << "]";
    }

    // 2. The whole internal track table, in iteration order.
    const std::size_t expected_tracks =
      static_cast<std::size_t>(record.integer("ntracks"));
    const auto & tracks = tracker->tracksForTest();
    ASSERT_EQ(tracks.size(), expected_tracks)
      << "internal track count differs [call " << index << ", " << call << "]";
    for (std::size_t i = 0; i < expected_tracks; ++i) {
      expectTrackMatches(tracks[i], record.tokens("t" + std::to_string(i)), index, i);
      ++tracks_checked;
    }

    // 3. Person memory: same ids, same insertion order.
    std::vector<std::string> expected_memory;
    if (record.tokens("memory")[0] != "skip") {
      expected_memory = record.tokens("memory");
    }
    const auto & memory = tracker->memoryForTest();
    ASSERT_EQ(memory.size(), expected_memory.size())
      << "memory record count differs [call " << index << "]";
    for (std::size_t i = 0; i < memory.size(); ++i) {
      EXPECT_EQ(memory[i].track_id, std::atoi(expected_memory[i].c_str()))
        << "memory id #" << i << " [loi goi " << index << "]";
    }

    ++calls_checked;
  }

  std::cout << "[info] replayed " << scenarios << " scenarios, "
            << calls_checked << " calls, " << tracks_checked
            << " full track comparisons; LARGEST relative difference over all"
            << " continuous quantities: " << g_max_relative_difference << std::endl;
  EXPECT_GT(calls_checked, 500u);
  EXPECT_GT(tracks_checked, 500u);
}
