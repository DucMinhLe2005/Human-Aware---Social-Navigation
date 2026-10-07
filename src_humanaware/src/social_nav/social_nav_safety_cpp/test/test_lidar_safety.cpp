// Replays the exact scenario recorded from the Python node
// (tools/dump_safety_golden.py) and compares EVERY step.
//
// No approximate comparison: this is a latching state machine, and a 1e-9
// difference in closing_speed can flip a threshold and make the two
// implementations diverge for good. Values are compared bit for bit; any
// place where that is impossible must say why.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "social_nav_safety_cpp/lidar_safety.hpp"

using social_nav_safety_cpp::LidarSafety;
using social_nav_safety_cpp::SafetyParams;
using social_nav_safety_cpp::ScanView;
using social_nav_safety_cpp::VelocityCommand;

namespace
{

/// Parses numbers written by Python's float.hex(); strtod accepts %a as well as inf/nan.
double parseHex(const std::string & token)
{
  return std::strtod(token.c_str(), nullptr);
}

struct Record
{
  int64_t t = 0;
  std::string kind;
  std::vector<double> geom;    // angle_min, angle_increment, range_min, range_max
  std::vector<float> ranges;
  std::vector<double> state;   // front, rear, left, right, closing
  std::vector<double> cmd;     // linear, angular
  std::vector<double> out;     // linear, angular
  int latched = 0;
};

std::vector<Record> loadGolden(const std::string & path)
{
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << "cannot open " << path
                              << " -- run tools/dump_safety_golden.py first";

  std::vector<Record> records;
  std::string line;
  bool open_record = false;

  while (std::getline(file, line)) {
    if (line == "--") {
      records.emplace_back();
      open_record = true;
      continue;
    }
    if (!open_record || line.empty()) {
      continue;
    }
    std::istringstream stream(line);
    std::string key;
    stream >> key;
    Record & record = records.back();

    if (key == "t") {
      stream >> record.t;
    } else if (key == "kind") {
      stream >> record.kind;
    } else if (key == "n" || key == "latched") {
      int value = 0;
      stream >> value;
      if (key == "latched") {
        record.latched = value;
      }
    } else {
      std::string token;
      while (stream >> token) {
        const double value = parseHex(token);
        if (key == "geom") {
          record.geom.push_back(value);
        } else if (key == "r") {
          record.ranges.push_back(static_cast<float>(value));
        } else if (key == "state") {
          record.state.push_back(value);
        } else if (key == "cmd") {
          record.cmd.push_back(value);
        } else if (key == "out") {
          record.out.push_back(value);
        }
      }
    }
  }
  return records;
}

/// Same parameters as the YAML file deployed on the robot.
SafetyParams robotParams()
{
  SafetyParams params;
  params.control_rate = 20.0;
  params.scan_timeout = 0.25;
  params.cmd_timeout = 0.25;
  params.robot_front_extent = 0.18;
  params.front_half_width = 0.22;
  params.robot_rear_extent = 0.18;
  params.rear_half_width = 0.22;
  params.robot_safety_radius = 0.20;
  params.turn_slow_distance = 0.15;
  params.turn_stop_distance = 0.03;
  params.slow_distance = 0.80;
  params.stop_distance = 0.45;
  params.emergency_distance = 0.30;
  params.ttc_slow = 2.0;
  params.ttc_stop = 1.0;
  params.closing_speed_alpha = 0.35;
  params.max_closing_speed = 3.0;
  params.release_distance = 0.60;
  params.release_ttc = 1.50;
  params.release_hold_time = 0.30;
  params.escape_angular_speed = 0.3;
  params.escape_away_enabled = true;
  params.escape_away_speed = 0.25;
  return params;
}

/// Accepts both values being infinite (== works on inf, but spelled out so a
/// failure message is readable).
void expectSame(double expected, double actual, const char * what, int64_t t)
{
  if (std::isinf(expected) || std::isinf(actual)) {
    EXPECT_EQ(std::isinf(expected), std::isinf(actual)) << what << " t=" << t;
    EXPECT_EQ(std::signbit(expected), std::signbit(actual)) << what << " t=" << t;
    return;
  }
  EXPECT_DOUBLE_EQ(expected, actual) << what << " t=" << t;
}

}  // namespace

TEST(LidarSafetyGolden, MatchesPythonStepByStep)
{
  const std::vector<Record> records = loadGolden(std::string(GOLDEN_DIR) + "/safety_trace.txt");
  ASSERT_FALSE(records.empty());

  LidarSafety safety(robotParams());

  int tick_count = 0;
  int scan_count = 0;

  for (const Record & record : records) {
    if (record.kind == "cmd") {
      ASSERT_EQ(record.cmd.size(), 2u);
      VelocityCommand command;
      command.linear_x = record.cmd[0];
      command.angular_z = record.cmd[1];
      safety.updateCommand(command, record.t);

    } else if (record.kind == "scan") {
      ASSERT_EQ(record.geom.size(), 4u);
      ASSERT_EQ(record.state.size(), 5u);
      ScanView view;
      view.ranges = record.ranges.data();
      view.count = record.ranges.size();
      view.angle_min = record.geom[0];
      view.angle_increment = record.geom[1];
      view.range_min = record.geom[2];
      view.range_max = record.geom[3];
      safety.updateScan(view, record.t);

      const auto measured = safety.clearances();
      expectSame(record.state[0], measured.front, "front_clearance", record.t);
      expectSame(record.state[1], measured.rear, "rear_clearance", record.t);
      expectSame(record.state[2], measured.left, "left_clearance", record.t);
      expectSame(record.state[3], measured.right, "right_clearance", record.t);
      expectSame(record.state[4], measured.closing_speed, "closing_speed", record.t);
      ++scan_count;

    } else if (record.kind == "tick") {
      ASSERT_EQ(record.out.size(), 2u);
      const VelocityCommand out = safety.step(record.t);
      expectSame(record.out[0], out.linear_x, "linear.x", record.t);
      expectSame(record.out[1], out.angular_z, "angular.z", record.t);
      EXPECT_EQ(record.latched != 0, safety.stopLatched()) << "stop_latched t=" << record.t;
      ++tick_count;
    }
  }

  // The scenario must actually exercise the branches, not stop after a few steps.
  EXPECT_GT(tick_count, 60) << "golden scenario too short to be trusted";
  EXPECT_GT(scan_count, 40);
}

TEST(LidarSafetyParams, ThrowsLikePythonOnInvalidParams)
{
  {
    SafetyParams params = robotParams();
    params.release_distance = params.stop_distance;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  {
    SafetyParams params = robotParams();
    params.release_ttc = params.ttc_stop;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  {
    SafetyParams params = robotParams();
    params.ttc_stop = 0.0;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  {
    SafetyParams params = robotParams();
    params.closing_speed_alpha = 1.5;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  {
    SafetyParams params = robotParams();
    params.emergency_distance = params.stop_distance + 0.1;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  {
    SafetyParams params = robotParams();
    params.turn_stop_distance = params.turn_slow_distance;
    EXPECT_THROW(params.validate(), std::invalid_argument);
  }
  EXPECT_NO_THROW(robotParams().validate());
}

// Locks a SAFETY DECISION, not an omission.
//
// When a person comes close to one side, the robot must keep moving: stopping
// abruptly in front of an approaching person causes the collision instead of
// avoiding it. The robot only stops for obstacles in its forward/backward
// corridor.
//
// This test fails as soon as someone adds a side-distance stop. If it fails,
// do not adapt the test -- revisit the decision above first.
TEST(LidarSafetyBehaviour, SideObstacleNeverStops)
{
  LidarSafety safety(robotParams());

  const int64_t second = 1000000000LL;
  int64_t t = second;

  // Person close on the left at 0.25 m (below emergency_distance 0.30), front
  // completely clear.
  std::vector<float> ranges(72, std::numeric_limits<float>::infinity());
  ranges[54] = 0.25f;   // angle +pi/2 with angle_min = -pi, 72 beams

  ScanView view;
  view.ranges = ranges.data();
  view.count = ranges.size();
  view.angle_min = -M_PI;
  view.angle_increment = 2.0 * M_PI / 72.0;
  view.range_min = 0.12;
  view.range_max = 12.0;

  VelocityCommand command;
  command.linear_x = 0.30;
  command.angular_z = 0.0;

  // Two cycles to pass the start-up phase (closing_speed needs history).
  for (int i = 0; i < 2; ++i) {
    safety.updateCommand(command, t);
    safety.updateScan(view, t);
    (void)safety.step(t);
    t += second / 20;
  }

  safety.updateCommand(command, t);
  safety.updateScan(view, t);
  const VelocityCommand out = safety.step(t);

  EXPECT_FALSE(safety.stopLatched())
    << "someone added a side-distance stop. This BREAKS the safety decision: "
       "a robot stopping in front of an approaching person gets hit. Read the "
       "comment above this test before changing anything.";
  EXPECT_GT(out.linear_x, 0.0);
}

// When a person comes close, the robot must move forward or backward to make
// way; standing still is wrong. Previously a forward latch also blocked every
// backward command. This test locks the fix.
TEST(LidarSafetyBehaviour, LatchedStillAllowsBackingOffWhenRearClear)
{
  LidarSafety safety(robotParams());

  const int64_t second = 1000000000LL;
  int64_t t = second;

  // Person right in front at 0.35 m (below stop_distance 0.45), rear clear.
  std::vector<float> ranges(72, std::numeric_limits<float>::infinity());
  ranges[36] = 0.35f;   // angle 0 with angle_min = -pi, 72 beams

  ScanView view;
  view.ranges = ranges.data();
  view.count = ranges.size();
  view.angle_min = -M_PI;
  view.angle_increment = 2.0 * M_PI / 72.0;
  view.range_min = 0.12;
  view.range_max = 12.0;

  VelocityCommand tien;
  tien.linear_x = 0.30;
  tien.angular_z = 0.0;

  for (int i = 0; i < 3; ++i) {
    safety.updateCommand(tien, t);
    safety.updateScan(view, t);
    (void)safety.step(t);
    t += second / 20;
  }
  ASSERT_TRUE(safety.stopLatched()) << "an obstacle in front must latch";

  // The planner changes its mind and backs off; the rear is clear, so allow it.
  VelocityCommand backward;
  backward.linear_x = -0.25;
  backward.angular_z = 0.0;

  safety.updateCommand(backward, t);
  safety.updateScan(view, t);
  const VelocityCommand out = safety.step(t);

  EXPECT_LT(out.linear_x, 0.0)
    << "the safety layer blocks backing off while latched forward -- this is "
       "what makes the robot freeze in front of a person.";
  EXPECT_GE(out.linear_x, -robotParams().escape_away_speed - 1e-9)
    << "must be limited by escape_away_speed";
  EXPECT_TRUE(safety.stopLatched())
    << "the latch must be KEPT, only the opposite direction is allowed";
}

// With the flag off the old behaviour returns.
TEST(LidarSafetyBehaviour, EscapeDisabledKeepsFullLock)
{
  SafetyParams params = robotParams();
  params.escape_away_enabled = false;
  LidarSafety safety(params);

  const int64_t second = 1000000000LL;
  int64_t t = second;

  std::vector<float> ranges(72, std::numeric_limits<float>::infinity());
  ranges[36] = 0.35f;

  ScanView view;
  view.ranges = ranges.data();
  view.count = ranges.size();
  view.angle_min = -M_PI;
  view.angle_increment = 2.0 * M_PI / 72.0;
  view.range_min = 0.12;
  view.range_max = 12.0;

  VelocityCommand tien;
  tien.linear_x = 0.30;
  for (int i = 0; i < 3; ++i) {
    safety.updateCommand(tien, t);
    safety.updateScan(view, t);
    (void)safety.step(t);
    t += second / 20;
  }

  VelocityCommand backward;
  backward.linear_x = -0.25;
  safety.updateCommand(backward, t);
  safety.updateScan(view, t);
  EXPECT_DOUBLE_EQ(safety.step(t).linear_x, 0.0);
}
