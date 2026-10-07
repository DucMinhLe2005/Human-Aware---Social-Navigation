// Core logic of the lidar safety layer -- no rclcpp dependency.
//
// C++ port of social_nav_safety/lidar_safety_node1.py. The core is separated
// from the node so the last guard before the motors can be unit-tested
// without a ROS graph.

#ifndef SOCIAL_NAV_SAFETY_CPP__LIDAR_SAFETY_HPP_
#define SOCIAL_NAV_SAFETY_CPP__LIDAR_SAFETY_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>

namespace social_nav_safety_cpp
{

/// Parameters; names and defaults match declare_parameter() of the Python node.
struct SafetyParams
{
  double control_rate = 20.0;
  double scan_timeout = 0.25;
  double cmd_timeout = 0.25;
  double robot_front_extent = 0.18;
  double front_half_width = 0.22;
  double robot_rear_extent = 0.18;
  double rear_half_width = 0.22;
  double robot_safety_radius = 0.20;
  double turn_slow_distance = 0.15;
  double turn_stop_distance = 0.03;
  double slow_distance = 0.80;
  double stop_distance = 0.45;
  double emergency_distance = 0.30;
  double ttc_slow = 2.0;
  double ttc_stop = 1.0;
  double closing_speed_alpha = 0.35;
  double max_closing_speed = 3.0;
  double release_distance = 0.60;
  double release_ttc = 1.50;
  double release_hold_time = 0.30;
  double escape_angular_speed = 0.3;

  /// Escape while latched: allow driving AWAY from the direction that caused the
  /// latch as long as the corridor on that side is clear. Standing still while a
  /// person approaches is the wrong reaction; without this, a forward latch also
  /// blocked every backward command.
  bool escape_away_enabled = true;
  double escape_away_speed = 0.15;

  /// Throws std::invalid_argument with the same message as the Python node, so
  /// the node fails at start-up instead of running with nonsensical parameters.
  void validate() const;
};

/// Minimal slice of sensor_msgs::msg::LaserScan, so the core needs no ROS headers.
struct ScanView
{
  const float * ranges = nullptr;
  std::size_t count = 0;
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
};

/// The only geometry_msgs::msg::Twist fields that are read or written.
struct VelocityCommand
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

/// State measured from one scan -- exposed so tests can read it.
struct Clearances
{
  double front = 0.0;
  double rear = 0.0;
  double left = 0.0;
  double right = 0.0;
  double closing_speed = 0.0;
};

class LidarSafety
{
public:
  explicit LidarSafety(const SafetyParams & params);

  /// Corresponds to raw_cmd_cb.
  void updateCommand(const VelocityCommand & command, int64_t now_ns);

  /// Corresponds to scan_cb.
  void updateScan(const ScanView & scan, int64_t now_ns);

  /// Corresponds to control_loop: returns the command that WILL be published.
  VelocityCommand step(int64_t now_ns);

  const SafetyParams & params() const {return params_;}
  Clearances clearances() const;
  bool stopLatched() const {return stop_latched_;}

private:
  double gateAngular(double angular_z) const;
  double holdSign(double angular_z, int64_t now_ns);
  double finalizeAngular(double requested_turn, int64_t now_ns);
  double holdLinearSign(double linear_x, int64_t now_ns);

  SafetyParams params_;

  VelocityCommand latest_raw_cmd_;
  std::optional<int64_t> last_cmd_ns_;
  std::optional<int64_t> last_scan_ns_;

  double front_clearance_;
  double rear_clearance_;
  double left_clearance_;
  double right_clearance_;

  std::optional<double> previous_clearance_;
  std::optional<int64_t> previous_scan_ns_;
  double closing_speed_ = 0.0;

  bool stop_latched_ = false;
  std::optional<int64_t> release_safe_since_ns_;

  double escape_direction_sign_ = 0.0;
  std::optional<int64_t> escape_committed_since_ns_;

  double last_output_sign_ = 0.0;
  std::optional<int64_t> sign_committed_since_ns_;

  double last_linear_output_sign_ = 0.0;
  std::optional<int64_t> linear_sign_committed_since_ns_;

  // Direction (+1 forward, -1 backward, 0 unknown / stale) at the moment
  // stop_latched was set. The release check is anchored to this, not to the
  // currently requested direction; otherwise an upstream that alternates
  // forward/backward could release a latch whose hazard was never cleared.
  double latch_direction_sign_ = 0.0;
};

}  // namespace social_nav_safety_cpp

#endif  // SOCIAL_NAV_SAFETY_CPP__LIDAR_SAFETY_HPP_
