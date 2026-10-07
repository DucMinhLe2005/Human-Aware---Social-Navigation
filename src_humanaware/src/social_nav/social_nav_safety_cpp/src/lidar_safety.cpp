#include "social_nav_safety_cpp/lidar_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace social_nav_safety_cpp
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Integer nanoseconds -> seconds. Python divides Python ints into a float;
/// C++ integer division would truncate and silently shift every time
/// threshold, so convert to double once here.
inline double nsToSeconds(int64_t delta_ns)
{
  return static_cast<double>(delta_ns) / 1e9;
}
}  // namespace

void SafetyParams::validate() const
{
  if (release_distance <= stop_distance) {
    throw std::invalid_argument("release_distance must be greater than stop_distance");
  }
  if (release_ttc <= ttc_stop) {
    throw std::invalid_argument("release_ttc must be greater than ttc_stop");
  }
  if (!(0.0 < ttc_stop && ttc_stop < ttc_slow)) {
    throw std::invalid_argument("Require 0 < ttc_stop < ttc_slow");
  }
  if (!(0.0 <= closing_speed_alpha && closing_speed_alpha <= 1.0)) {
    throw std::invalid_argument("closing_speed_alpha must be in [0, 1]");
  }
  if (!(0.0 <= emergency_distance && emergency_distance <= stop_distance &&
    stop_distance < slow_distance))
  {
    throw std::invalid_argument("Require emergency_distance <= stop_distance < slow_distance");
  }
  if (!(0.0 <= turn_stop_distance && turn_stop_distance < turn_slow_distance)) {
    throw std::invalid_argument("Require 0 <= turn_stop_distance < turn_slow_distance");
  }
}

LidarSafety::LidarSafety(const SafetyParams & params)
: params_(params),
  front_clearance_(kInf),
  rear_clearance_(kInf),
  left_clearance_(kInf),
  right_clearance_(kInf)
{
  params_.validate();
}

Clearances LidarSafety::clearances() const
{
  return Clearances{front_clearance_, rear_clearance_, left_clearance_, right_clearance_,
    closing_speed_};
}

void LidarSafety::updateCommand(const VelocityCommand & command, int64_t now_ns)
{
  latest_raw_cmd_ = command;
  last_cmd_ns_ = now_ns;
}

void LidarSafety::updateScan(const ScanView & scan, int64_t now_ns)
{
  double minimum_clearance = kInf;
  double rear_clearance_min = kInf;
  double left_clearance = kInf;
  double right_clearance = kInf;

  for (std::size_t index = 0; index < scan.count; ++index) {
    const double measurement = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(measurement)) {
      continue;
    }
    if (measurement < scan.range_min) {
      continue;
    }
    if (measurement > scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double point_x = measurement * std::cos(angle);
    const double point_y = measurement * std::sin(angle);

    if (point_x > 0.0 && std::abs(point_y) <= params_.front_half_width) {
      minimum_clearance = std::min(minimum_clearance, point_x - params_.robot_front_extent);
    }

    if (point_x < 0.0 && std::abs(point_y) <= params_.rear_half_width) {
      rear_clearance_min = std::min(rear_clearance_min, -point_x - params_.robot_rear_extent);
    }

    // A beam exactly on the x axis (point_y == 0.0) counts for neither side,
    // as in the Python reference.
    const double side_clearance = measurement - params_.robot_safety_radius;
    if (point_y > 0.0) {
      left_clearance = std::min(left_clearance, side_clearance);
    } else if (point_y < 0.0) {
      right_clearance = std::min(right_clearance, side_clearance);
    }
  }

  const double current_clearance = std::max(0.0, minimum_clearance);
  const double current_rear_clearance =
    std::isfinite(rear_clearance_min) ? std::max(0.0, rear_clearance_min) : kInf;
  left_clearance_ = std::isfinite(left_clearance) ? std::max(0.0, left_clearance) : kInf;
  right_clearance_ = std::isfinite(right_clearance) ? std::max(0.0, right_clearance) : kInf;

  const bool valid_history =
    previous_clearance_.has_value() && previous_scan_ns_.has_value() &&
    std::isfinite(current_clearance);

  if (valid_history) {
    const double dt = nsToSeconds(now_ns - *previous_scan_ns_);
    if (dt > 0.01 && dt < 0.5) {
      double raw_closing_speed = (*previous_clearance_ - current_clearance) / dt;
      raw_closing_speed = std::max(0.0, std::min(params_.max_closing_speed, raw_closing_speed));

      const double alpha = params_.closing_speed_alpha;
      closing_speed_ = alpha * raw_closing_speed + (1.0 - alpha) * closing_speed_;
    } else {
      closing_speed_ = 0.0;
    }
  } else {
    closing_speed_ = 0.0;
  }

  front_clearance_ = current_clearance;
  rear_clearance_ = current_rear_clearance;
  if (std::isfinite(current_clearance)) {
    previous_clearance_ = current_clearance;
  } else {
    previous_clearance_.reset();
  }
  previous_scan_ns_ = now_ns;
  last_scan_ns_ = now_ns;
}

double LidarSafety::gateAngular(double angular_z) const
{
  if (std::abs(angular_z) <= 1e-6) {
    return 0.0;
  }

  const double side_clearance = std::min(left_clearance_, right_clearance_);
  const double span = params_.turn_slow_distance - params_.turn_stop_distance;
  double side_scale = (side_clearance - params_.turn_stop_distance) / span;
  side_scale = std::max(0.0, std::min(1.0, side_scale));
  return angular_z * side_scale;
}

double LidarSafety::holdSign(double angular_z, int64_t now_ns)
{
  if (std::abs(angular_z) <= 1e-6) {
    last_output_sign_ = 0.0;
    sign_committed_since_ns_.reset();
    return angular_z;
  }

  const double sign = angular_z > 0.0 ? 1.0 : -1.0;

  if (last_output_sign_ == 0.0 || !sign_committed_since_ns_.has_value() ||
    sign == last_output_sign_)
  {
    last_output_sign_ = sign;
    sign_committed_since_ns_ = now_ns;
    return angular_z;
  }

  const double held_seconds = nsToSeconds(now_ns - *sign_committed_since_ns_);
  if (held_seconds < params_.release_hold_time) {
    return 0.0;
  }

  last_output_sign_ = sign;
  sign_committed_since_ns_ = now_ns;
  return angular_z;
}

double LidarSafety::finalizeAngular(double requested_turn, int64_t now_ns)
{
  return holdSign(gateAngular(requested_turn), now_ns);
}

double LidarSafety::holdLinearSign(double linear_x, int64_t now_ns)
{
  if (std::abs(linear_x) <= 1e-6) {
    last_linear_output_sign_ = 0.0;
    linear_sign_committed_since_ns_.reset();
    return linear_x;
  }

  const double sign = linear_x > 0.0 ? 1.0 : -1.0;

  if (last_linear_output_sign_ == 0.0 || !linear_sign_committed_since_ns_.has_value() ||
    sign == last_linear_output_sign_)
  {
    last_linear_output_sign_ = sign;
    linear_sign_committed_since_ns_ = now_ns;
    return linear_x;
  }

  const double held_seconds = nsToSeconds(now_ns - *linear_sign_committed_since_ns_);
  if (held_seconds < params_.release_hold_time) {
    return 0.0;
  }

  last_linear_output_sign_ = sign;
  linear_sign_committed_since_ns_ = now_ns;
  return linear_x;
}

VelocityCommand LidarSafety::step(int64_t now_ns)
{
  const bool cmd_missing = !last_cmd_ns_.has_value();
  const bool scan_missing = !last_scan_ns_.has_value();

  const bool cmd_stale =
    !cmd_missing && nsToSeconds(now_ns - *last_cmd_ns_) > params_.cmd_timeout;
  const bool scan_stale =
    !scan_missing && nsToSeconds(now_ns - *last_scan_ns_) > params_.scan_timeout;

  if (cmd_missing || scan_missing || cmd_stale || scan_stale) {
    if (!stop_latched_) {
      latch_direction_sign_ = 0.0;
    }
    stop_latched_ = true;
    release_safe_since_ns_.reset();
    return VelocityCommand{};
  }

  VelocityCommand safe_cmd;
  safe_cmd.linear_x = latest_raw_cmd_.linear_x;
  safe_cmd.angular_z = latest_raw_cmd_.angular_z;

  double clearance;
  if (safe_cmd.linear_x > 0.0) {
    clearance = front_clearance_;
  } else if (safe_cmd.linear_x < 0.0) {
    clearance = rear_clearance_;
  } else {
    clearance = kInf;
  }

  double ttc = kInf;
  if (safe_cmd.linear_x > 0.0 && closing_speed_ > 1e-3 && std::isfinite(clearance)) {
    ttc = clearance / closing_speed_;
  }

  // No hard stop for side obstacles, on purpose: when a person comes close to
  // one side, the robot must keep moving -- stopping would let the approaching
  // person walk into it. The robot only stops for obstacles inside its
  // forward / backward corridor. emergency_distance is still read and
  // validated but not used, so existing YAML files keep working.
  // Locked by the test LidarSafetyBehaviour.SideObstacleNeverStops.
  const bool stop_required =
    clearance <= params_.stop_distance || ttc <= params_.ttc_stop;

  if (stop_required) {
    if (!stop_latched_) {
      latch_direction_sign_ =
        safe_cmd.linear_x > 0.0 ? 1.0 : (safe_cmd.linear_x < 0.0 ? -1.0 : 0.0);
    }
    stop_latched_ = true;
    release_safe_since_ns_.reset();

    VelocityCommand blocked;
    double requested_turn = safe_cmd.angular_z;

    // The planner still asks to drive straight although lidar blocks it. Rotate
    // in place towards the side with more free space, and keep the chosen
    // direction for release_hold_time to avoid flipping on sensor noise.
    if (safe_cmd.linear_x > 0.0 && std::abs(requested_turn) <= 1e-6) {
      const bool commit_expired =
        !escape_committed_since_ns_.has_value() ||
        nsToSeconds(now_ns - *escape_committed_since_ns_) >= params_.release_hold_time;
      if (escape_direction_sign_ == 0.0 || commit_expired) {
        escape_direction_sign_ = left_clearance_ > right_clearance_ ? 1.0 : -1.0;
        escape_committed_since_ns_ = now_ns;
      }
      requested_turn = escape_direction_sign_ * params_.escape_angular_speed;
    } else {
      escape_direction_sign_ = 0.0;
      escape_committed_since_ns_.reset();
    }

    blocked.angular_z = finalizeAngular(requested_turn, now_ns);
    return blocked;
  }

  if (stop_latched_) {
    // Escape. Latched because one direction is blocked, but the planner asks to
    // drive in the OPPOSITE direction. Moving away from the hazard is never more
    // dangerous than standing still. The corridor on the other side must still be
    // clear (>= release_distance) and the latch is kept, so the robot cannot
    // return towards the original hazard until it has cleared.
    if (params_.escape_away_enabled && latch_direction_sign_ != 0.0 &&
      std::abs(safe_cmd.linear_x) > 1e-6)
    {
      const double request_sign = safe_cmd.linear_x > 0.0 ? 1.0 : -1.0;
      const double away_clearance = request_sign > 0.0 ? front_clearance_ : rear_clearance_;
      if (request_sign == -latch_direction_sign_ &&
        away_clearance >= params_.release_distance)
      {
        VelocityCommand escape;
        escape.linear_x = std::max(
          -params_.escape_away_speed,
          std::min(params_.escape_away_speed, safe_cmd.linear_x));
        escape.angular_z = finalizeAngular(safe_cmd.angular_z, now_ns);
        // Record the issued sign so holdLinearSign blocks repeated reversals
        // afterwards; skip it here because this reversal is intentional.
        last_linear_output_sign_ = request_sign;
        linear_sign_committed_since_ns_ = now_ns;
        release_safe_since_ns_.reset();
        return escape;
      }
    }

    // Anchor the release check to the direction that actually caused the latch,
    // not to the direction raw_cmd.linear.x requests now (see the header).
    double release_clearance;
    double release_ttc;
    if (latch_direction_sign_ > 0.0) {
      release_clearance = front_clearance_;
      release_ttc =
        (closing_speed_ > 1e-3 && std::isfinite(release_clearance)) ?
        release_clearance / closing_speed_ :
        kInf;
    } else if (latch_direction_sign_ < 0.0) {
      release_clearance = rear_clearance_;
      release_ttc = kInf;
    } else {
      release_clearance = clearance;
      release_ttc = ttc;
    }

    const bool release_safe =
      release_clearance >= params_.release_distance && release_ttc >= params_.release_ttc;

    if (!release_safe) {
      release_safe_since_ns_.reset();
      VelocityCommand blocked;
      blocked.angular_z = finalizeAngular(safe_cmd.angular_z, now_ns);
      return blocked;
    }

    if (!release_safe_since_ns_.has_value()) {
      release_safe_since_ns_ = now_ns;
      VelocityCommand blocked;
      blocked.angular_z = finalizeAngular(safe_cmd.angular_z, now_ns);
      return blocked;
    }

    const double safe_duration = nsToSeconds(now_ns - *release_safe_since_ns_);
    if (safe_duration < params_.release_hold_time) {
      VelocityCommand blocked;
      blocked.angular_z = finalizeAngular(safe_cmd.angular_z, now_ns);
      return blocked;
    }

    stop_latched_ = false;
    release_safe_since_ns_.reset();
    latch_direction_sign_ = 0.0;
  }

  escape_direction_sign_ = 0.0;
  escape_committed_since_ns_.reset();

  double scale = 1.0;

  if (std::isfinite(clearance) && clearance < params_.slow_distance) {
    const double distance_scale =
      (clearance - params_.stop_distance) / (params_.slow_distance - params_.stop_distance);
    scale = std::min(scale, std::max(0.0, std::min(1.0, distance_scale)));
  }

  if (ttc < params_.ttc_slow) {
    const double ttc_scale =
      (ttc - params_.ttc_stop) / (params_.ttc_slow - params_.ttc_stop);
    scale = std::min(scale, std::max(0.0, std::min(1.0, ttc_scale)));
  }

  safe_cmd.linear_x = holdLinearSign(safe_cmd.linear_x * scale, now_ns);
  safe_cmd.angular_z = finalizeAngular(safe_cmd.angular_z, now_ns);
  return safe_cmd;
}

}  // namespace social_nav_safety_cpp
