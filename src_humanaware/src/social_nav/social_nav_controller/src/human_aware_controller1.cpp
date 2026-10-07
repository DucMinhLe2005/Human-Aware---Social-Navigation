#include "social_nav_controller/human_aware_controller1.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace social_nav_controller
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

template<typename T>
void declareIfNotDeclared(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value);
  }
}
}  // namespace

void HumanAwareController1::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"HumanAwareController1: failed to lock node"};
  }
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare_all = [&](const std::string & p, auto default_value) {
      declareIfNotDeclared(node, plugin_name_ + "." + p, default_value);
    };

  declare_all("max_linear_vel", max_linear_vel_);
  declare_all("min_linear_vel", min_linear_vel_);
  declare_all("max_angular_vel", max_angular_vel_);
  declare_all("linear_samples", linear_samples_);
  declare_all("angular_samples", angular_samples_);
  declare_all("sim_time", sim_time_);
  declare_all("sim_dt", sim_dt_);
  declare_all("lookahead_dist", lookahead_dist_);
  declare_all("rotate_heading_deadband", rotate_heading_deadband_);
  declare_all("rotate_in_place_angle", rotate_in_place_angle_);
  declare_all("goal_approach_distance", goal_approach_distance_);
  declare_all("goal_yaw_deadband", goal_yaw_deadband_);
  declare_all("goal_yaw_gain", goal_yaw_gain_);
  declare_all("goal_min_rotate_speed", goal_min_rotate_speed_);
  declare_all("w_goal", w_goal_);
  declare_all("w_speed", w_speed_);
  declare_all("w_path", w_path_);
  declare_all("w_social", w_social_);
  declare_all("w_human_proximity", w_human_proximity_);
  declare_all("w_human_ttc", w_human_ttc_);
  declare_all("w_heading", w_heading_);
  declare_all("w_smooth", w_smooth_);
  declare_all("switch_score_margin", switch_score_margin_);
  declare_all("robot_radius", robot_radius_);
  declare_all("collision_cost", collision_cost_);
  declare_all("costmap_check_radius", costmap_check_radius_);
  declare_all("unknown_is_obstacle", unknown_is_obstacle_);
  declare_all("predictive_humans_enabled", predictive_humans_enabled_);
  declare_all("human_radius", human_radius_);
  declare_all("human_safety_margin", human_safety_margin_);
  declare_all("human_hard_margin", human_hard_margin_);
  declare_all("human_uncertainty_rate", human_uncertainty_rate_);
  declare_all("human_reversal_factor", human_reversal_factor_);
  declare_all("human_stationary_uncertainty_rate", human_stationary_uncertainty_rate_);
  declare_all("human_prediction_horizon", human_prediction_horizon_);
  declare_all("human_covariance_gain", human_covariance_gain_);
  declare_all("human_covariance_sigma_cap", human_covariance_sigma_cap_);
  declare_all("tracked_humans_timeout", tracked_humans_timeout_);
  declare_all("require_tracked_humans", require_tracked_humans_);
  declare_all("emergency_ttc_enter", emergency_ttc_enter_);
  declare_all("emergency_ttc_exit", emergency_ttc_exit_);
  declare_all("emergency_exit_hold", emergency_exit_hold_);
  declare_all("emergency_clearance_improvement", emergency_clearance_improvement_);
  declare_all("emergency_clearance_tie_margin", emergency_clearance_tie_margin_);
  declare_all("passing_enabled", passing_enabled_);
  declare_all("yield_enabled", yield_enabled_);
  declare_all("yield_trigger_time", yield_trigger_time_);
  declare_all("yield_clearance", yield_clearance_);
  declare_all("w_yield", w_yield_);
  declare_all("stuck_escape_enabled", stuck_escape_enabled_);
  declare_all("pass_ahead_margin", pass_ahead_margin_);
  declare_all("pass_behind_margin", pass_behind_margin_);
  declare_all("w_passing", w_passing_);
  declare_all("intent_confidence_min", intent_confidence_min_);
  declare_all("intent_ema_alpha", intent_ema_alpha_);
  declare_all("intent_min_speed", intent_min_speed_);
  declare_all("intent_ttl", intent_ttl_);
  declare_all("tracked_humans_topic", std::string("/planning/tracked_humans"));

  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".linear_samples", linear_samples_);
  node->get_parameter(plugin_name_ + ".angular_samples", angular_samples_);
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);
  node->get_parameter(plugin_name_ + ".sim_dt", sim_dt_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".rotate_heading_deadband", rotate_heading_deadband_);
  node->get_parameter(plugin_name_ + ".rotate_in_place_angle", rotate_in_place_angle_);
  node->get_parameter(plugin_name_ + ".goal_approach_distance", goal_approach_distance_);
  node->get_parameter(plugin_name_ + ".goal_yaw_deadband", goal_yaw_deadband_);
  node->get_parameter(plugin_name_ + ".goal_yaw_gain", goal_yaw_gain_);
  node->get_parameter(plugin_name_ + ".goal_min_rotate_speed", goal_min_rotate_speed_);
  node->get_parameter(plugin_name_ + ".w_goal", w_goal_);
  node->get_parameter(plugin_name_ + ".w_speed", w_speed_);
  node->get_parameter(plugin_name_ + ".w_path", w_path_);
  node->get_parameter(plugin_name_ + ".w_social", w_social_);
  node->get_parameter(plugin_name_ + ".w_human_proximity", w_human_proximity_);
  node->get_parameter(plugin_name_ + ".w_human_ttc", w_human_ttc_);
  node->get_parameter(plugin_name_ + ".w_heading", w_heading_);
  node->get_parameter(plugin_name_ + ".w_smooth", w_smooth_);
  node->get_parameter(plugin_name_ + ".switch_score_margin", switch_score_margin_);
  node->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
  node->get_parameter(plugin_name_ + ".collision_cost", collision_cost_);
  node->get_parameter(plugin_name_ + ".costmap_check_radius", costmap_check_radius_);
  node->get_parameter(plugin_name_ + ".unknown_is_obstacle", unknown_is_obstacle_);
  node->get_parameter(plugin_name_ + ".predictive_humans_enabled", predictive_humans_enabled_);
  node->get_parameter(plugin_name_ + ".human_radius", human_radius_);
  node->get_parameter(plugin_name_ + ".human_safety_margin", human_safety_margin_);
  node->get_parameter(plugin_name_ + ".human_hard_margin", human_hard_margin_);
  node->get_parameter(plugin_name_ + ".human_uncertainty_rate", human_uncertainty_rate_);
  node->get_parameter(plugin_name_ + ".human_reversal_factor", human_reversal_factor_);
  node->get_parameter(
    plugin_name_ + ".human_stationary_uncertainty_rate", human_stationary_uncertainty_rate_);
  node->get_parameter(plugin_name_ + ".human_prediction_horizon", human_prediction_horizon_);
  node->get_parameter(plugin_name_ + ".human_covariance_gain", human_covariance_gain_);
  node->get_parameter(plugin_name_ + ".human_covariance_sigma_cap", human_covariance_sigma_cap_);
  node->get_parameter(plugin_name_ + ".tracked_humans_timeout", tracked_humans_timeout_);
  node->get_parameter(plugin_name_ + ".require_tracked_humans", require_tracked_humans_);
  node->get_parameter(plugin_name_ + ".emergency_ttc_enter", emergency_ttc_enter_);
  node->get_parameter(plugin_name_ + ".emergency_ttc_exit", emergency_ttc_exit_);
  node->get_parameter(plugin_name_ + ".emergency_exit_hold", emergency_exit_hold_);
  node->get_parameter(
    plugin_name_ + ".emergency_clearance_improvement", emergency_clearance_improvement_);
  node->get_parameter(
    plugin_name_ + ".emergency_clearance_tie_margin", emergency_clearance_tie_margin_);
  node->get_parameter(plugin_name_ + ".passing_enabled", passing_enabled_);
  node->get_parameter(plugin_name_ + ".yield_enabled", yield_enabled_);
  node->get_parameter(plugin_name_ + ".yield_trigger_time", yield_trigger_time_);
  node->get_parameter(plugin_name_ + ".yield_clearance", yield_clearance_);
  node->get_parameter(plugin_name_ + ".w_yield", w_yield_);
  node->get_parameter(plugin_name_ + ".stuck_escape_enabled", stuck_escape_enabled_);
  node->get_parameter(plugin_name_ + ".pass_ahead_margin", pass_ahead_margin_);
  node->get_parameter(plugin_name_ + ".pass_behind_margin", pass_behind_margin_);
  node->get_parameter(plugin_name_ + ".w_passing", w_passing_);
  node->get_parameter(plugin_name_ + ".intent_confidence_min", intent_confidence_min_);
  node->get_parameter(plugin_name_ + ".intent_ema_alpha", intent_ema_alpha_);
  node->get_parameter(plugin_name_ + ".intent_min_speed", intent_min_speed_);
  node->get_parameter(plugin_name_ + ".intent_ttl", intent_ttl_);
  std::string tracked_humans_topic = "/planning/tracked_humans";
  node->get_parameter(plugin_name_ + ".tracked_humans_topic", tracked_humans_topic);

  tracked_humans_sub_ = node->create_subscription<thesis_msgs::msg::TrackedHumans>(
    tracked_humans_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&HumanAwareController1::trackedHumansCallback, this, std::placeholders::_1));

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/selected_trajectory", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    logger_,
    "HumanAwareController1 (%s) configured: %dx%d samples, sim_time=%.2fs, "
    "R_hard=%.3fm, R_social=%.3fm",
    plugin_name_.c_str(), linear_samples_, angular_samples_, sim_time_, rHard(), rSocial());
}

void HumanAwareController1::cleanup()
{
  tracked_humans_sub_.reset();
  local_path_pub_.reset();
}

void HumanAwareController1::activate()
{
  if (local_path_pub_) {
    local_path_pub_->on_activate();
  }
}

void HumanAwareController1::deactivate()
{
  if (local_path_pub_) {
    local_path_pub_->on_deactivate();
  }
}

void HumanAwareController1::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    return;
  }
  if (percentage) {
    max_linear_vel_ = max_linear_vel_ * speed_limit / 100.0;
  } else {
    max_linear_vel_ = speed_limit;
  }
}

void HumanAwareController1::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void HumanAwareController1::trackedHumansCallback(
  const thesis_msgs::msg::TrackedHumans::SharedPtr msg)
{
  // Tracked humans are published in the odom frame. Reject messages in any other
  // frame instead of silently using wrong coordinates for the risk model.
  if (!msg->header.frame_id.empty() && msg->header.frame_id != "odom") {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "HumanAwareController1: /planning/tracked_humans frame_id='%s' != 'odom' -- message ignored",
      msg->header.frame_id.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(humans_mutex_);
  latest_humans_msg_ = msg;
  latest_humans_stamp_ = rclcpp::Time(msg->header.stamp);
}

bool HumanAwareController1::transformPlanToCostmapFrame()
{
  path_costmap_frame_.clear();
  if (global_plan_.poses.empty()) {
    return false;
  }
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  path_costmap_frame_.reserve(global_plan_.poses.size());

  if (global_plan_.header.frame_id.empty() || global_plan_.header.frame_id == costmap_frame) {
    for (const auto & pose : global_plan_.poses) {
      path_costmap_frame_.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    goal_yaw_costmap_frame_ = tf2::getYaw(global_plan_.poses.back().pose.orientation);
    goal_yaw_valid_ = true;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      costmap_frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "HumanAwareController1: cannot transform %s -> %s: %s",
      global_plan_.header.frame_id.c_str(), costmap_frame.c_str(), e.what());
    return false;
  }
  const double tx = transform.transform.translation.x;
  const double ty = transform.transform.translation.y;
  const double yaw = tf2::getYaw(transform.transform.rotation);
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
  for (const auto & pose : global_plan_.poses) {
    const double x = pose.pose.position.x, y = pose.pose.position.y;
    path_costmap_frame_.push_back(
      {tx + cos_yaw * x - sin_yaw * y, ty + sin_yaw * x + cos_yaw * y});
  }
  goal_yaw_costmap_frame_ =
    yaw + tf2::getYaw(global_plan_.poses.back().pose.orientation);
  goal_yaw_valid_ = true;
  return true;
}

int HumanAwareController1::nearestIndex(const std::vector<Point2D> & path, double x, double y)
{
  if (path.empty()) {
    return 0;
  }
  int best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].x - x, dy = path[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = static_cast<int>(i);
    }
  }
  return best;
}

Point2D HumanAwareController1::pickLookaheadTarget(
  const std::vector<Point2D> & path, int start_index, double lookahead_distance)
{
  if (path.empty()) {
    return {0.0, 0.0};
  }
  double accumulated = 0.0;
  Point2D previous = path[start_index];
  for (size_t i = start_index + 1; i < path.size(); ++i) {
    const Point2D & current = path[i];
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    if (accumulated >= lookahead_distance) {
      return current;
    }
    previous = current;
  }
  return path.back();
}

void HumanAwareController1::simulate(
  double x, double y, double yaw, double v, double w, double duration,
  std::vector<Point2D> & trajectory_out, double & end_yaw_out) const
{
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / sim_dt_)));
  trajectory_out.clear();
  trajectory_out.reserve(steps);
  for (int i = 0; i < steps; ++i) {
    x += v * std::cos(yaw) * sim_dt_;
    y += v * std::sin(yaw) * sim_dt_;
    yaw += w * sim_dt_;
    trajectory_out.push_back({x, y});
  }
  end_yaw_out = yaw;
}

double HumanAwareController1::pathAlignmentCost(const std::vector<Point2D> & trajectory) const
{
  if (trajectory.empty() || path_costmap_frame_.empty()) {
    return 0.0;
  }
  const size_t step = std::max<size_t>(1, trajectory.size() / 8);
  double total = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < trajectory.size(); i += step) {
    double best_d2 = std::numeric_limits<double>::max();
    for (const auto & p : path_costmap_frame_) {
      const double dx = p.x - trajectory[i].x, dy = p.y - trajectory[i].y;
      best_d2 = std::min(best_d2, dx * dx + dy * dy);
    }
    total += std::sqrt(best_d2);
    ++count;
  }
  return count > 0 ? total / static_cast<double>(count) : 0.0;
}

std::pair<bool, double> HumanAwareController1::trajectoryCostmapCost(
  const std::vector<Point2D> & trajectory) const
{
  auto * costmap = costmap_ros_->getCostmap();
  if (trajectory.empty()) {
    return {false, 0.0};
  }
  const double resolution = costmap->getResolution();
  // Check radius around each trajectory point. Default 0.0 (point check): the
  // Nav2 costmap is already inflated by the footprint radius, so sweeping a disc
  // of robot_radius again would double-count the robot size and close narrow
  // passages. robot_radius_ is only used by the human risk model.
  const double check_radius = std::max(0.0, costmap_check_radius_);
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(check_radius / resolution)));
  const double radius_sq = (check_radius / resolution) * (check_radius / resolution);

  unsigned char worst = 0;
  for (const auto & point : trajectory) {
    unsigned int center_mx, center_my;
    if (!costmap->worldToMap(point.x, point.y, center_mx, center_my)) {
      return {true, 1.0};
    }
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_sq + 1e-9) {
          continue;
        }
        const int mx = static_cast<int>(center_mx) + dx;
        const int my = static_cast<int>(center_my) + dy;
        if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
          my >= static_cast<int>(costmap->getSizeInCellsY()))
        {
          return {true, 1.0};
        }
        const unsigned char cost = costmap->getCost(mx, my);
        if (unknown_is_obstacle_ && cost == nav2_costmap_2d::NO_INFORMATION) {
          return {true, 1.0};
        }
        if (cost != nav2_costmap_2d::NO_INFORMATION) {
          worst = std::max(worst, cost);
        }
      }
    }
  }
  if (static_cast<int>(worst) >= collision_cost_) {
    return {true, std::min(1.0, worst / 255.0)};
  }
  return {false, worst / 255.0};
}

double HumanAwareController1::segmentBoundaryTtc(
  double rx, double ry, double vx, double vy, double duration,
  double boundary_start, double boundary_rate)
{
  const double distance_sq = rx * rx + ry * ry;
  if (distance_sq <= boundary_start * boundary_start + 1e-12) {
    return 0.0;
  }
  const double velocity_sq = vx * vx + vy * vy;
  const double position_velocity = rx * vx + ry * vy;
  const double a = velocity_sq - boundary_rate * boundary_rate;
  const double b = 2.0 * (position_velocity - boundary_start * boundary_rate);
  const double c = distance_sq - boundary_start * boundary_start;
  const double epsilon = 1e-12;

  if (std::abs(a) <= epsilon) {
    if (std::abs(b) <= epsilon) {
      return kInf;
    }
    const double root = -c / b;
    if (root >= -epsilon && root <= duration + epsilon) {
      return std::min(duration, std::max(0.0, root));
    }
    return kInf;
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < -epsilon) {
    return kInf;
  }
  const double sqrt_disc = std::sqrt(std::max(0.0, discriminant));
  const double denom = 2.0 * a;
  double earliest = kInf;
  for (const double root : {(-b - sqrt_disc) / denom, (-b + sqrt_disc) / denom}) {
    if (root >= -epsilon && root <= duration + epsilon) {
      earliest = std::min(earliest, std::min(duration, std::max(0.0, root)));
    }
  }
  return earliest;
}

std::pair<double, double> HumanAwareController1::segmentMinimumClearance(
  double rx, double ry, double vx, double vy, double duration,
  double boundary_start, double boundary_rate)
{
  const double position_sq = rx * rx + ry * ry;
  const double velocity_sq = vx * vx + vy * vy;
  const double position_velocity = rx * vx + ry * vy;

  double best_time = 0.0;
  double best_clearance = std::sqrt(position_sq) - boundary_start;

  auto consider = [&](double t) {
      const double x = rx + vx * t, y = ry + vy * t;
      const double clearance = std::hypot(x, y) - (boundary_start + boundary_rate * t);
      if (clearance < best_clearance || (clearance == best_clearance && t < best_time)) {
        best_clearance = clearance;
        best_time = t;
      }
    };

  consider(duration);
  if (velocity_sq > 1e-24) {
    const double speed = std::sqrt(velocity_sq);
    const double raw_closest = -position_velocity / velocity_sq;
    const double t_closest = std::min(duration, std::max(0.0, raw_closest));
    consider(t_closest);

    if (boundary_rate < speed - 1e-12) {
      const double perpendicular_sq = std::max(
        0.0, position_sq - position_velocity * position_velocity / velocity_sq);
      const double denom = speed * std::sqrt(
        std::max(1e-24, velocity_sq - boundary_rate * boundary_rate));
      const double stationary_time = std::min(
        duration,
        std::max(0.0, raw_closest + boundary_rate * std::sqrt(perpendicular_sq) / denom));
      consider(stationary_time);
    }
  }
  return {best_clearance, best_time};
}

HumanRisk HumanAwareController1::invalidHumanRisk()
{
  HumanRisk risk;
  risk.valid = false;
  risk.active = true;
  risk.current_clearance = -kInf;
  risk.min_future_clearance = -kInf;
  risk.final_clearance = -kInf;
  risk.time_to_minimum = 0.0;
  risk.ttc = 0.0;
  risk.hard_ttc = 0.0;
  risk.hard_collision = true;
  return risk;
}

HumanRisk HumanAwareController1::trajectoryHumanRisk(
  const std::vector<Point2D> & trajectory, double robot_x, double robot_y) const
{
  thesis_msgs::msg::TrackedHumans::SharedPtr msg;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(humans_mutex_);
    msg = latest_humans_msg_;
    stamp = latest_humans_stamp_;
  }

  HumanRisk inactive;
  inactive.valid = true;
  inactive.active = false;
  inactive.hard_collision = false;
  // current_clearance / min_future_clearance / ... default to +inf in the struct.

  if (!predictive_humans_enabled_ || !msg || msg->humans.empty()) {
    return inactive;
  }

  const double state_age = (clock_->now() - stamp).seconds();
  // A corrupted message age (NaN or negative) is a data-integrity error and
  // vetoes every trajectory; a merely stale message is handled below.
  if (!std::isfinite(state_age) || state_age < 0.0) {
    return invalidHumanRisk();
  }
  if (state_age > tracked_humans_timeout_) {
    // Stale data. When require_tracked_humans is true this case is already
    // stopped at the top of computeVelocityCommands().
    return inactive;
  }

  if (!std::isfinite(robot_x) || !std::isfinite(robot_y)) {
    // Invalid robot pose (TF / AMCL glitch): veto every trajectory.
    return invalidHumanRisk();
  }

  std::vector<Point2D> robot_points;
  robot_points.reserve(trajectory.size() + 1);
  robot_points.push_back({robot_x, robot_y});
  for (const auto & p : trajectory) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return invalidHumanRisk();
    }
    robot_points.push_back(p);
  }

  const double total_duration = static_cast<double>(robot_points.size() - 1) * sim_dt_;
  const double r_hard = rHard();
  const double r_social = rSocial();
  const double soft_band_width = r_social - r_hard;

  std::vector<HumanTrackRisk> per_human;
  per_human.reserve(msg->humans.size());

  // Duplicate track ids indicate a tracker bug: veto instead of treating them
  // as two real people.
  std::unordered_set<uint32_t> seen_ids;
  for (const auto & human : msg->humans) {
    if (!seen_ids.insert(human.id).second) {
      return invalidHumanRisk();
    }
    const double human_x = human.x, human_y = human.y;
    const double human_vx = human.vx, human_vy = human.vy;
    const double observation_age = human.observation_age;
    const double cxx = human.covariance_xx, cxy = human.covariance_xy, cyy = human.covariance_yy;
    if (!std::isfinite(human_x) || !std::isfinite(human_y) || !std::isfinite(human_vx) ||
      !std::isfinite(human_vy) || !std::isfinite(observation_age) || !std::isfinite(cxx) ||
      !std::isfinite(cxy) || !std::isfinite(cyy))
    {
      return invalidHumanRisk();
    }
    const double cov_det = cxx * cyy - cxy * cxy;
    if (cxx < 0.0 || cyy < 0.0 || cov_det < -1e-9) {
      return invalidHumanRisk();
    }

    const double cov_disc = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
    const double largest_variance = std::max(0.0, 0.5 * (cxx + cyy + cov_disc));
    const double covariance_uncertainty = std::min(
      human_covariance_sigma_cap_, std::sqrt(human_covariance_gain_ * largest_variance));
    // Uncertainty growth. A constant rate assumes the person keeps walking
    // straight; a person walking at 0.7 m/s who turns around can end up far from
    // the constant-velocity prediction. Adding human_reversal_factor * |v| grows
    // the disc with the person's own speed (1.0 covers stopping up to twice the
    // predicted displacement; 0.0 keeps the constant-rate model).
    const double human_speed = std::hypot(human_vx, human_vy);
    double uncertainty_rate = human.is_moving ?
      human_uncertainty_rate_ : human_stationary_uncertainty_rate_;
    if (human_reversal_factor_ > 0.0 && std::isfinite(human_speed)) {
      uncertainty_rate = std::max(
        uncertainty_rate, uncertainty_rate + human_reversal_factor_ * human_speed);
    }

    const double human_now_x = human_x + human_vx * state_age;
    const double human_now_y = human_y + human_vy * state_age;
    const double age_uncertainty = covariance_uncertainty + uncertainty_rate * state_age;

    const double current_rx = robot_x - human_now_x, current_ry = robot_y - human_now_y;
    const double current_distance = std::hypot(current_rx, current_ry);
    const double current_clearance = current_distance - age_uncertainty - r_hard;

    double min_clearance = current_clearance;
    double time_to_minimum = 0.0;
    double social_ttc = (current_distance <= r_social + age_uncertainty) ? 0.0 : kInf;
    double hard_ttc = (current_clearance <= 0.0) ? 0.0 : kInf;
    double initial_clearance_rate = 0.0;

    for (size_t index = 0; index + 1 < robot_points.size(); ++index) {
      const double segment_start = static_cast<double>(index) * sim_dt_;
      const double rsx = robot_points[index].x, rsy = robot_points[index].y;
      const double rex = robot_points[index + 1].x, rey = robot_points[index + 1].y;
      const double robot_vx = (rex - rsx) / sim_dt_, robot_vy = (rey - rsy) / sim_dt_;
      const double human_sx = human_now_x + human_vx * segment_start;
      const double human_sy = human_now_y + human_vy * segment_start;
      const double relative_x = rsx - human_sx, relative_y = rsy - human_sy;
      const double relative_vx = robot_vx - human_vx, relative_vy = robot_vy - human_vy;
      const double uncertainty_start = age_uncertainty + uncertainty_rate * segment_start;

      if (index == 0) {
        const double separation = std::hypot(relative_x, relative_y);
        if (separation <= 1e-12) {
          initial_clearance_rate = -kInf;
        } else {
          initial_clearance_rate =
            (relative_x * relative_vx + relative_y * relative_vy) / separation -
            uncertainty_rate;
        }
      }

      auto [segment_min, minimum_offset] = segmentMinimumClearance(
        relative_x, relative_y, relative_vx, relative_vy, sim_dt_,
        r_hard + uncertainty_start, uncertainty_rate);
      if (segment_min < min_clearance) {
        min_clearance = segment_min;
        time_to_minimum = segment_start + minimum_offset;
      }

      if (!std::isfinite(social_ttc) && segment_min <= soft_band_width + 1e-12) {
        const double segment_social_ttc = segmentBoundaryTtc(
          relative_x, relative_y, relative_vx, relative_vy, sim_dt_,
          r_social + uncertainty_start, uncertainty_rate);
        if (std::isfinite(segment_social_ttc)) {
          social_ttc = segment_start + segment_social_ttc;
        }
      }
      if (!std::isfinite(hard_ttc) && segment_min <= 1e-12) {
        const double segment_hard_ttc = segmentBoundaryTtc(
          relative_x, relative_y, relative_vx, relative_vy, sim_dt_,
          r_hard + uncertainty_start, uncertainty_rate);
        if (std::isfinite(segment_hard_ttc)) {
          hard_ttc = segment_start + segment_hard_ttc;
        }
      }
    }

    const double human_final_x = human_now_x + human_vx * total_duration;
    const double human_final_y = human_now_y + human_vy * total_duration;
    const double final_uncertainty = age_uncertainty + uncertainty_rate * total_duration;
    const double final_clearance = std::hypot(
      robot_points.back().x - human_final_x, robot_points.back().y - human_final_y) -
      final_uncertainty - r_hard;

    HumanTrackRisk track_risk;
    track_risk.track_id = human.id;
    track_risk.current_clearance = current_clearance;
    track_risk.min_future_clearance = min_clearance;
    track_risk.final_clearance = final_clearance;
    track_risk.time_to_minimum = time_to_minimum;
    track_risk.ttc = social_ttc;
    track_risk.hard_ttc = hard_ttc;
    track_risk.initial_clearance_rate = initial_clearance_rate;
    track_risk.hard_collision = std::isfinite(hard_ttc);
    per_human.push_back(track_risk);
  }

  if (per_human.empty()) {
    return inactive;
  }

  const auto minimum_track = std::min_element(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & a, const HumanTrackRisk & b) {
      return a.min_future_clearance < b.min_future_clearance;
    });

  HumanRisk result;
  result.valid = true;
  result.active = true;
  result.per_human = per_human;
  result.current_clearance = std::min_element(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & a, const HumanTrackRisk & b) {
      return a.current_clearance < b.current_clearance;
    })->current_clearance;
  result.min_future_clearance = minimum_track->min_future_clearance;
  result.final_clearance = std::min_element(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & a, const HumanTrackRisk & b) {
      return a.final_clearance < b.final_clearance;
    })->final_clearance;
  result.time_to_minimum = minimum_track->time_to_minimum;
  result.ttc = std::min_element(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & a, const HumanTrackRisk & b) {return a.ttc < b.ttc;})->ttc;
  result.hard_ttc = std::min_element(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & a, const HumanTrackRisk & b) {
      return a.hard_ttc < b.hard_ttc;
    })->hard_ttc;
  result.hard_collision = std::any_of(
    per_human.begin(), per_human.end(),
    [](const HumanTrackRisk & r) {return r.hard_collision;});
  return result;
}

void HumanAwareController1::updateHumanIntents(
  const thesis_msgs::msg::TrackedHumans & msg, int64_t now_ns)
{
  for (const auto & human : msg.humans) {
    const double speed = std::hypot(human.vx, human.vy);
    if (!human.is_moving || speed < intent_min_speed_ || !std::isfinite(speed)) {
      // A standing person has no walking direction. Drop the intent so that
      // confidence restarts from zero when they move again.
      human_intents_.erase(human.id);
      continue;
    }
    const double dir_x = human.vx / speed;
    const double dir_y = human.vy / speed;
    auto it = human_intents_.find(human.id);
    if (it == human_intents_.end()) {
      human_intents_[human.id] = HumanIntent{dir_x, dir_y, 0.0, now_ns};
      continue;
    }
    HumanIntent & intent = it->second;
    const double dot = intent.dir_x * dir_x + intent.dir_y * dir_y;
    if (dot < 0.0) {
      // Reversal: follow the new direction immediately but reset confidence to 0,
      // passing in front of this person is not allowed until it recovers.
      intent.dir_x = dir_x;
      intent.dir_y = dir_y;
      intent.confidence = 0.0;
    } else {
      const double smoothed_x =
        intent_ema_alpha_ * dir_x + (1.0 - intent_ema_alpha_) * intent.dir_x;
      const double smoothed_y =
        intent_ema_alpha_ * dir_y + (1.0 - intent_ema_alpha_) * intent.dir_y;
      const double norm = std::hypot(smoothed_x, smoothed_y);
      if (norm > 1e-9) {
        intent.dir_x = smoothed_x / norm;
        intent.dir_y = smoothed_y / norm;
      }
      intent.confidence = std::min(1.0, intent.confidence + intent_ema_alpha_);
    }
    intent.last_seen_ns = now_ns;
  }

  for (auto it = human_intents_.begin(); it != human_intents_.end(); ) {
    const double age = static_cast<double>(now_ns - it->second.last_seen_ns) / 1e9;
    it = (age > intent_ttl_ || age < 0.0) ? human_intents_.erase(it) : std::next(it);
  }
}

double HumanAwareController1::crossingGap(
  const std::vector<Point2D> & trajectory, double & confidence_out) const
{
  confidence_out = 1.0;
  double worst_gap = kInf;

  thesis_msgs::msg::TrackedHumans::SharedPtr msg;
  double state_age = 0.0;
  {
    std::lock_guard<std::mutex> lock(humans_mutex_);
    msg = latest_humans_msg_;
    if (msg) {
      state_age = (clock_->now() - latest_humans_stamp_).seconds();
    }
  }
  if (!msg || !std::isfinite(state_age) || state_age < 0.0 ||
    state_age > tracked_humans_timeout_)
  {
    return kInf;
  }

  for (const auto & human : msg->humans) {
    const auto it = human_intents_.find(human.id);
    if (it == human_intents_.end()) {
      continue;   // standing or direction unknown: left to the clearance terms
    }
    const double speed = std::hypot(human.vx, human.vy);
    if (speed < intent_min_speed_) {
      continue;
    }
    // Compensate for message age, as in trajectoryHumanRisk().
    const double hx = human.x + human.vx * state_age;
    const double hy = human.y + human.vy * state_age;
    const double ux = it->second.dir_x;
    const double uy = it->second.dir_y;

    // Point of the robot trajectory closest to the person's walking ray, i.e.
    // where the two paths come closest.
    double best_distance = kInf;
    double t_robot = 0.0;
    double along_human = 0.0;
    for (size_t index = 0; index < trajectory.size(); ++index) {
      const double px = trajectory[index].x - hx;
      const double py = trajectory[index].y - hy;
      const double along = px * ux + py * uy;      // along the person's walking direction
      if (along < 0.0) {
        continue;   // behind the person: they never reach it
      }
      const double perpendicular = std::abs(-px * uy + py * ux);
      if (perpendicular < best_distance) {
        best_distance = perpendicular;
        t_robot = static_cast<double>(index + 1) * sim_dt_;
        along_human = along;
      }
    }
    if (!std::isfinite(best_distance)) {
      continue;
    }
    // Only consider real encounters; farther than this the paths do not interact.
    const double corridor = rSocial() + human_radius_;
    if (best_distance > corridor) {
      continue;
    }

    const double t_human = along_human / speed;
    const double gap = t_human - t_robot;
    if (std::abs(gap) < std::abs(worst_gap)) {
      worst_gap = gap;
      confidence_out = it->second.confidence;
    }
  }
  return worst_gap;
}

double HumanAwareController1::yieldCost(
  const std::vector<Point2D> & trajectory, double robot_x, double robot_y) const
{
  if (!yield_enabled_ || trajectory.empty()) {
    return 0.0;
  }

  thesis_msgs::msg::TrackedHumans::SharedPtr msg;
  double state_age = 0.0;
  {
    std::lock_guard<std::mutex> lock(humans_mutex_);
    msg = latest_humans_msg_;
    if (msg) {
      state_age = (clock_->now() - latest_humans_stamp_).seconds();
    }
  }
  if (!msg || !std::isfinite(state_age) || state_age < 0.0 ||
    state_age > tracked_humans_timeout_)
  {
    return 0.0;
  }

  double worst = 0.0;
  for (const auto & human : msg->humans) {
    const auto it = human_intents_.find(human.id);
    if (it == human_intents_.end()) {
      continue;                       // standing: no walking path to yield
    }
    const double speed = std::hypot(human.vx, human.vy);
    if (speed < intent_min_speed_) {
      continue;
    }
    const double hx = human.x + human.vx * state_age;
    const double hy = human.y + human.vy * state_age;
    const double ux = it->second.dir_x;
    const double uy = it->second.dir_y;

    // Robot position relative to the person: along and across their walking direction.
    const double px = robot_x - hx;
    const double py = robot_y - hy;
    const double along = px * ux + py * uy;
    if (along <= 0.0) {
      continue;                       // robot behind the person: walking away
    }
    const double t_arrive = along / speed;
    if (t_arrive > yield_trigger_time_) {
      continue;                       // arrives later than the trigger time
    }
    const double d_now = std::abs(-px * uy + py * ux);
    if (d_now >= yield_clearance_) {
      continue;                       // already outside their walking path
    }

    // How far this trajectory moves the robot away from the person's walking path.
    const auto & end = trajectory.back();
    const double qx = end.x - hx;
    const double qy = end.y - hy;
    const double d_end = std::abs(-qx * uy + qy * ux);

    // Remaining shortfall. Standing still keeps the cost high; driving forward or
    // backward are both fine as long as the robot moves away from the path.
    const double shortfall = std::max(0.0, yield_clearance_ - d_end) / yield_clearance_;
    worst = std::max(worst, shortfall);
  }
  return worst;
}

double HumanAwareController1::passingCost(double gap, double confidence) const
{
  if (!passing_enabled_ || !std::isfinite(gap)) {
    return 0.0;
  }
  // Direction uncertain (e.g. just reversed): widen the forbidden time gap.
  const double widen = confidence >= intent_confidence_min_ ? 1.0 : 1.5;
  const double ahead = pass_ahead_margin_ * widen;
  const double behind = pass_behind_margin_ * widen;
  if (gap >= ahead || gap <= -behind) {
    return 0.0;      // wide enough: pass ahead or behind
  }
  const double margin = gap >= 0.0 ? ahead : behind;
  if (margin <= 1e-9) {
    return 0.0;
  }
  return 1.0 - std::abs(gap) / margin;   // 1.0 when both arrive at the same time
}

double HumanAwareController1::humanProximityCost(double clearance) const
{
  if (clearance == kInf) {
    return 0.0;
  }
  if (clearance == -kInf) {
    return 1.0;
  }
  const double span = rSocial() - rHard();
  if (span <= 1e-9) {
    return 0.0;
  }
  return std::clamp((span - clearance) / span, 0.0, 1.0);
}

double HumanAwareController1::humanTtcCost(double ttc) const
{
  if (ttc == kInf) {
    return 0.0;
  }
  if (!std::isfinite(ttc) || ttc <= 0.0) {
    return 1.0;
  }
  return std::clamp(1.0 - ttc / human_prediction_horizon_, 0.0, 1.0);
}

bool HumanAwareController1::updateEmergencyState(const HumanRisk & standing_risk, int64_t now_ns)
{
  const bool threat = !standing_risk.valid ||
    (standing_risk.active &&
    (standing_risk.current_clearance <= 0.0 || standing_risk.ttc <= emergency_ttc_enter_));
  if (threat) {
    emergency_active_ = true;
    emergency_safe_since_set_ = false;
    return true;
  }
  if (!emergency_active_) {
    return false;
  }

  const bool safe = standing_risk.valid &&
    (!standing_risk.active ||
    (standing_risk.current_clearance > 0.0 && standing_risk.ttc >= emergency_ttc_exit_));
  if (!safe) {
    emergency_safe_since_set_ = false;
    return true;
  }
  if (emergency_exit_hold_ <= 0.0) {
    emergency_active_ = false;
    emergency_safe_since_set_ = false;
    return false;
  }
  if (now_ns <= 0) {
    return true;
  }
  if (!emergency_safe_since_set_ || now_ns < emergency_safe_since_ns_) {
    emergency_safe_since_ns_ = now_ns;
    emergency_safe_since_set_ = true;
    return true;
  }
  const double held_seconds = static_cast<double>(now_ns - emergency_safe_since_ns_) / 1e9;
  if (held_seconds >= emergency_exit_hold_) {
    emergency_active_ = false;
    emergency_safe_since_set_ = false;
  }
  return emergency_active_;
}

std::vector<double> HumanAwareController1::linearVelocitySamples() const
{
  const double min_v = min_linear_vel_;
  const double max_v = std::max(min_linear_vel_, max_linear_vel_);
  const int samples = std::max(1, linear_samples_);
  std::vector<double> result;

  auto linspace = [](double a, double b, int n) {
      std::vector<double> v;
      if (n <= 1) {
        v.push_back(a);
        return v;
      }
      v.reserve(n);
      for (int i = 0; i < n; ++i) {
        v.push_back(a + (b - a) * static_cast<double>(i) / static_cast<double>(n - 1));
      }
      return v;
    };

  if (min_v >= 0.0) {
    result = linspace(min_v, max_v, samples);
    if (min_v > 0.0) {
      result.insert(result.begin(), 0.0);
    }
    return result;
  }
  if (max_v <= 0.0) {
    result = linspace(min_v, max_v, samples);
    if (max_v < 0.0) {
      result.push_back(0.0);
    }
    return result;
  }

  const double span = max_v - min_v;
  const int negative_count = std::max(
    1, std::min(samples - 1, static_cast<int>(std::lround(samples * (-min_v) / span))));
  const int positive_count = samples + 1 - negative_count;
  const auto negative = linspace(min_v, 0.0, negative_count);
  const auto positive = linspace(0.0, max_v, positive_count);
  result = negative;
  result.insert(result.end(), positive.begin() + 1, positive.end());
  return result;
}

double HumanAwareController1::candidateScore(
  const Candidate & candidate, double target_x, double target_y) const
{
  const auto & end = candidate.trajectory.back();
  const double goal_distance = std::hypot(end.x - target_x, end.y - target_y);
  const double path_cost = pathAlignmentCost(candidate.trajectory);
  const double target_heading = std::atan2(target_y - end.y, target_x - end.x);
  const double heading_error = std::abs(
    std::atan2(
      std::sin(target_heading - candidate.end_yaw), std::cos(target_heading - candidate.end_yaw)));
  const double continuity_cost = std::abs(candidate.linear_velocity - prev_linear_velocity_) +
    std::abs(candidate.angular_velocity - prev_angular_velocity_);
  const double proximity_cost = humanProximityCost(candidate.human_risk.min_future_clearance);
  const double ttc_cost = humanTtcCost(candidate.human_risk.ttc);

  return -w_goal_ * goal_distance + w_speed_ * candidate.linear_velocity -
         w_path_ * path_cost - w_social_ * candidate.social_cost -
         w_human_proximity_ * proximity_cost - w_human_ttc_ * ttc_cost -
         w_heading_ * heading_error - w_smooth_ * continuity_cost -
         w_passing_ * candidate.passing_cost - w_yield_ * candidate.yield_cost;
}

std::tuple<double, double, double, double, double, double> HumanAwareController1::safetyKey(
  const Candidate & candidate, double normal_score) const
{
  const double ttc = candidate.human_risk.ttc;
  const double ttc_rank = std::isfinite(ttc) ? ttc : human_prediction_horizon_ + 1.0;
  return {
    candidate.human_risk.min_future_clearance,
    candidate.human_risk.final_clearance,
    ttc_rank,
    normal_score,
    -std::abs(candidate.angular_velocity),
    -std::abs(candidate.linear_velocity)};
}

bool HumanAwareController1::isRecoveryCandidate(
  const HumanRisk & candidate_risk, const HumanRisk & standing_risk) const
{
  if (!candidate_risk.valid || !standing_risk.valid) {
    return false;
  }
  bool found_inside_human = false;
  for (const auto & standing_human : standing_risk.per_human) {
    const auto it = std::find_if(
      candidate_risk.per_human.begin(), candidate_risk.per_human.end(),
      [&](const HumanTrackRisk & r) {return r.track_id == standing_human.track_id;});
    if (it == candidate_risk.per_human.end()) {
      return false;
    }
    const HumanTrackRisk & candidate_human = *it;

    if (standing_human.current_clearance > 0.0) {
      if (candidate_human.hard_collision) {
        return false;
      }
      continue;
    }
    found_inside_human = true;
    if (candidate_human.min_future_clearance < standing_human.min_future_clearance - 1e-9) {
      return false;
    }
    if (candidate_human.final_clearance <
      standing_human.final_clearance + emergency_clearance_improvement_)
    {
      return false;
    }
    if (candidate_human.final_clearance <
      standing_human.current_clearance + emergency_clearance_improvement_)
    {
      return false;
    }
    if (candidate_human.initial_clearance_rate <= standing_human.initial_clearance_rate + 1e-9) {
      return false;
    }
  }
  return found_inside_human;
}

bool HumanAwareController1::goalApproachCommand(
  double robot_x, double robot_y, double robot_yaw,
  geometry_msgs::msg::TwistStamped & cmd) const
{
  // Goal approach: the robot is at the goal position and only has to rotate to
  // the goal heading so that the stateful goal checker can succeed.
  if (!goal_yaw_valid_ || path_costmap_frame_.empty()) {
    return false;
  }
  const Point2D & goal_point = path_costmap_frame_.back();
  const double goal_distance = std::hypot(goal_point.x - robot_x, goal_point.y - robot_y);
  if (goal_distance > goal_approach_distance_) {
    return false;
  }

  double yaw_error = goal_yaw_costmap_frame_ - robot_yaw;
  while (yaw_error > M_PI) {yaw_error -= 2.0 * M_PI;}
  while (yaw_error < -M_PI) {yaw_error += 2.0 * M_PI;}

  if (std::abs(yaw_error) <= goal_yaw_deadband_) {
    return true;  // v = w = 0: the goal checker succeeds this tick
  }

  double angular = std::clamp(
    goal_yaw_gain_ * yaw_error, -max_angular_vel_, max_angular_vel_);
  // Very small commands do not overcome wheel static friction; enforce a
  // minimum rotation speed so the robot cannot stall here.
  if (std::abs(angular) < goal_min_rotate_speed_) {
    angular = std::copysign(goal_min_rotate_speed_, angular);
  }
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = angular;
  return true;
}

geometry_msgs::msg::TwistStamped HumanAwareController1::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();

  if (!transformPlanToCostmapFrame() || path_costmap_frame_.empty()) {
    return cmd;  // stop (v = w = 0): no valid global plan
  }

  // require_tracked_humans: stop when tracked-human data is missing or stale
  // instead of driving as if nobody were around.
  if (require_tracked_humans_ && predictive_humans_enabled_) {
    thesis_msgs::msg::TrackedHumans::SharedPtr humans_msg;
    rclcpp::Time humans_stamp;
    {
      std::lock_guard<std::mutex> lock(humans_mutex_);
      humans_msg = latest_humans_msg_;
      humans_stamp = latest_humans_stamp_;
    }
    // Check the age of the message itself. An empty humans list is valid (nobody
    // in view) and must not be treated as missing data.
    bool humans_stale = true;
    if (humans_msg) {
      const double state_age = (clock_->now() - humans_stamp).seconds();
      humans_stale = !std::isfinite(state_age) || state_age<0.0 ||
          state_age> tracked_humans_timeout_;
    }
    if (humans_stale) {
      const int64_t now_ns = static_cast<int64_t>(clock_->now().nanoseconds());
      if (now_ns - last_stale_humans_warn_ns_ > 5'000'000'000LL) {
        RCLCPP_WARN(
          logger_,
          "HumanAwareController1 (%s): tracked-human data missing/stale "
          "(require_tracked_humans=true) -- stopping safely (v=w=0)",
          plugin_name_.c_str());
        last_stale_humans_warn_ns_ = now_ns;
      }
      return cmd;  // stop (v=w=0)
    }
  }

  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(pose.pose.orientation);

  const int nearest_index = nearestIndex(path_costmap_frame_, robot_x, robot_y);
  const Point2D target = pickLookaheadTarget(path_costmap_frame_, nearest_index, lookahead_dist_);

  double standing_end_yaw = 0.0;
  simulate(
    robot_x, robot_y, robot_yaw, 0.0, 0.0, human_prediction_horizon_,
    standing_trajectory_scratch_, standing_end_yaw);
  {
    thesis_msgs::msg::TrackedHumans::SharedPtr intents_msg;
    {
      std::lock_guard<std::mutex> lock(humans_mutex_);
      intents_msg = latest_humans_msg_;
    }
    if (intents_msg) {
      updateHumanIntents(*intents_msg, clock_->now().nanoseconds());
    }
  }

  const HumanRisk standing_risk = trajectoryHumanRisk(
    standing_trajectory_scratch_, robot_x, robot_y);
  const bool emergency_active = updateEmergencyState(
    standing_risk, static_cast<int64_t>(clock_->now().nanoseconds()));

  // Goal approach runs after the emergency check: avoiding a person has priority.
  if (!emergency_active && goalApproachCommand(robot_x, robot_y, robot_yaw, cmd)) {
    return cmd;
  }

  const auto linear_velocities = linearVelocitySamples();
  std::vector<double> angular_velocities;
  angular_velocities.reserve(angular_samples_);
  for (int i = 0; i < angular_samples_; ++i) {
    const double t = angular_samples_ > 1 ?
      static_cast<double>(i) / static_cast<double>(angular_samples_ - 1) : 0.0;
    angular_velocities.push_back(-max_angular_vel_ + 2.0 * max_angular_vel_ * t);
  }

  // resize(), not clear(): keeps the capacity of each Candidate::trajectory.
  candidate_pool_.resize(linear_velocities.size() * angular_velocities.size());

  {
    size_t idx = 0;
    for (const double v : linear_velocities) {
      for (const double w : angular_velocities) {
        Candidate & candidate = candidate_pool_[idx++];
        candidate.linear_velocity = v;
        candidate.angular_velocity = w;
        simulate(
          robot_x, robot_y, robot_yaw, v, w, sim_time_, candidate.trajectory,
          candidate.end_yaw);

        const auto [collision, social_cost] = trajectoryCostmapCost(candidate.trajectory);
        candidate.social_cost = social_cost;
        if (collision) {
          // candidate_pool_ is reused across ticks: mark the risk invalid explicitly so
          // that no stale human_risk from the previous tick survives in this slot.
          candidate.human_risk = invalidHumanRisk();
          continue;
        }

        double risk_end_yaw;
        simulate(
          robot_x, robot_y, robot_yaw, v, w, human_prediction_horizon_,
          risk_trajectory_scratch_, risk_end_yaw);
        candidate.human_risk = trajectoryHumanRisk(risk_trajectory_scratch_, robot_x, robot_y);
        // Use the prediction horizon (3 s), not the control horizon (1.5 s): a person
        // 3 m away walking at 0.7 m/s needs about 4 s to reach the crossing point.
        double passing_confidence = 1.0;
        const double gap = crossingGap(risk_trajectory_scratch_, passing_confidence);
        candidate.passing_cost = passingCost(gap, passing_confidence);
        candidate.yield_cost = yieldCost(candidate.trajectory, robot_x, robot_y);
      }
    }
  }

  std::vector<Candidate *> hard_safe_candidates;
  for (auto & c : candidate_pool_) {
    if (c.human_risk.valid && !c.human_risk.hard_collision) {
      hard_safe_candidates.push_back(&c);
    }
  }
  const bool current_inside_hard_boundary = standing_risk.valid && standing_risk.active &&
    standing_risk.current_clearance <= 0.0;

  const Candidate * chosen = nullptr;

  if (emergency_active) {
    std::vector<Candidate *> eligible;
    if (current_inside_hard_boundary) {
      for (auto & c : candidate_pool_) {
        if (isRecoveryCandidate(c.human_risk, standing_risk)) {
          eligible.push_back(&c);
        }
      }
    } else {
      eligible = hard_safe_candidates;
    }

    if (!eligible.empty()) {
      std::vector<std::pair<Candidate *, std::tuple<double, double, double, double, double,
        double>>> scored;
      scored.reserve(eligible.size());
      for (auto * c : eligible) {
        scored.emplace_back(c, safetyKey(*c, candidateScore(*c, target.x, target.y)));
      }
      auto best_it = std::max_element(
        scored.begin(), scored.end(),
        [](const auto & a, const auto & b) {return a.second < b.second;});
      Candidate * emergency_candidate = best_it->first;
      const auto best_key = best_it->second;

      const double prev_sign = std::abs(prev_linear_velocity_) > 1e-6 ?
        std::copysign(1.0, prev_linear_velocity_) : 0.0;
      if (prev_sign != 0.0) {
        std::vector<std::pair<Candidate *, std::tuple<double, double, double, double, double,
          double>>> same_sign;
        for (auto & item : scored) {
          if (std::abs(item.first->linear_velocity) > 1e-6 &&
            std::copysign(1.0, item.first->linear_velocity) == prev_sign)
          {
            same_sign.push_back(item);
          }
        }
        if (!same_sign.empty()) {
          auto same_best_it = std::max_element(
            same_sign.begin(), same_sign.end(),
            [](const auto & a, const auto & b) {return a.second < b.second;});
          if (std::get<0>(best_key) - std::get<0>(same_best_it->second) <=
            emergency_clearance_tie_margin_)
          {
            emergency_candidate = same_best_it->first;
          }
        }
      }
      chosen = emergency_candidate;
    }
  } else if (!hard_safe_candidates.empty()) {
    std::vector<Candidate *> translational, rotational;
    for (auto * c : hard_safe_candidates) {
      if (std::abs(c->linear_velocity) > 1e-4) {
        translational.push_back(c);
      } else if (std::abs(c->angular_velocity) > 1e-4) {
        rotational.push_back(c);
      }
    }

    const double target_heading = std::atan2(target.y - robot_y, target.x - robot_x);
    const double heading_error = std::atan2(
      std::sin(target_heading - robot_yaw), std::cos(target_heading - robot_yaw));

    std::vector<Candidate *> directed_rotations;
    for (auto * c : rotational) {
      if (c->angular_velocity * heading_error > 0.0) {
        directed_rotations.push_back(c);
      }
    }

    // Hysteresis: enter rotate-in-place above rotate_in_place_angle_, leave it only
    // below rotate_heading_deadband_, so the pool does not switch every tick.
    if (!heading_rotation_mode_) {
      if (std::abs(heading_error) >= rotate_in_place_angle_) {
        heading_rotation_mode_ = true;
      }
    } else if (std::abs(heading_error) < rotate_heading_deadband_) {
      heading_rotation_mode_ = false;
    }

    std::vector<Candidate *> pool;
    if (heading_rotation_mode_) {
      pool = directed_rotations;
      pool.insert(pool.end(), translational.begin(), translational.end());
    } else if (!translational.empty()) {
      pool = translational;
    } else if (std::abs(heading_error) > rotate_heading_deadband_ && !directed_rotations.empty()) {
      pool = directed_rotations;
    }

    if (!pool.empty()) {
      std::vector<std::pair<Candidate *, double>> scored;
      scored.reserve(pool.size());
      for (auto * c : pool) {
        scored.emplace_back(c, candidateScore(*c, target.x, target.y));
      }
      auto best_it = std::max_element(
        scored.begin(), scored.end(),
        [](const auto & a, const auto & b) {return a.second < b.second;});
      Candidate * best = best_it->first;
      const double best_score = best_it->second;

      bool exact_match_found = false;
      for (auto & item : scored) {
        if (item.first == best ||
          std::abs(item.first->linear_velocity - prev_linear_velocity_) >= 1e-6 ||
          std::abs(item.first->angular_velocity - prev_angular_velocity_) >= 1e-6)
        {
          continue;
        }
        exact_match_found = true;
        if (best_score <= item.second + switch_score_margin_) {
          best = item.first;
        }
        break;
      }

      if (!exact_match_found) {
        const double prev_sign = std::abs(prev_linear_velocity_) > 1e-6 ?
          std::copysign(1.0, prev_linear_velocity_) : 0.0;
        if (prev_sign != 0.0) {
          std::vector<std::pair<Candidate *, double>> same_sign;
          for (auto & item : scored) {
            if (std::abs(item.first->linear_velocity) > 1e-6 &&
              std::copysign(1.0, item.first->linear_velocity) == prev_sign)
            {
              same_sign.push_back(item);
            }
          }
          if (!same_sign.empty()) {
            auto same_best_it = std::max_element(
              same_sign.begin(), same_sign.end(),
              [](const auto & a, const auto & b) {return a.second < b.second;});
            if (best_score <= same_best_it->second + switch_score_margin_) {
              best = same_best_it->first;
            }
          }
        }
      }
      chosen = best;
    }
  }

  if (chosen == nullptr && stuck_escape_enabled_ && standing_risk.valid && standing_risk.active) {
    // Stuck: no candidate is acceptable while a person is close. Standing still
    // in a walking person's path is the worst option, so pick the candidate that
    // increases the clearance fastest and use it only if it is better than
    // standing still. Candidates that hit a static obstacle stay excluded.
    const Candidate * escape = nullptr;
    double best_min = standing_risk.min_future_clearance;
    double best_final = standing_risk.final_clearance;
    for (const auto & candidate : candidate_pool_) {
      if (!candidate.human_risk.valid || !candidate.human_risk.active) {
        continue;
      }
      const double candidate_min = candidate.human_risk.min_future_clearance;
      const double candidate_final = candidate.human_risk.final_clearance;
      if (candidate_min > best_min + 1e-6 ||
        (candidate_min > best_min - 1e-6 && candidate_final > best_final + 1e-6))
      {
        best_min = candidate_min;
        best_final = candidate_final;
        escape = &candidate;
      }
    }
    if (escape != nullptr) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "HumanAwareController1: stuck (no acceptable candidate) -- escaping with "
        "(v=%.2f, w=%.2f), clearance %.2f -> %.2f m instead of stopping",
        escape->linear_velocity, escape->angular_velocity,
        standing_risk.min_future_clearance, best_min);
      chosen = escape;
    }
  }

  if (chosen == nullptr) {
    // No safe candidate: stop (v = w = 0) without resetting the previous command,
    // this is an interruption rather than a steady state. Always log it, so a
    // stop can be told apart from a stop caused by the safety layer.
    std::size_t n_blocked_by_people = 0;
    std::size_t n_blocked_by_obstacles = 0;
    for (const auto & c : candidate_pool_) {
      if (!c.human_risk.valid) {
        n_blocked_by_obstacles += 1;
      } else if (c.human_risk.hard_collision) {
        n_blocked_by_people += 1;
      }
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "HumanAwareController1: STOP -- no acceptable candidate "
      "(%zu candidates: %zu blocked by people, %zu by static obstacles; current clearance "
      "%.2f m). stuck_escape_enabled=%s",
      candidate_pool_.size(), n_blocked_by_people, n_blocked_by_obstacles,
      standing_risk.valid ? standing_risk.current_clearance : std::nan(""),
      stuck_escape_enabled_ ? "true" : "false");
    return cmd;
  }

  if (std::abs(chosen->linear_velocity) > 1e-4 || std::abs(chosen->angular_velocity) > 1e-4) {
    prev_linear_velocity_ = chosen->linear_velocity;
    prev_angular_velocity_ = chosen->angular_velocity;
  }

  cmd.twist.linear.x = chosen->linear_velocity;
  cmd.twist.angular.z = chosen->angular_velocity;

  if (local_path_pub_ && local_path_pub_->is_activated()) {
    nav_msgs::msg::Path selected_path;
    selected_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    selected_path.header.stamp = cmd.header.stamp;
    for (const auto & p : chosen->trajectory) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = selected_path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      selected_path.poses.push_back(ps);
    }
    local_path_pub_->publish(selected_path);
  }

  return cmd;
}

}  // namespace social_nav_controller

PLUGINLIB_EXPORT_CLASS(social_nav_controller::HumanAwareController1, nav2_core::Controller)
