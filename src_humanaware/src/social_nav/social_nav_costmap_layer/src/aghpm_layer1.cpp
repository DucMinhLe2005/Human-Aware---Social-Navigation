#include "social_nav_costmap_layer/aghpm_layer1.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace social_nav_costmap_layer
{

namespace
{

// Asymmetric Gaussian social cost.
double asymmetricGaussian(
  double x, double y, double x0, double y0, double theta,
  double sigma_front, double sigma_back, double sigma_side)
{
  sigma_front = std::max(sigma_front, 1e-6);
  sigma_back = std::max(sigma_back, 1e-6);
  sigma_side = std::max(sigma_side, 1e-6);

  const double dx = x - x0;
  const double dy = y - y0;
  const double forward = dx * std::cos(theta) + dy * std::sin(theta);
  const double side = -dx * std::sin(theta) + dy * std::cos(theta);
  const double sigma_forward = (forward >= 0.0) ? sigma_front : sigma_back;

  return std::exp(
    -0.5 * (
      (forward * forward) / (sigma_forward * sigma_forward) +
      (side * side) / (sigma_side * sigma_side)));
}

// Position variances along / across the walking direction.
void directionalPositionVariances(
  double covariance_xx, double covariance_xy, double covariance_yy,
  double yaw, double & variance_forward, double & variance_side)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  variance_forward = cos_yaw * cos_yaw * covariance_xx +
    2.0 * cos_yaw * sin_yaw * covariance_xy +
    sin_yaw * sin_yaw * covariance_yy;

  variance_side = sin_yaw * sin_yaw * covariance_xx -
    2.0 * sin_yaw * cos_yaw * covariance_xy +
    cos_yaw * cos_yaw * covariance_yy;

  variance_forward = std::max(0.0, variance_forward);
  variance_side = std::max(0.0, variance_side);
}

}  // namespace

void AghpmLayer1::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"AghpmLayer1: failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "tracked_humans_topic", rclcpp::ParameterValue(std::string("/planning/tracked_humans")));
  declareParameter("max_human_distance", rclcpp::ParameterValue(10.0));
  declareParameter("max_cost_value", rclcpp::ParameterValue(220.0));

  declareParameter("sigma_front", rclcpp::ParameterValue(0.60));
  declareParameter("sigma_back", rclcpp::ParameterValue(0.30));
  declareParameter("sigma_side", rclcpp::ParameterValue(0.45));
  declareParameter("human_cost_radius", rclcpp::ParameterValue(2.0));

  declareParameter("velocity_social_enabled", rclcpp::ParameterValue(true));
  declareParameter("velocity_sigma_front_gain", rclcpp::ParameterValue(0.10));
  declareParameter("moving_sigma_front_extra", rclcpp::ParameterValue(0.0));
  declareParameter("velocity_sigma_side_gain", rclcpp::ParameterValue(0.03));
  declareParameter("velocity_sigma_age_gain", rclcpp::ParameterValue(0.05));
  declareParameter("velocity_cost_radius_gain", rclcpp::ParameterValue(0.10));

  declareParameter("social_covariance_enabled", rclcpp::ParameterValue(true));
  declareParameter("social_covariance_gain", rclcpp::ParameterValue(1.0));
  declareParameter("social_covariance_sigma_cap", rclcpp::ParameterValue(0.30));

  declareParameter("velocity_future_enabled", rclcpp::ParameterValue(true));
  declareParameter("velocity_future_horizon", rclcpp::ParameterValue(1.0));
  declareParameter("velocity_future_step", rclcpp::ParameterValue(0.2));
  declareParameter("velocity_future_cost_max", rclcpp::ParameterValue(0.45));
  declareParameter("velocity_future_decay_tau", rclcpp::ParameterValue(1.5));
  declareParameter("velocity_future_sigma_long", rclcpp::ParameterValue(0.30));
  declareParameter("velocity_future_sigma_side", rclcpp::ParameterValue(0.25));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "tracked_humans_topic", tracked_humans_topic_);
  node->get_parameter(name_ + "." + "max_human_distance", max_human_distance_);
  node->get_parameter(name_ + "." + "max_cost_value", max_cost_value_);

  node->get_parameter(name_ + "." + "sigma_front", sigma_front_);
  node->get_parameter(name_ + "." + "sigma_back", sigma_back_);
  node->get_parameter(name_ + "." + "sigma_side", sigma_side_);
  node->get_parameter(name_ + "." + "human_cost_radius", human_cost_radius_);

  node->get_parameter(name_ + "." + "velocity_social_enabled", velocity_social_enabled_);
  node->get_parameter(name_ + "." + "velocity_sigma_front_gain", velocity_sigma_front_gain_);
  node->get_parameter(name_ + "." + "moving_sigma_front_extra", moving_sigma_front_extra_);
  node->get_parameter(name_ + "." + "velocity_sigma_side_gain", velocity_sigma_side_gain_);
  node->get_parameter(name_ + "." + "velocity_sigma_age_gain", velocity_sigma_age_gain_);
  node->get_parameter(name_ + "." + "velocity_cost_radius_gain", velocity_cost_radius_gain_);

  node->get_parameter(name_ + "." + "social_covariance_enabled", social_covariance_enabled_);
  node->get_parameter(name_ + "." + "social_covariance_gain", social_covariance_gain_);
  node->get_parameter(
    name_ + "." + "social_covariance_sigma_cap", social_covariance_sigma_cap_);

  node->get_parameter(name_ + "." + "velocity_future_enabled", velocity_future_enabled_);
  node->get_parameter(name_ + "." + "velocity_future_horizon", velocity_future_horizon_);
  node->get_parameter(name_ + "." + "velocity_future_step", velocity_future_step_);
  node->get_parameter(name_ + "." + "velocity_future_cost_max", velocity_future_cost_max_);
  node->get_parameter(name_ + "." + "velocity_future_decay_tau", velocity_future_decay_tau_);
  node->get_parameter(name_ + "." + "velocity_future_sigma_long", velocity_future_sigma_long_);
  node->get_parameter(name_ + "." + "velocity_future_sigma_side", velocity_future_sigma_side_);

  // Safety: never let max_cost_value_ exceed the lethal thresholds, whatever the
  // YAML says -- the key invariant of this layer (see the header).
  max_cost_value_ = std::clamp(
    max_cost_value_,
    0.0,
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1));

  tracked_humans_sub_ = node->create_subscription<thesis_msgs::msg::TrackedHumans>(
    tracked_humans_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&AghpmLayer1::trackedHumansCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    logger_,
    "AghpmLayer1 (%s) initialized: topic=%s, max_cost_value=%.1f",
    name_.c_str(), tracked_humans_topic_.c_str(), max_cost_value_);
}

void AghpmLayer1::trackedHumansCallback(const thesis_msgs::msg::TrackedHumans::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(humans_mutex_);
  latest_msg_ = msg;
}

void AghpmLayer1::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  resolved_humans_.clear();
  if (!enabled_) {
    return;
  }

  thesis_msgs::msg::TrackedHumans::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(humans_mutex_);
    msg = latest_msg_;
  }
  if (!msg) {
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  double frame_dx = 0.0, frame_dy = 0.0, frame_yaw = 0.0;
  bool need_transform = !msg->header.frame_id.empty() && msg->header.frame_id != costmap_frame;
  if (need_transform) {
    // Rare case: tracked humans are normally published in odom, the local
    // costmap's global frame. This is only a fallback if the frame changes.
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "AghpmLayer1: TrackedHumans frame '%s' != costmap frame '%s' but no TF buffer "
        "available; skipping humans this cycle",
        msg->header.frame_id.c_str(), costmap_frame.c_str());
      return;
    }
    try {
      auto transform = tf_->lookupTransform(
        costmap_frame, msg->header.frame_id, tf2::TimePointZero);
      frame_dx = transform.transform.translation.x;
      frame_dy = transform.transform.translation.y;
      const auto & q = transform.transform.rotation;
      frame_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "AghpmLayer1: cannot transform %s -> %s: %s",
        msg->header.frame_id.c_str(), costmap_frame.c_str(), e.what());
      return;
    }
  }
  const double cos_f = std::cos(frame_yaw);
  const double sin_f = std::sin(frame_yaw);

  for (const auto & human : msg->humans) {
    double hx = human.x;
    double hy = human.y;
    double hyaw = human.yaw;
    if (need_transform) {
      const double rx = hx, ry = hy;
      hx = frame_dx + cos_f * rx - sin_f * ry;
      hy = frame_dy + sin_f * rx + cos_f * ry;
      hyaw += frame_yaw;
    }

    const double distance = std::hypot(hx - robot_x, hy - robot_y);
    if (max_human_distance_ > 0.0 && distance > max_human_distance_) {
      continue;
    }

    // --- Per-human zone parameters ---
    double dyn_sigma_front = sigma_front_;
    double dyn_sigma_back = sigma_back_;
    double dyn_sigma_side = sigma_side_;
    double dyn_cost_radius = human_cost_radius_;

    if (human.is_moving) {
      dyn_sigma_front += moving_sigma_front_extra_;
    }

    if (velocity_social_enabled_ && human.is_moving) {
      const double speed = human.speed;
      const double observation_age = std::min(human.observation_age, 3.0);
      const double age_inflation = social_covariance_enabled_ ?
        0.0 :
        velocity_sigma_age_gain_ * observation_age;

      dyn_sigma_front += velocity_sigma_front_gain_ * speed + age_inflation;
      dyn_sigma_side += velocity_sigma_side_gain_ * speed + age_inflation;
      dyn_sigma_back += 0.5 * age_inflation;

      if (dyn_cost_radius > 0.0) {
        dyn_cost_radius += velocity_cost_radius_gain_ * speed + age_inflation;
      }
    }

    if (social_covariance_enabled_) {
      double variance_forward = 0.0, variance_side = 0.0;
      directionalPositionVariances(
        human.covariance_xx, human.covariance_xy, human.covariance_yy, hyaw,
        variance_forward, variance_side);

      const double cov_sigma_forward = std::min(
        social_covariance_sigma_cap_,
        std::sqrt(social_covariance_gain_ * variance_forward));
      const double cov_sigma_side = std::min(
        social_covariance_sigma_cap_,
        std::sqrt(social_covariance_gain_ * variance_side));

      dyn_sigma_front = std::hypot(dyn_sigma_front, cov_sigma_forward);
      dyn_sigma_back = std::hypot(dyn_sigma_back, cov_sigma_forward);
      dyn_sigma_side = std::hypot(dyn_sigma_side, cov_sigma_side);
    }

    ResolvedHuman resolved;
    resolved.x = hx;
    resolved.y = hy;
    resolved.yaw = hyaw;
    resolved.vx = human.vx;
    resolved.vy = human.vy;
    resolved.speed = human.speed;
    resolved.moving = human.is_moving;
    resolved.sigma_front = dyn_sigma_front;
    resolved.sigma_back = dyn_sigma_back;
    resolved.sigma_side = dyn_sigma_side;
    resolved.cost_radius = dyn_cost_radius;

    double bounding_radius = std::max(
      {dyn_cost_radius, dyn_sigma_front * 3.0, dyn_sigma_back * 3.0, dyn_sigma_side * 3.0});
    if (velocity_future_enabled_ && resolved.moving && velocity_future_horizon_ > 0.0) {
      const double future_extent = velocity_future_horizon_ * resolved.speed +
        2.5 * std::max(velocity_future_sigma_long_, velocity_future_sigma_side_);
      bounding_radius = std::max(bounding_radius, future_extent);
    }
    resolved.bounding_radius = bounding_radius;
    resolved_humans_.push_back(resolved);

    *min_x = std::min(*min_x, hx - bounding_radius);
    *min_y = std::min(*min_y, hy - bounding_radius);
    *max_x = std::max(*max_x, hx + bounding_radius);
    *max_y = std::max(*max_y, hy + bounding_radius);
  }
}

double AghpmLayer1::computeCostAt(double wx, double wy) const
{
  double best_cost = 0.0;

  for (const auto & human : resolved_humans_) {
    const double dist = std::hypot(wx - human.x, wy - human.y);
    if (dist <= human.cost_radius || human.cost_radius <= 0.0) {
      const double gaussian = asymmetricGaussian(
        wx, wy, human.x, human.y, human.yaw,
        human.sigma_front, human.sigma_back, human.sigma_side);
      if (human.cost_radius <= 0.0 || dist <= human.cost_radius) {
        best_cost = std::max(best_cost, gaussian);
      }
    }

    if (velocity_future_enabled_ && human.moving &&
      velocity_future_horizon_ > 0.0 && human.speed > 1e-6)
    {
      const int sample_count = std::max(
        1, static_cast<int>(std::ceil(velocity_future_horizon_ / velocity_future_step_)));
      const double cutoff_radius = 2.5 * std::max(
        velocity_future_sigma_long_, velocity_future_sigma_side_);

      for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
        const double t = std::min(sample_index * velocity_future_step_, velocity_future_horizon_);
        const double future_x = human.x + human.vx * t;
        const double future_y = human.y + human.vy * t;
        const double future_dist = std::hypot(wx - future_x, wy - future_y);
        if (future_dist > cutoff_radius) {
          continue;
        }
        const double amplitude = velocity_future_cost_max_ *
          std::exp(-t / velocity_future_decay_tau_);
        const double future_gaussian = amplitude * asymmetricGaussian(
          wx, wy, future_x, future_y, human.yaw,
          velocity_future_sigma_long_, velocity_future_sigma_long_,
          velocity_future_sigma_side_);
        best_cost = std::max(best_cost, future_gaussian);
      }
    }
  }

  return std::clamp(best_cost, 0.0, 1.0);
}

void AghpmLayer1::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || resolved_humans_.empty()) {
    return;
  }

  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  max_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      const double cost01 = computeCostAt(wx, wy);
      if (cost01 <= 0.0) {
        continue;
      }
      const auto cost_val = static_cast<unsigned char>(std::lround(cost01 * max_cost_value_));
      if (cost_val == 0) {
        continue;
      }
      const unsigned char old_cost = master_grid.getCost(i, j);
      if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(i, j, cost_val);
      } else if (cost_val > old_cost) {
        master_grid.setCost(i, j, cost_val);
      }
    }
  }
}

void AghpmLayer1::reset()
{
  std::lock_guard<std::mutex> lock(humans_mutex_);
  latest_msg_.reset();
  resolved_humans_.clear();
}

}  // namespace social_nav_costmap_layer

PLUGINLIB_EXPORT_CLASS(social_nav_costmap_layer::AghpmLayer1, nav2_costmap_2d::Layer)
