// AGHPM (Asymmetric Gaussian Human Proxemics Model) costmap layer.
//
// A nav2_costmap_2d Layer plugin that writes an asymmetric Gaussian social cost
// around each tracked human directly into the local costmap master grid. The
// Gaussian is longer in front of a person than behind, grows with walking speed
// and with the track covariance, and adds a soft predicted corridor along the
// walking direction.
//
// Design notes:
//  - The cost never reaches LETHAL_OBSTACLE (254) or INSCRIBED_INFLATED_OBSTACLE
//    (253); it is capped at max_cost_value (default 220). The planner therefore
//    never treats the area around a person as a permanent hard obstacle, which
//    would make it circle without progress. The hard safety radius is handled
//    by the controller, not by the costmap.
//  - Tracked humans are published in the odom frame, which is also the global
//    frame of the local costmap, so no TF lookup is normally needed; a
//    transform is applied only if the message frame differs.
//  - Group behaviour (merging the zones of people standing together) is not
//    modelled.
#ifndef SOCIAL_NAV_COSTMAP_LAYER__AGHPM_LAYER1_HPP_
#define SOCIAL_NAV_COSTMAP_LAYER__AGHPM_LAYER1_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "thesis_msgs/msg/tracked_humans.hpp"

namespace social_nav_costmap_layer
{

struct ResolvedHuman
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double speed = 0.0;
  bool moving = false;
  double sigma_front = 0.0;
  double sigma_back = 0.0;
  double sigma_side = 0.0;
  double cost_radius = 0.0;
  // Bounding radius (world units) used to extend updateBounds -- covers
  // cost_radius and the reach of the soft predicted corridor when moving.
  double bounding_radius = 0.0;
};

class AghpmLayer1 : public nav2_costmap_2d::Layer
{
public:
  AghpmLayer1() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void trackedHumansCallback(const thesis_msgs::msg::TrackedHumans::SharedPtr msg);
  double computeCostAt(double wx, double wy) const;

  rclcpp::Subscription<thesis_msgs::msg::TrackedHumans>::SharedPtr tracked_humans_sub_;

  std::mutex humans_mutex_;
  thesis_msgs::msg::TrackedHumans::SharedPtr latest_msg_;

  // Snapshot resolved once in updateBounds() (sigmas depending on speed and
  // covariance) and reused by updateCosts() in the SAME cycle, as other Nav2
  // layers (e.g. inflation_layer) do.
  std::vector<ResolvedHuman> resolved_humans_;

  // --- Parameters ---
  bool enabled_ = true;
  std::string tracked_humans_topic_ = "/planning/tracked_humans";
  double max_human_distance_ = 10.0;
  double max_cost_value_ = 220.0;  // < INSCRIBED_INFLATED_OBSTACLE (253)

  double sigma_front_ = 0.60;
  double sigma_back_ = 0.30;
  double sigma_side_ = 0.45;
  double human_cost_radius_ = 2.0;

  bool velocity_social_enabled_ = true;
  double velocity_sigma_front_gain_ = 0.10;
  // Constant added to sigma_front only while the person is MOVING.
  double moving_sigma_front_extra_ = 0.0;
  double velocity_sigma_side_gain_ = 0.03;
  double velocity_sigma_age_gain_ = 0.05;
  double velocity_cost_radius_gain_ = 0.10;

  bool social_covariance_enabled_ = true;
  double social_covariance_gain_ = 1.0;
  double social_covariance_sigma_cap_ = 0.30;

  bool velocity_future_enabled_ = true;
  double velocity_future_horizon_ = 1.0;
  double velocity_future_step_ = 0.2;
  double velocity_future_cost_max_ = 0.45;
  double velocity_future_decay_tau_ = 1.5;
  double velocity_future_sigma_long_ = 0.30;
  double velocity_future_sigma_side_ = 0.25;
};

}  // namespace social_nav_costmap_layer

#endif  // SOCIAL_NAV_COSTMAP_LAYER__AGHPM_LAYER1_HPP_
