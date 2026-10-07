// Human-aware local controller (Nav2 FollowPath plugin).
//
// Dynamic-window approach that samples (v, w) pairs, forward-simulates each one
// and scores it against the global path, the costmap and the tracked humans.
//
// Safety model:
//  - Closed-form hard / social risk per human (segment minimum clearance and
//    boundary time-to-collision with a growing uncertainty radius).
//  - Hysteretic emergency mode (emergency_ttc_enter / exit + exit_hold).
//  - Recovery candidates when the robot is already inside the hard radius.
//  - Anti-oscillation: a new candidate must beat the previous one by
//    switch_score_margin in normal mode; same-sign tie-break in emergency mode.
//  - The published (v, w) is the risk-checked command itself; there is no
//    separate path-following layer.
//
// Performance: 7 x 9 velocity samples at 20 Hz, and the trajectory buffers are
// reused across ticks so the control loop does not allocate after warm-up.
#ifndef SOCIAL_NAV_CONTROLLER__HUMAN_AWARE_CONTROLLER1_HPP_
#define SOCIAL_NAV_CONTROLLER__HUMAN_AWARE_CONTROLLER1_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "thesis_msgs/msg/tracked_humans.hpp"

namespace social_nav_controller
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

// Risk of one trajectory with respect to one tracked human.
struct HumanTrackRisk
{
  int32_t track_id = 0;
  double current_clearance = 0.0;
  double min_future_clearance = 0.0;
  double final_clearance = 0.0;
  double time_to_minimum = 0.0;
  double ttc = 0.0;
  double hard_ttc = 0.0;
  double initial_clearance_rate = 0.0;
  bool hard_collision = false;
};

// Risk of one trajectory aggregated over all tracked humans.
struct HumanRisk
{
  bool valid = true;
  bool active = false;
  std::vector<HumanTrackRisk> per_human;
  double current_clearance = std::numeric_limits<double>::infinity();
  double min_future_clearance = std::numeric_limits<double>::infinity();
  double final_clearance = std::numeric_limits<double>::infinity();
  double time_to_minimum = std::numeric_limits<double>::infinity();
  double ttc = std::numeric_limits<double>::infinity();
  double hard_ttc = std::numeric_limits<double>::infinity();
  bool hard_collision = false;
};

/// Smoothed walking direction of a person together with a confidence value.
///
/// The raw tracker velocity is noisy and lags when a person reverses, so
/// `confidence` drops to zero as soon as a reversal is detected and then
/// recovers slowly. Passing in front of a person is only allowed when the
/// walking direction is known with enough confidence.
struct HumanIntent
{
  double dir_x = 0.0;
  double dir_y = 0.0;
  double confidence = 0.0;
  int64_t last_seen_ns = 0;
};

struct Candidate
{
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  std::vector<Point2D> trajectory;
  double end_yaw = 0.0;
  double social_cost = 0.0;
  HumanRisk human_risk;
  /// 0 = wide time gap at the crossing point; 1 = robot and person arrive together.
  double passing_cost = 0.0;
  /// 0 = trajectory leaves the person's walking path; 1 = still blocking it.
  double yield_cost = 0.0;
};

class HumanAwareController1 : public nav2_core::Controller
{
public:
  HumanAwareController1() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void trackedHumansCallback(const thesis_msgs::msg::TrackedHumans::SharedPtr msg);
  bool transformPlanToCostmapFrame();

  double rHard() const {return robot_radius_ + human_radius_ + human_hard_margin_;}
  double rSocial() const
  {
    return std::max(
      robot_radius_ + human_radius_ + human_safety_margin_, rHard());
  }
  static int nearestIndex(const std::vector<Point2D> & path, double x, double y);
  static Point2D pickLookaheadTarget(
    const std::vector<Point2D> & path, int start_index, double lookahead_distance);
  void simulate(
    double x, double y, double yaw, double v, double w, double duration,
    std::vector<Point2D> & trajectory_out, double & end_yaw_out) const;
  double pathAlignmentCost(const std::vector<Point2D> & trajectory) const;
  std::pair<bool, double> trajectoryCostmapCost(const std::vector<Point2D> & trajectory) const;
  static double segmentBoundaryTtc(
    double rx, double ry, double vx, double vy, double duration,
    double boundary_start, double boundary_rate);
  static std::pair<double, double> segmentMinimumClearance(
    double rx, double ry, double vx, double vy, double duration,
    double boundary_start, double boundary_rate);
  static HumanRisk invalidHumanRisk();
  HumanRisk trajectoryHumanRisk(
    const std::vector<Point2D> & trajectory, double robot_x, double robot_y) const;
  double humanProximityCost(double clearance) const;
  double humanTtcCost(double ttc) const;

  /// Update walking direction and confidence of every track. Called once per cycle.
  void updateHumanIntents(const thesis_msgs::msg::TrackedHumans & msg, int64_t now_ns);

  /// Time gap at the point where the trajectory crosses a person's path:
  /// gap = t_person - t_robot (s). gap > 0: robot arrives first; gap < 0:
  /// person arrives first; +inf: paths do not cross. Returns the most dangerous
  /// gap (smallest |gap|) over all walking people.
  double crossingGap(const std::vector<Point2D> & trajectory, double & confidence_out) const;

  double passingCost(double gap, double confidence) const;

  /// Cost of NOT yielding: a person is approaching (usually from the side) and
  /// this trajectory does not take the robot out of their walking path.
  /// 0 = far enough away; 1 = still in the middle of the path.
  double yieldCost(
    const std::vector<Point2D> & trajectory, double robot_x, double robot_y) const;
  // Returns true when in the final goal-approach phase and the command was written.
  bool goalApproachCommand(
    double robot_x, double robot_y, double robot_yaw,
    geometry_msgs::msg::TwistStamped & cmd) const;
  bool updateEmergencyState(const HumanRisk & standing_risk, int64_t now_ns);
  std::vector<double> linearVelocitySamples() const;
  double candidateScore(
    const Candidate & candidate, double target_x, double target_y) const;
  std::tuple<double, double, double, double, double, double> safetyKey(
    const Candidate & candidate, double normal_score) const;
  bool isRecoveryCandidate(const HumanRisk & candidate_risk, const HumanRisk & standing_risk) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("HumanAwareController1")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<thesis_msgs::msg::TrackedHumans>::SharedPtr tracked_humans_sub_;
  mutable std::mutex humans_mutex_;
  thesis_msgs::msg::TrackedHumans::SharedPtr latest_humans_msg_;
  rclcpp::Time latest_humans_stamp_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_pub_;

  nav_msgs::msg::Path global_plan_;
  std::vector<Point2D> path_costmap_frame_;
  // Goal heading in the costmap frame (path_costmap_frame_ only stores x, y).
  double goal_yaw_costmap_frame_ = 0.0;
  bool goal_yaw_valid_ = false;

  // Buffers reused across ticks to avoid heap allocation at 20 Hz.
  // candidate_pool_ is resize()d, not clear()ed, so each Candidate::trajectory
  // keeps its capacity.
  std::vector<Candidate> candidate_pool_;
  std::vector<Point2D> risk_trajectory_scratch_;
  std::vector<Point2D> standing_trajectory_scratch_;

  // --- Parameters ---
  double max_linear_vel_ = 0.5;
  double min_linear_vel_ = 0.0;
  double max_angular_vel_ = 1.2;
  // Goal approach. Nav2's stateful goal checker latches the position condition
  // once the robot is within xy_goal_tolerance and then only waits for the yaw,
  // so near the goal the controller stops translating and rotates towards the
  // goal heading. goal_approach_distance must be <= xy_goal_tolerance, otherwise
  // the robot stops before it enters the goal radius.
  double goal_approach_distance_ = 0.25;
  double goal_yaw_deadband_ = 0.10;
  double goal_yaw_gain_ = 1.5;
  double goal_min_rotate_speed_ = 0.25;
  int linear_samples_ = 7;
  int angular_samples_ = 9;

  double sim_time_ = 1.5;
  double sim_dt_ = 0.1;
  double lookahead_dist_ = 0.8;
  double rotate_heading_deadband_ = 0.08;
  double rotate_in_place_angle_ = 0.35;

  double w_goal_ = 2.0;
  double w_speed_ = 0.3;
  double w_path_ = 0.8;
  double w_social_ = 3.0;
  double w_human_proximity_ = 3.0;
  double w_human_ttc_ = 3.0;
  double w_heading_ = 1.0;
  double w_smooth_ = 1.0;
  double switch_score_margin_ = 0.3;

  double robot_radius_ = 0.22;
  // Costmap cost [0, 255] treated as a hard collision. Set to
  // INSCRIBED_INFLATED_OBSTACLE (253) so the social layer (capped at 220) can
  // never veto a trajectory on its own; only real obstacles reach this value.
  int collision_cost_ = 253;
  // Radius of the disc checked against the costmap along a trajectory. 0.0 =
  // point check, because the Nav2 costmap is already inflated by the footprint.
  // Using robot_radius_ here would count the robot size twice.
  double costmap_check_radius_ = 0.0;
  bool unknown_is_obstacle_ = true;

  bool predictive_humans_enabled_ = true;
  double human_radius_ = 0.30;
  double human_safety_margin_ = 0.20;
  double human_hard_margin_ = 0.05;
  double human_uncertainty_rate_ = 0.10;
  /// Growth of the uncertainty disc with the person's speed. 0 = disabled.
  double human_reversal_factor_ = 0.0;
  double human_stationary_uncertainty_rate_ = 0.03;
  double human_prediction_horizon_ = 3.0;
  double human_covariance_gain_ = 1.0;
  double human_covariance_sigma_cap_ = 0.20;
  double tracked_humans_timeout_ = 0.35;
  // If tracked humans are missing or stale, stop (v = w = 0) instead of
  // driving as if nobody were around.
  bool require_tracked_humans_ = true;
  int64_t last_stale_humans_warn_ns_ = 0;

  double emergency_ttc_enter_ = 1.5;
  double emergency_ttc_exit_ = 2.0;
  double emergency_exit_hold_ = 0.5;
  double emergency_clearance_improvement_ = 0.02;
  double emergency_clearance_tie_margin_ = 0.03;

  // --- Passing a walking person ---
  // Distance alone cannot tell "in front of" from "behind" a person: two
  // trajectories at the same distance can cross in front of them or pass
  // behind them. The passing cost scores the time gap at the crossing point.
  bool passing_enabled_ = true;
  /// When no candidate is acceptable and a person is close, move in the
  /// direction that opens the gap instead of stopping in their path.
  bool stuck_escape_enabled_ = true;
  double pass_ahead_margin_ = 1.5;      ///< s, robot must reach the crossing this much earlier
  double pass_behind_margin_ = 0.8;     ///< s, or pass behind the person by this much
  double w_passing_ = 5.0;
  double intent_confidence_min_ = 0.6;  ///< below this the direction is unknown (wider gap)
  double intent_ema_alpha_ = 0.3;
  double intent_min_speed_ = 0.15;      ///< below this the person is considered standing
  double intent_ttl_ = 2.0;             ///< s, forget a track not seen for this long

  // --- Yielding to a person approaching from the side ---
  // Scores the perpendicular distance from the robot to the person's predicted
  // walking path and penalises every candidate that does not increase it,
  // including standing still. Forward or backward is left to the scoring.
  bool yield_enabled_ = true;
  double yield_trigger_time_ = 4.0;   ///< s: only yield if the person arrives within this time
  double yield_clearance_ = 0.70;     ///< m: desired distance from the person's path
  double w_yield_ = 10.0;
  mutable std::unordered_map<uint32_t, HumanIntent> human_intents_;
  bool emergency_active_ = false;
  bool emergency_safe_since_set_ = false;
  int64_t emergency_safe_since_ns_ = 0;

  // Hysteresis between the "rotate in place" and "drive" candidate pools in
  // normal mode. Without it, a heading error hovering around
  // rotate_in_place_angle_ switches the pool every tick and the command
  // oscillates between creeping and full rotation.
  bool heading_rotation_mode_ = false;

  double prev_linear_velocity_ = 0.0;
  double prev_angular_velocity_ = 0.0;
};

}  // namespace social_nav_controller

#endif  // SOCIAL_NAV_CONTROLLER__HUMAN_AWARE_CONTROLLER1_HPP_
