// Thin node: reads /scan and /cmd_vel_raw, runs LidarSafety, publishes /cmd_vel.
//
// C++ port of social_nav_safety/lidar_safety_node1.py. The default node name is
// "lidar_safety_node" as in Python; the launch file sets
// name='lidar_safety_node1', which is the name the parameter file binds to.

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "social_nav_safety_cpp/lidar_safety.hpp"

namespace social_nav_safety_cpp
{

class LidarSafetyNode : public rclcpp::Node
{
public:
  LidarSafetyNode()
  : rclcpp::Node("lidar_safety_node")
  {
    SafetyParams params;
    params.control_rate = declare_parameter<double>("control_rate", params.control_rate);
    params.scan_timeout = declare_parameter<double>("scan_timeout", params.scan_timeout);
    params.cmd_timeout = declare_parameter<double>("cmd_timeout", params.cmd_timeout);
    params.robot_front_extent =
      declare_parameter<double>("robot_front_extent", params.robot_front_extent);
    params.front_half_width =
      declare_parameter<double>("front_half_width", params.front_half_width);
    params.robot_rear_extent =
      declare_parameter<double>("robot_rear_extent", params.robot_rear_extent);
    params.rear_half_width = declare_parameter<double>("rear_half_width", params.rear_half_width);
    params.robot_safety_radius =
      declare_parameter<double>("robot_safety_radius", params.robot_safety_radius);
    params.turn_slow_distance =
      declare_parameter<double>("turn_slow_distance", params.turn_slow_distance);
    params.turn_stop_distance =
      declare_parameter<double>("turn_stop_distance", params.turn_stop_distance);
    params.slow_distance = declare_parameter<double>("slow_distance", params.slow_distance);
    params.stop_distance = declare_parameter<double>("stop_distance", params.stop_distance);
    params.emergency_distance =
      declare_parameter<double>("emergency_distance", params.emergency_distance);
    params.ttc_slow = declare_parameter<double>("ttc_slow", params.ttc_slow);
    params.ttc_stop = declare_parameter<double>("ttc_stop", params.ttc_stop);
    params.closing_speed_alpha =
      declare_parameter<double>("closing_speed_alpha", params.closing_speed_alpha);
    params.max_closing_speed =
      declare_parameter<double>("max_closing_speed", params.max_closing_speed);
    params.release_distance =
      declare_parameter<double>("release_distance", params.release_distance);
    params.release_ttc = declare_parameter<double>("release_ttc", params.release_ttc);
    params.release_hold_time =
      declare_parameter<double>("release_hold_time", params.release_hold_time);
    params.escape_angular_speed =
      declare_parameter<double>("escape_angular_speed", params.escape_angular_speed);
    params.escape_away_enabled =
      declare_parameter<bool>("escape_away_enabled", params.escape_away_enabled);
    params.escape_away_speed =
      declare_parameter<double>("escape_away_speed", params.escape_away_speed);

    // Throws on nonsensical parameters, so the node fails at start-up.
    safety_ = std::make_unique<LidarSafety>(params);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {rawCmdCallback(*msg);});

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {scanCallback(*msg);});

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    const double period_seconds = 1.0 / std::max(params.control_rate, 1e-3);
    // Use the node clock (create_timer), not a wall timer, so the control loop
    // follows /clock when use_sim_time is true.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(period_seconds),
      [this]() {controlLoop();});

    RCLCPP_INFO(get_logger(), "LiDAR safety node started (C++)");
  }

private:
  void rawCmdCallback(const geometry_msgs::msg::Twist & msg)
  {
    VelocityCommand command;
    command.linear_x = msg.linear.x;
    command.angular_z = msg.angular.z;
    safety_->updateCommand(command, get_clock()->now().nanoseconds());
  }

  void scanCallback(const sensor_msgs::msg::LaserScan & msg)
  {
    ScanView view;
    view.ranges = msg.ranges.data();
    view.count = msg.ranges.size();
    view.angle_min = static_cast<double>(msg.angle_min);
    view.angle_increment = static_cast<double>(msg.angle_increment);
    view.range_min = static_cast<double>(msg.range_min);
    view.range_max = static_cast<double>(msg.range_max);
    safety_->updateScan(view, get_clock()->now().nanoseconds());
  }

  void controlLoop()
  {
    const VelocityCommand command = safety_->step(get_clock()->now().nanoseconds());
    geometry_msgs::msg::Twist msg;
    msg.linear.x = command.linear_x;
    msg.angular.z = command.angular_z;
    safe_cmd_pub_->publish(msg);
  }

  std::unique_ptr<LidarSafety> safety_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace social_nav_safety_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Single-threaded spin, as in the Python node: the state is not protected by
    // any lock, so the scan and control callbacks must never run concurrently.
    rclcpp::spin(std::make_shared<social_nav_safety_cpp::LidarSafetyNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_safety_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
