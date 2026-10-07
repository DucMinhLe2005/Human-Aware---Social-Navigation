// Thin node: takes human detections (camera) and scans (lidar), runs
// HumanTracker and publishes thesis_msgs/TrackedHumans.
//
// C++ port of social_nav_tracking/human_tracker_node1.py. The node name
// `human_tracker_node1` is kept so that human_tracker_params.yaml binds.
//
// Tracks are kept and published in the "odom" frame: the local costmap runs
// in odom, so the costmap layer does not need a map->odom lookup per track,
// and the person detector already publishes in odom. Only the static-map
// cluster filter needs map, and it transforms a few cluster centres only.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "thesis_msgs/msg/multiple_pose_result3_d.hpp"
#include "thesis_msgs/msg/pose_result3_d.hpp"
#include "thesis_msgs/msg/tracked_human.hpp"
#include "thesis_msgs/msg/tracked_humans.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "social_nav_tracking_cpp/distance_transform.hpp"
#include "social_nav_tracking_cpp/human_tracking.hpp"
#include "social_nav_tracking_cpp/lidar_clustering.hpp"

namespace social_nav_tracking_cpp
{
namespace
{

constexpr int64_t kWarningIntervalNs = 5000000000;   // 5 s
constexpr int64_t kPruneLogIntervalNs = 10000000000;  // 10 s

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(double angle)
{
  // atan2(sin, cos), not fmod, to match the Python reference.
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct HumanPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Body centre and torso heading from keypoints. `outlier_radius`: drop points
/// farther than this from the per-axis median of the torso keypoints (0 = off).
double torso_outlier_radius_ = 0.0;

bool poseToXYYaw(const thesis_msgs::msg::PoseResult3D & pose_result, HumanPose & out)
{
  if (pose_result.keypoints.empty()) {
    return false;
  }

  // Keep insertion order (the "first keypoint" fallback depends on it); a
  // repeated id overwrites the earlier value.
  std::vector<std::pair<int64_t, const thesis_msgs::msg::PoseKeypoint3D *>> keypoints;
  for (const auto & keypoint : pose_result.keypoints) {
    if (!(keypoint.confidence > 0.3f)) {
      continue;
    }
    bool replaced = false;
    for (auto & entry : keypoints) {
      if (entry.first == keypoint.id) {
        entry.second = &keypoint;
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      keypoints.emplace_back(keypoint.id, &keypoint);
    }
  }
  if (keypoints.empty()) {
    return false;
  }

  auto find = [&keypoints](int64_t id) -> const thesis_msgs::msg::PoseKeypoint3D * {
      for (const auto & entry : keypoints) {
        if (entry.first == id) {
          return entry.second;
        }
      }
      return nullptr;
    };

  std::vector<const thesis_msgs::msg::PoseKeypoint3D *> torso;
  for (const int64_t id : {5, 6, 11, 12}) {
    const auto * keypoint = find(id);
    if (keypoint != nullptr) {
      torso.push_back(keypoint);
    }
  }

  if (!torso.empty()) {
    // Outlier rejection before averaging. Depth is sampled at the keypoint pixel,
    // so a keypoint that falls just outside the person reads the wall behind
    // them and drags the mean metres away. Use the per-axis median as anchor
    // and average only the points within torso_outlier_radius of it; fall back
    // to the anchor if none qualifies.
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(torso.size());
    ys.reserve(torso.size());
    for (const auto * keypoint : torso) {
      xs.push_back(keypoint->x);
      ys.push_back(keypoint->y);
    }
    auto median_of = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
      };
    const double anchor_x = median_of(xs);
    const double anchor_y = median_of(ys);

    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t count = 0;
    for (const auto * keypoint : torso) {
      const double dx = keypoint->x - anchor_x;
      const double dy = keypoint->y - anchor_y;
      if (torso_outlier_radius_ > 0.0 &&
        dx * dx + dy * dy > torso_outlier_radius_ * torso_outlier_radius_)
      {
        continue;
      }
      sum_x += keypoint->x;
      sum_y += keypoint->y;
      count += 1;
    }
    if (count > 0) {
      out.x = sum_x / static_cast<double>(count);
      out.y = sum_y / static_cast<double>(count);
    } else {
      out.x = anchor_x;
      out.y = anchor_y;
    }
  } else {
    out.x = keypoints.front().second->x;
    out.y = keypoints.front().second->y;
  }

  double human_yaw = 0.0;
  const auto * left_shoulder = find(5);
  const auto * right_shoulder = find(6);
  const auto * left_hip = find(11);
  const auto * right_hip = find(12);
  const auto * nose = find(0);

  if (left_shoulder != nullptr && right_shoulder != nullptr) {
    human_yaw = std::atan2(
      right_shoulder->y - left_shoulder->y,
      right_shoulder->x - left_shoulder->x) + M_PI / 2.0;
  } else if (left_hip != nullptr && right_hip != nullptr) {
    human_yaw = std::atan2(
      right_hip->y - left_hip->y,
      right_hip->x - left_hip->x) + M_PI / 2.0;
  } else if (nose != nullptr && (left_hip != nullptr || right_hip != nullptr)) {
    const auto * hip = left_hip != nullptr ? left_hip : right_hip;
    human_yaw = std::atan2(nose->y - hip->y, nose->x - hip->x);
  }

  out.yaw = normalizeAngle(human_yaw);
  return true;
}

}  // namespace

class HumanTrackerNode1 : public rclcpp::Node
{
public:
  HumanTrackerNode1()
  : Node("human_tracker_node1")
  {
    declareAndReadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // spin_thread = true: /tf is filled by a dedicated thread, so lookups with a
    // timeout (see lookup2DTransform) can actually wait for new transforms.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // The map must use TRANSIENT_LOCAL QoS to match map_server, which publishes
    // the map once at start-up; with VOLATILE QoS a tracker started later would
    // never receive it and the static-obstacle filter would silently do nothing.
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&HumanTrackerNode1::mapCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<thesis_msgs::msg::MultiplePoseResult3D>(
      "/perception/yolov8_multiple_pose_results_3d_ori", 1,
      std::bind(&HumanTrackerNode1::poseCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HumanTrackerNode1::scanCallback, this, std::placeholders::_1));

    tracked_humans_pub_ = this->create_publisher<thesis_msgs::msg::TrackedHumans>(
      "/planning/tracked_humans", 10);
    tracked_humans_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/local_debug/tracked_humans", 10);
    lidar_person_centers_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/planning/local_debug/lidar_person_centers", 10);

    // Use the node clock (not a wall timer) so the timer follows /clock when
    // use_sim_time is true.
    timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      rclcpp::Duration::from_seconds(1.0 / update_rate_),
      std::bind(&HumanTrackerNode1::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "human_tracker_node1 (C++) started: scan_topic='%s', update_rate=%.1f Hz, "
      "tracker frame=odom (see file header)",
      scan_topic_.c_str(), update_rate_);
  }

private:
  // ---- parameters ---------------------------------------------------------

  void declareAndReadParameters()
  {
    update_rate_ = this->declare_parameter<double>("update_rate", 4.0);
    occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 65);
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    lidar_min_range_ = this->declare_parameter<double>("lidar_min_range", 0.15);
    lidar_max_range_ = this->declare_parameter<double>("lidar_max_range", 8.0);
    lidar_observation_timeout_ =
      this->declare_parameter<double>("lidar_observation_timeout", 0.5);
    publish_tracking_markers_ =
      this->declare_parameter<bool>("publish_tracking_markers", true);
    human_lidar_fusion_radius_ =
      this->declare_parameter<double>("human_lidar_fusion_radius", 0.4);
    human_lidar_sync_tolerance_ =
      this->declare_parameter<double>("human_lidar_sync_tolerance", 0.10);
    lidar_person_cluster_min_points_ =
      this->declare_parameter<int>("lidar_person_cluster_min_points", 3);
    lidar_person_cluster_max_gap_ =
      this->declare_parameter<double>("lidar_person_cluster_max_gap", 0.15);
    lidar_person_cluster_max_diameter_ =
      this->declare_parameter<double>("lidar_person_cluster_max_diameter", 0.75);
    lidar_person_radius_ = this->declare_parameter<double>("lidar_person_radius", 0.28);
    // Merge the two leg clusters of one person. 0 = off.
    lidar_person_merge_distance_ =
      this->declare_parameter<double>("lidar_person_merge_distance", 0.0);
    lidar_static_rejection_radius_ =
      this->declare_parameter<double>("lidar_static_rejection_radius", 0.15);
    publish_unconfirmed_lidar_only_ =
      this->declare_parameter<bool>("publish_unconfirmed_lidar_only", false);
    // Drop lidar-only tracks whose published position lies on a static obstacle.
    // > 0 enables it. See dropTracksOnStaticMap.
    publish_static_rejection_radius_ =
      this->declare_parameter<double>("publish_static_rejection_radius", 0.0);
    // Maximum age (s) of the last measurement of a standing track that is still
    // published. > 0 enables it. See dropStaleFrozenTracks.
    max_frozen_publish_age_ =
      this->declare_parameter<double>("max_frozen_publish_age", 0.0);

    // An empty list in YAML (`key: []`) has no type and fails to load; to disable
    // this filter, remove the key from the YAML file instead.
    const std::vector<double> static_zone_values =
      this->declare_parameter<std::vector<double>>(
      "lidar_static_exclusion_zones", std::vector<double>{});
    for (std::size_t index = 0; index + 2 < static_zone_values.size(); index += 3) {
      lidar_static_exclusion_zones_.push_back(
        ExclusionZone{
          static_zone_values[index], static_zone_values[index + 1],
          static_zone_values[index + 2]});
    }

    HumanTrackerParams params;
    params.association_gate =
      this->declare_parameter<double>("track_association_gate", 0.75);
    params.association_gate_speed_factor =
      this->declare_parameter<double>("track_association_gate_speed_factor", 0.5);
    params.min_hits_for_velocity =
      this->declare_parameter<int>("track_min_hits_for_velocity", 3);
    params.max_plausible_speed =
      this->declare_parameter<double>("track_max_plausible_speed", 2.0);
    params.moving_speed_threshold =
      this->declare_parameter<double>("track_moving_speed_threshold", 0.20);
    params.moving_exit_speed_threshold = std::max(
      0.0, this->declare_parameter<double>("track_moving_exit_speed_threshold", 0.10));
    params.moving_hold_time = std::max(
      0.0, this->declare_parameter<double>("track_moving_hold_time", 0.5));
    params.track_timeout = this->declare_parameter<double>("track_timeout", 0.35);
    params.kf_process_noise_std =
      this->declare_parameter<double>("track_kf_process_noise_std", 1.0);
    // Reversal handling -- see HumanTrackerParams::reversal_* in the header.
    params.reversal_speed_threshold =
      this->declare_parameter<double>("track_reversal_speed_threshold", 0.25);
    params.reversal_innovation_threshold =
      this->declare_parameter<double>("track_reversal_innovation_threshold", 0.10);
    params.reversal_covariance_boost =
      this->declare_parameter<double>("track_reversal_covariance_boost", 9.0);
    params.kf_measurement_noise_std =
      this->declare_parameter<double>("track_kf_measurement_noise_std", 0.15);
    params.kf_lidar_measurement_noise_std =
      this->declare_parameter<double>("track_kf_lidar_measurement_noise_std", 0.20);
    params.track_min_hits_to_confirm =
      this->declare_parameter<int>("track_min_hits_to_confirm", 3);
    params.track_lidar_coast_gate =
      this->declare_parameter<double>("track_lidar_coast_gate", 0.5);
    params.track_coast_match_timeout =
      this->declare_parameter<double>("track_coast_match_timeout", 0.5);
    params.track_coast_timeout =
      this->declare_parameter<double>("track_coast_timeout", 3.0);
    // The four defaults below differ from the HumanTrackerParams struct defaults
    // on purpose: these node defaults are the ones that apply when the YAML file
    // does not set a value (as in the Python reference). The deployed YAML sets
    // all four explicitly.
    params.lidar_only_association_gate =
      this->declare_parameter<double>("lidar_only_association_gate", 0.35);
    params.lidar_only_min_hits = this->declare_parameter<int>("lidar_only_min_hits", 4);
    params.lidar_only_min_duration =
      this->declare_parameter<double>("lidar_only_min_duration", 0.0);
    params.camera_vouched_lidar_hold =
      this->declare_parameter<bool>("camera_vouched_lidar_hold", false);
    params.ghost_publish_stationary =
      this->declare_parameter<bool>("ghost_publish_stationary", false);
    publish_dedupe_distance_ =
      this->declare_parameter<double>("publish_dedupe_distance", 0.0);
    torso_outlier_radius_ =
      this->declare_parameter<double>("torso_outlier_radius", 0.0);
    params.lidar_only_min_displacement =
      this->declare_parameter<double>("lidar_only_min_displacement", 0.12);
    params.lidar_only_moving_speed_threshold =
      this->declare_parameter<double>("lidar_only_moving_speed_threshold", 0.15);
    params.lidar_only_enabled = this->declare_parameter<bool>("lidar_only_enabled", true);
    params.lidar_only_confirm_decay_sec =
      this->declare_parameter<double>("lidar_only_confirm_decay_sec", 3.0);
    params.fusion_confirm_min_hits =
      this->declare_parameter<int>("fusion_confirm_min_hits", 3);
    params.lidar_only_track_timeout =
      this->declare_parameter<double>("lidar_only_track_timeout", 0.5);
    params.person_memory_ttl = this->declare_parameter<double>("person_memory_ttl", 5.0);
    params.person_memory_radius_base =
      this->declare_parameter<double>("person_memory_radius_base", 0.5);
    params.person_memory_radius_max =
      this->declare_parameter<double>("person_memory_radius_max", 2.5);
    params.person_memory_growth_speed =
      this->declare_parameter<double>("person_memory_growth_speed", 1.5);
    params.ghost_publish_min_speed =
      this->declare_parameter<double>("ghost_publish_min_speed", 0.3);

    tracker_ = std::make_unique<HumanTracker>(params);
  }

  // ---- static map -------------------------------------------------------

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const std::size_t width = msg->info.width;
    const std::size_t height = msg->info.height;
    const bool valid_map =
      width > 0 && height > 0 && msg->info.resolution > 0.0 &&
      msg->data.size() == width * height;
    if (!valid_map) {
      return;
    }

    std::vector<char> obstacles(msg->data.size());
    bool any_obstacle = false;
    for (std::size_t i = 0; i < msg->data.size(); ++i) {
      obstacles[i] = static_cast<char>(msg->data[i] >= occupied_threshold_);
      any_obstacle = any_obstacle || obstacles[i];
    }

    std::vector<double> clearance;
    if (any_obstacle) {
      clearance = exactEuclideanDistanceTransform(obstacles, width, height);
      for (double & value : clearance) {
        value *= msg->info.resolution;
      }
    } else {
      clearance.assign(msg->data.size(), std::numeric_limits<double>::infinity());
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      static_map_.grid.assign(msg->data.begin(), msg->data.end());
      static_map_.clearance = std::move(clearance);
      static_map_.width = width;
      static_map_.height = height;
      static_map_.resolution = msg->info.resolution;
      static_map_.origin_x = msg->info.origin.position.x;
      static_map_.origin_y = msg->info.origin.position.y;
      static_map_.origin_yaw = quaternionToYaw(msg->info.origin.orientation);
      has_map_ = true;
    }

    if (!logged_first_map_) {
      logged_first_map_ = true;
      // Log the map reception: if the map never arrives, the static-obstacle filter
      // silently does nothing, so make that visible.
      RCLCPP_INFO(
        this->get_logger(),
        "static map received: %zux%zu @ %.3f m/cell", width, height, msg->info.resolution);
    }
  }

  // ---- TF -----------------------------------------------------------------

  struct Transform2D
  {
    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
  };

  /// Returns (tx, ty, yaw) of source_frame in target_frame.
  ///
  /// timeout_sec is 0.1 s for lidar and 0.05 s for human poses and map->odom.
  /// Waiting helps because the TransformListener runs its own thread: sensor
  /// data is usually a few tens of ms ahead of the latest transform.
  bool lookup2DTransform(
    const std::string & target_frame, const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp, double timeout_sec, Transform2D & out) const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePoint(
          std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)),
        tf2::durationFromSec(timeout_sec));
      out.tx = transform.transform.translation.x;
      out.ty = transform.transform.translation.y;
      out.yaw = quaternionToYaw(transform.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool transformXYYaw(
    const HumanPose & in, const std::string & source_frame,
    const std::string & target_frame, const builtin_interfaces::msg::Time & stamp,
    HumanPose & out)
  {
    if (source_frame.empty() || source_frame == target_frame) {
      out = in;
      return true;
    }
    Transform2D transform;
    if (!lookup2DTransform(target_frame, source_frame, stamp, 0.05, transform)) {
      const int64_t now_ns = this->get_clock()->now().nanoseconds();
      if (now_ns - last_human_tf_warning_ns_ > kWarningIntervalNs) {
        RCLCPP_WARN(
          this->get_logger(), "Cannot transform human pose %s -> %s",
          source_frame.c_str(), target_frame.c_str());
        last_human_tf_warning_ns_ = now_ns;
      }
      return false;
    }
    const double cos_yaw = std::cos(transform.yaw);
    const double sin_yaw = std::sin(transform.yaw);
    out.x = transform.tx + cos_yaw * in.x - sin_yaw * in.y;
    out.y = transform.ty + sin_yaw * in.x + cos_yaw * in.y;
    out.yaw = normalizeAngle(in.yaw + transform.yaw);
    return true;
  }

  // ---- lidar state snapshot ---------------------------------------------

  struct LidarSnapshot
  {
    std::vector<Point2D> person_centers;
    std::size_t point_count = 0;
    bool is_fresh = false;
    int64_t generation = 0;
    int64_t stamp_ns = 0;
  };

  LidarSnapshot snapshotLidar(int64_t now_ns)
  {
    LidarSnapshot snapshot;
    int64_t last_update_ns = 0;
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      snapshot.person_centers = lidar_person_centers_odom_;
      snapshot.point_count = lidar_point_count_;
      last_update_ns = last_lidar_update_ns_;
      snapshot.stamp_ns = last_lidar_stamp_ns_;
      snapshot.generation = lidar_generation_;
    }

    const double receipt_age = last_update_ns > 0 ?
      static_cast<double>(now_ns - last_update_ns) / 1e9 :
      std::numeric_limits<double>::infinity();
    snapshot.is_fresh =
      last_update_ns > 0 &&
      receipt_age >= 0.0 && receipt_age <= lidar_observation_timeout_ &&
      snapshot.point_count > 0;
    return snapshot;
  }

  void handleClockReset(int64_t now_ns)
  {
    const bool clock_moved_backwards = last_clock_ns_ > 0 && now_ns < last_clock_ns_;
    last_clock_ns_ = now_ns;
    if (!clock_moved_backwards) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      tracker_->reset();
    }
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      lidar_point_count_ = 0;
      lidar_person_centers_odom_.clear();
      last_lidar_update_ns_ = 0;
      last_lidar_stamp_ns_ = 0;
      lidar_generation_ += 1;
    }
    last_coasted_lidar_generation_ = -1;
    last_fused_lidar_generation_ = -1;
    last_human_measurement_ns_ = 0;
    RCLCPP_WARN(
      this->get_logger(), "ROS clock moved backwards; cleared human tracks and LiDAR cache");
  }

  // ---- camera detections ------------------------------------------------

  void poseCallback(const thesis_msgs::msg::MultiplePoseResult3D::SharedPtr msg)
  {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    handleClockReset(now_ns);

    int64_t measurement_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000 +
      static_cast<int64_t>(msg->header.stamp.nanosec);
    if (measurement_ns <= 0) {
      measurement_ns = now_ns;
    }
    if (measurement_ns <= last_human_measurement_ns_) {
      return;
    }
    last_human_measurement_ns_ = measurement_ns;

    const LidarSnapshot snapshot = snapshotLidar(now_ns);
    const double lidar_time_error = snapshot.stamp_ns > 0 ?
      std::fabs(static_cast<double>(snapshot.stamp_ns - measurement_ns)) / 1e9 :
      std::numeric_limits<double>::infinity();
    const bool can_fuse_lidar =
      snapshot.is_fresh &&
      snapshot.generation != last_fused_lidar_generation_ &&
      lidar_time_error <= human_lidar_sync_tolerance_;
    // The fusion branch (associating a detection with the nearest lidar point)
    // only records last_fused_lidar_generation; it does not change the output
    // coordinates, so human_lidar_fusion_radius / human_lidar_sync_tolerance
    // currently have no effect. Behaviour kept identical to the reference.
    (void)can_fuse_lidar;
    (void)human_lidar_fusion_radius_;

    std::vector<Detection> humans;
    for (const auto & pose : msg->poses_result_3d) {
      HumanPose human;
      if (!poseToXYYaw(pose, human)) {
        continue;
      }
      std::string source_frame = pose.header.frame_id;
      if (source_frame.empty()) {
        source_frame = msg->header.frame_id;
      }
      if (source_frame.empty()) {
        source_frame = "odom";
      }
      HumanPose human_odom;
      if (!transformXYYaw(human, source_frame, "odom", msg->header.stamp, human_odom)) {
        continue;
      }
      humans.push_back(Detection{human_odom.x, human_odom.y, human_odom.yaw});
    }

    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      tracker_->update(humans, measurement_ns);
    }
  }

  // ---- lidar --------------------------------------------------------------

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) {
      return;
    }

    const double sensor_max = std::isfinite(msg->range_max) ?
      static_cast<double>(msg->range_max) : lidar_max_range_;
    const double minimum = std::max(static_cast<double>(msg->range_min), lidar_min_range_);
    const double maximum = std::min(sensor_max, lidar_max_range_);

    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    handleClockReset(now_ns);

    int64_t scan_stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000 +
      static_cast<int64_t>(msg->header.stamp.nanosec);
    if (scan_stamp_ns <= 0) {
      scan_stamp_ns = now_ns;
    }

    std::vector<std::size_t> valid_indices;
    valid_indices.reserve(msg->ranges.size());
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double range = msg->ranges[i];
      if (std::isfinite(range) && range >= minimum && range <= maximum) {
        valid_indices.push_back(i);
      }
    }

    if (valid_indices.empty()) {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      lidar_point_count_ = 0;
      lidar_person_centers_odom_.clear();
      last_lidar_update_ns_ = now_ns;
      last_lidar_stamp_ns_ = scan_stamp_ns;
      lidar_generation_ += 1;
      return;
    }

    std::string source_frame = msg->header.frame_id;
    if (source_frame.empty()) {
      source_frame = "lidar_link";
    }
    Transform2D transform;
    if (!lookup2DTransform("odom", source_frame, msg->header.stamp, 0.1, transform)) {
      if (now_ns - last_lidar_tf_warning_ns_ > kWarningIntervalNs) {
        RCLCPP_WARN(
          this->get_logger(), "Cannot transform LiDAR %s -> odom", source_frame.c_str());
        last_lidar_tf_warning_ns_ = now_ns;
      }
      return;
    }

    const double cos_yaw = std::cos(transform.yaw);
    const double sin_yaw = std::sin(transform.yaw);

    std::vector<Point2D> odom_points;
    std::vector<int64_t> scan_indices;
    odom_points.reserve(valid_indices.size());
    scan_indices.reserve(valid_indices.size());
    for (const std::size_t index : valid_indices) {
      const double angle = static_cast<double>(msg->angle_min) +
        static_cast<double>(index) * static_cast<double>(msg->angle_increment);
      const double range = msg->ranges[index];
      const double sensor_x = range * std::cos(angle);
      const double sensor_y = range * std::sin(angle);
      odom_points.push_back(
        Point2D{
          transform.tx + cos_yaw * sensor_x - sin_yaw * sensor_y,
          transform.ty + sin_yaw * sensor_x + cos_yaw * sensor_y});
      scan_indices.push_back(static_cast<int64_t>(index));
    }

    std::vector<Point2D> person_centers = clusterLidarPersonCenters(
      odom_points, scan_indices, Point2D{transform.tx, transform.ty},
      lidar_person_cluster_min_points_, lidar_person_cluster_max_gap_,
      lidar_person_cluster_max_diameter_, lidar_person_radius_);
    person_centers = mergeNearbyCenters(person_centers, lidar_person_merge_distance_);
    person_centers = filterByStaticMapViaOdom(person_centers, msg->header.stamp);

    geometry_msgs::msg::PoseArray centers_msg;
    centers_msg.header.stamp = msg->header.stamp;
    centers_msg.header.frame_id = "odom";
    for (const auto & center : person_centers) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = center.x;
      pose.position.y = center.y;
      pose.orientation.w = 1.0;
      centers_msg.poses.push_back(pose);
    }
    lidar_person_centers_pub_->publish(centers_msg);

    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      // Only the point count is needed (snapshotLidar checks whether points exist),
      // so the full point cloud is not stored.
      lidar_point_count_ = odom_points.size();
      lidar_person_centers_odom_ = person_centers;
      last_lidar_update_ns_ = now_ns;
      last_lidar_stamp_ns_ = scan_stamp_ns;
      lidar_generation_ += 1;
    }
  }

  /// Filter clusters against the static map (map frame) and return the result in odom.
  std::vector<Point2D> filterByStaticMapViaOdom(
    const std::vector<Point2D> & centers_odom, const builtin_interfaces::msg::Time & stamp)
  {
    if (centers_odom.empty()) {
      return centers_odom;
    }

    StaticMap map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!has_map_) {
        return centers_odom;
      }
      map_copy = static_map_;
    }

    Transform2D transform;
    if (!lookup2DTransform("map", "odom", stamp, 0.05, transform)) {
      const int64_t now_ns = this->get_clock()->now().nanoseconds();
      if (now_ns - last_static_filter_tf_warning_ns_ > kWarningIntervalNs) {
        RCLCPP_WARN(
          this->get_logger(),
          "Cannot transform odom -> map for static-lidar filter; "
          "skipping the static map filter this time");
        last_static_filter_tf_warning_ns_ = now_ns;
      }
      return centers_odom;
    }

    const double cos_yaw = std::cos(transform.yaw);
    const double sin_yaw = std::sin(transform.yaw);
    std::vector<Point2D> centers_map;
    centers_map.reserve(centers_odom.size());
    for (const auto & center : centers_odom) {
      centers_map.push_back(
        Point2D{
          transform.tx + cos_yaw * center.x - sin_yaw * center.y,
          transform.ty + sin_yaw * center.x + cos_yaw * center.y});
    }

    // Both filters return indices, so mapping back to odom is an index lookup.
    const auto kept_by_map = filterByStaticMapIndices(
      centers_map, map_copy, lidar_static_rejection_radius_);

    std::vector<Point2D> kept_map_points;
    kept_map_points.reserve(kept_by_map.size());
    for (const std::size_t index : kept_by_map) {
      kept_map_points.push_back(centers_map[index]);
    }
    const auto kept_by_zones = filterByExclusionZonesIndices(
      kept_map_points, lidar_static_exclusion_zones_);

    std::vector<Point2D> result;
    result.reserve(kept_by_zones.size());
    for (const std::size_t index : kept_by_zones) {
      result.push_back(centers_odom[kept_by_map[index]]);
    }
    return result;
  }

  // ---- publishing -------------------------------------------------------

  struct PublishedPosition
  {
    double x = 0.0;
    double y = 0.0;
    bool moving = false;
    double observation_age = 0.0;
  };

  PublishedPosition trackPositionAt(const TrackPtr & track, int64_t now_ns)
  {
    const auto & params = tracker_->params();
    PublishedPosition out;
    out.moving = track->isMoving(
      params.min_hits_for_velocity, params.moving_speed_threshold, now_ns,
      params.moving_hold_time, params.moving_exit_speed_threshold);
    double dt = std::max(0.0, static_cast<double>(now_ns - track->last_update_ns) / 1e9);
    dt = std::min(dt, params.track_coast_timeout);
    out.observation_age = dt;
    if (!out.moving) {
      out.x = track->x;
      out.y = track->y;
      return out;
    }
    const auto predicted = track->predict(dt);
    out.x = predicted.first;
    out.y = predicted.second;
    return out;
  }

  /// Remove unconfirmed lidar-only tracks from the published message.
  ///
  /// lidar_only_min_hits / lidar_only_min_displacement only set the
  /// `lidar_only_confirmed` flag, while every lidar-born track without camera
  /// hits is labelled LIDAR_ONLY. Without this filter a fresh cluster on a wall
  /// would be published as a person immediately.
  std::vector<TrackPtr> filterPublishableTracks(const std::vector<TrackPtr> & tracks) const
  {
    if (publish_unconfirmed_lidar_only_) {
      return tracks;
    }
    std::vector<TrackPtr> result;
    for (const auto & track : tracks) {
      if (!track->lidar_origin_x.has_value() || track->camera_hits > 0 ||
        track->lidar_only_confirmed)
      {
        result.push_back(track);
      }
    }
    return result;
  }

  /// Drop lidar-only tracks whose PUBLISHED position lies on a static obstacle.
  ///
  /// filterByStaticMapViaOdom runs once on the cluster centre at detection time,
  /// but the published position is the Kalman estimate (and, for moving tracks,
  /// a prediction), which can drift onto a wall and stay there. In a 5-minute
  /// crossing scenario, 99% of ghost ids were within 0.40 m of a static obstacle
  /// versus 13% of ids matching a real person.
  ///
  /// Only applies to tracks never confirmed by the camera, so a real person
  /// standing next to a wall is kept. Output filter at node level; the
  /// HumanTracker core is untouched.
  std::vector<TrackPtr> dropTracksOnStaticMap(
    const std::vector<TrackPtr> & tracks, const rclcpp::Time & now)
  {
    if (publish_static_rejection_radius_ <= 0.0 || tracks.empty()) {
      return tracks;
    }

    StaticMap map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!has_map_) {
        return tracks;
      }
      map_copy = static_map_;
    }

    // Use the LATEST transform, not `now`: map->odom from AMCL always lags the
    // current time slightly, so asking for `now` fails with "extrapolation into
    // the future" every time. map->odom only changes when AMCL corrects the pose.
    //
    // Timeout must be 0: this runs every tick in the publishing thread, and
    // waiting here starves the thread that fills the TF buffer.
    builtin_interfaces::msg::Time latest;
    latest.sec = 0;
    latest.nanosec = 0;
    Transform2D transform;
    if (!lookup2DTransform("map", "odom", latest, 0.0, transform)) {
      static_tf_failures_ += 1;
      return tracks;   // lookup failed: do not filter (same as the input filter)
    }

    const double cos_yaw = std::cos(transform.yaw);
    const double sin_yaw = std::sin(transform.yaw);

    std::vector<Point2D> centers_map;
    std::vector<std::size_t> checked_indices;
    centers_map.reserve(tracks.size());
    checked_indices.reserve(tracks.size());
    std::vector<TrackPtr> result;
    result.reserve(tracks.size());

    for (std::size_t i = 0; i < tracks.size(); ++i) {
      const auto & track = tracks[i];
      if (track->camera_hits > 0 || track->fusion_hits > 0) {
        kept_camera_confirmed_ += 1;
        result.push_back(track);        // camera-confirmed: keep
        continue;
      }
      const PublishedPosition position = trackPositionAt(track, now.nanoseconds());
      centers_map.push_back(
        Point2D{
          transform.tx + cos_yaw * position.x - sin_yaw * position.y,
          transform.ty + sin_yaw * position.x + cos_yaw * position.y});
      checked_indices.push_back(i);
    }

    const auto kept = filterByStaticMapIndices(
      centers_map, map_copy, publish_static_rejection_radius_);
    for (const std::size_t k : kept) {
      result.push_back(tracks[checked_indices[k]]);
    }
    static_checked_ += checked_indices.size();
    dropped_on_static_map_ += checked_indices.size() - kept.size();
    return result;
  }

  /// Drop "ghost footprints": tracks that lost their measurements but keep being
  /// published at an old position after the person has walked on.
  ///
  /// Published tracks were found behind the walking person 72% of the time
  /// (median 0.74 m behind), while raw lidar clusters sat on the person (median
  /// 0.04 m). The sensor is right; the tracker keeps stale tracks alive.
  ///
  /// Output filter at node level; the HumanTracker core is untouched.
  std::vector<TrackPtr> dropStaleFrozenTracks(
    const std::vector<TrackPtr> & tracks, const rclcpp::Time & now)
  {
    if (max_frozen_publish_age_ <= 0.0 || tracks.empty()) {
      return tracks;
    }
    std::vector<TrackPtr> result;
    result.reserve(tracks.size());
    for (const auto & track : tracks) {
      const PublishedPosition position = trackPositionAt(track, now.nanoseconds());
      // Applied to moving tracks too: a track born on a single leg has its own
      // velocity estimate and keeps predicting along the old direction after the
      // person has left, so it also falls behind. Expired is expired.
      if (position.observation_age > max_frozen_publish_age_) {
        dropped_stale_frozen_ += 1;
        continue;
      }
      result.push_back(track);
    }
    return result;
  }

  /// Deduplicate tracks: of two published tracks closer than
  /// publish_dedupe_distance, keep only one.
  ///
  /// One real person was often published as 2-5 tracks, all clustered around
  /// the person (none farther than 3 m). Each copy drew its own social zone, so
  /// the robot saw a crowd where there was one person.
  ///
  /// Keep the camera-confirmed track first, then the one with more hits. The
  /// dropped track is not deleted from the tracker, only not published this tick.
  std::vector<TrackPtr> dropDuplicateTracks(
    const std::vector<TrackPtr> & tracks, const rclcpp::Time & now)
  {
    if (publish_dedupe_distance_ <= 0.0 || tracks.size() < 2) {
      return tracks;
    }
    // Most trusted first: camera-confirmed, then more hits.
    std::vector<const TrackPtr *> order;
    order.reserve(tracks.size());
    for (const auto & t : tracks) {
      order.push_back(&t);
    }
    std::stable_sort(
      order.begin(), order.end(),
      [](const TrackPtr * a, const TrackPtr * b) {
        const bool ca = (*a)->camera_hits > 0 || (*a)->fusion_hits > 0;
        const bool cb = (*b)->camera_hits > 0 || (*b)->fusion_hits > 0;
        if (ca != cb) {return ca;}
        return (*a)->hits > (*b)->hits;
      });

    const double threshold_sq = publish_dedupe_distance_ * publish_dedupe_distance_;
    std::vector<PublishedPosition> kept_positions;
    std::vector<TrackPtr> result;
    result.reserve(tracks.size());
    for (const TrackPtr * tp : order) {
      const PublishedPosition pos = trackPositionAt(*tp, now.nanoseconds());
      bool duplicate = false;
      for (const auto & other : kept_positions) {
        const double dx = pos.x - other.x;
        const double dy = pos.y - other.y;
        if (dx * dx + dy * dy <= threshold_sq) {duplicate = true; break;}
      }
      if (duplicate) {
        dropped_duplicate_ += 1;
        continue;
      }
      kept_positions.push_back(pos);
      result.push_back(*tp);
    }
    return result;
  }

  void publishTrackedHumans(const std::vector<TrackPtr> & tracks, const rclcpp::Time & now)
  {
    thesis_msgs::msg::TrackedHumans msg;
    msg.header.stamp = now;
    msg.header.frame_id = "odom";

    for (const auto & track : tracks) {
      const PublishedPosition position = trackPositionAt(track, now.nanoseconds());
      const Eigen::Matrix4d covariance = track->covarianceAt(
        position.observation_age, tracker_->params().kf_process_noise_std);

      thesis_msgs::msg::TrackedHuman human;
      human.id = static_cast<uint32_t>(track->track_id);
      human.x = position.x;
      human.y = position.y;
      human.yaw = track->yaw;
      human.vx = track->vx;
      human.vy = track->vy;
      human.speed = track->speed();
      human.is_moving = position.moving;
      human.hits = static_cast<uint32_t>(track->hits);
      human.source = static_cast<uint8_t>(track->source);
      human.mode = static_cast<uint8_t>(track->mode);
      human.covariance_xx = covariance(0, 0);
      human.covariance_xy = covariance(0, 1);
      human.covariance_yy = covariance(1, 1);
      human.observation_age = position.observation_age;
      msg.humans.push_back(human);
    }
    tracked_humans_pub_->publish(msg);
  }

  void publishTrackedHumanMarkers(const std::vector<TrackPtr> & tracks, const rclcpp::Time & now)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = now;
    delete_marker.header.frame_id = "odom";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto & track : tracks) {
      // Second call to trackPositionAt (and isMoving) in the same tick. isMoving has
      // a side effect, so the marker colour and the published is_moving flag can
      // differ. Same behaviour as the Python reference.
      const PublishedPosition position = trackPositionAt(track, now.nanoseconds());

      visualization_msgs::msg::Marker sphere;
      sphere.header.stamp = now;
      sphere.header.frame_id = "odom";
      sphere.ns = "tracked_humans_id";
      sphere.id = track->track_id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = position.x;
      sphere.pose.position.y = position.y;
      sphere.pose.position.z = 0.9;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color.a = 0.8f;
      sphere.color.r = position.moving ? 1.0f : 0.2f;
      sphere.color.g = position.moving ? 0.2f : 1.0f;
      sphere.color.b = 0.2f;
      sphere.lifetime.sec = 1;
      marker_array.markers.push_back(sphere);

      visualization_msgs::msg::Marker text;
      text.header.stamp = now;
      text.header.frame_id = "odom";
      text.ns = "tracked_humans_id";
      text.id = 10000 + track->track_id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = position.x;
      text.pose.position.y = position.y;
      text.pose.position.z = 1.4;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.25;
      text.color.a = 1.0f;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      char buffer[128];
      std::snprintf(
        buffer, sizeof(buffer), "H%d v=%.2fm/s src=%d mode=%d",
        track->track_id, track->speed(), track->source, track->mode);
      text.text = buffer;
      text.lifetime.sec = 1;
      marker_array.markers.push_back(text);
    }

    tracked_humans_markers_pub_->publish(marker_array);
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->get_clock()->now();
    const int64_t now_ns = now.nanoseconds();
    handleClockReset(now_ns);

    const LidarSnapshot snapshot = snapshotLidar(now_ns);

    std::vector<TrackPtr> tracks;
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      const bool new_lidar_scan =
        snapshot.is_fresh && snapshot.generation != last_coasted_lidar_generation_;
      if (new_lidar_scan) {
        // Two different timestamps on purpose: coast uses the scan time, the tick
        // uses now. Do not merge them.
        tracker_->coastWithLidar(snapshot.person_centers, snapshot.stamp_ns);
        tracker_->tick(now_ns);
        last_coasted_lidar_generation_ = snapshot.generation;
      } else {
        tracker_->tick(now_ns);
      }
      tracks = tracker_->getTracks();
    }

    tracks = filterPublishableTracks(tracks);
    tracks = dropTracksOnStaticMap(tracks, now);
    tracks = dropStaleFrozenTracks(tracks, now);
    tracks = dropDuplicateTracks(tracks, now);

    publishTrackedHumans(tracks, now);
    if (publish_tracking_markers_) {
      publishTrackedHumanMarkers(tracks, now);
    }

    logPruneReasons(now_ns);
  }

  /// Every 10 s, report which branch removed tracks and how much memory is held.
  void logPruneReasons(int64_t now_ns)
  {
    if (now_ns - last_prune_log_ns_ < kPruneLogIntervalNs) {
      return;
    }
    last_prune_log_ns_ = now_ns;

    std::string reasons;
    std::size_t memory_size = 0;
    {
      std::lock_guard<std::mutex> lock(tracker_mutex_);
      for (const auto & entry : tracker_->pruneReasons()) {
        if (!reasons.empty()) {
          reasons += ", ";
        }
        reasons += "'" + entry.first + "': " + std::to_string(entry.second);
      }
      memory_size = tracker_->memorySize();
    }
    if (!reasons.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "removed tracks by branch (cumulative): {%s}; tracks held: %zu; "
        "static-obstacle filter: checked %zu, DROPPED %zu, kept as camera-confirmed %zu, "
        "TF failures %zu; ghost footprints dropped %zu; duplicates dropped %zu",
        reasons.c_str(), memory_size, static_checked_, dropped_on_static_map_,
        kept_camera_confirmed_, static_tf_failures_, dropped_stale_frozen_, dropped_duplicate_);
    }
  }

  // ---- state ------------------------------------------------------------

  double update_rate_ = 4.0;
  int occupied_threshold_ = 65;
  std::string scan_topic_ = "/scan";
  double lidar_min_range_ = 0.15;
  double lidar_max_range_ = 8.0;
  double lidar_observation_timeout_ = 0.5;
  bool publish_tracking_markers_ = true;
  double human_lidar_fusion_radius_ = 0.4;
  double human_lidar_sync_tolerance_ = 0.10;
  int lidar_person_cluster_min_points_ = 3;
  double lidar_person_cluster_max_gap_ = 0.15;
  double lidar_person_cluster_max_diameter_ = 0.75;
  double lidar_person_radius_ = 0.28;
  double lidar_person_merge_distance_ = 0.0;
  double lidar_static_rejection_radius_ = 0.15;
  double publish_static_rejection_radius_ = 0.0;
  double max_frozen_publish_age_ = 0.0;
  double publish_dedupe_distance_ = 0.0;
  std::size_t dropped_duplicate_ = 0;
  std::size_t dropped_stale_frozen_ = 0;
  std::size_t dropped_on_static_map_ = 0;
  std::size_t static_checked_ = 0;
  std::size_t kept_camera_confirmed_ = 0;
  std::size_t static_tf_failures_ = 0;
  bool publish_unconfirmed_lidar_only_ = false;
  std::vector<ExclusionZone> lidar_static_exclusion_zones_;

  std::unique_ptr<HumanTracker> tracker_;

  StaticMap static_map_;
  bool has_map_ = false;
  bool logged_first_map_ = false;
  std::mutex map_mutex_;

  std::vector<Point2D> lidar_person_centers_odom_;
  std::size_t lidar_point_count_ = 0;
  int64_t last_lidar_update_ns_ = 0;
  int64_t lidar_generation_ = 0;
  int64_t last_coasted_lidar_generation_ = -1;
  int64_t last_lidar_stamp_ns_ = 0;
  int64_t last_fused_lidar_generation_ = -1;
  std::mutex lidar_mutex_;
  std::mutex tracker_mutex_;

  int64_t last_prune_log_ns_ = 0;
  int64_t last_lidar_tf_warning_ns_ = 0;
  int64_t last_human_tf_warning_ns_ = 0;
  int64_t last_static_filter_tf_warning_ns_ = 0;
  int64_t last_human_measurement_ns_ = 0;
  int64_t last_clock_ns_ = 0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<thesis_msgs::msg::MultiplePoseResult3D>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<thesis_msgs::msg::TrackedHumans>::SharedPtr tracked_humans_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_humans_markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lidar_person_centers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace social_nav_tracking_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<social_nav_tracking_cpp::HumanTrackerNode1>();
  // MultiThreadedExecutor with 2 threads, as in the Python reference. All
  // callbacks are in the node's default mutually exclusive group, so they still
  // run one at a time; the mutexes inside the node are a second safeguard.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
