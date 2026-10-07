// Multi-frame person tracker: optimal assignment (Hungarian) + constant-velocity
// Kalman filter. C++ port of social_nav_tracking/human_tracking.py.
//
// This file does not depend on rclcpp so it can be tested without a ROS graph;
// the node (human_tracker_node.cpp) is a thin wrapper around it.

#ifndef SOCIAL_NAV_TRACKING_CPP__HUMAN_TRACKING_HPP_
#define SOCIAL_NAV_TRACKING_CPP__HUMAN_TRACKING_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// cpplint mistakes Eigen/Dense for a C system header; this order is correct
// (third-party headers after the standard library).
#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "social_nav_tracking_cpp/lidar_clustering.hpp"

namespace social_nav_tracking_cpp
{

// These constants must match thesis_msgs/TrackedHuman.msg.
constexpr int kTrackSourceCamera = 0;
constexpr int kTrackSourceLidarCoast = 1;
constexpr int kTrackSourceLidarOnly = 2;

constexpr int kTrackModeFusion = 0;
constexpr int kTrackModeCameraOnly = 1;
constexpr int kTrackModeLidarOnly = 2;
constexpr int kTrackModePrediction = 3;

// Upper bound on person-memory records. A real room rarely has this many people
// lost at the same time; the cap stops a pathological case where the whole room
// is remembered as people and every passing lidar cluster is revived.
constexpr std::size_t kMemoryMaxEntries = 8;

/// One camera measurement: position in odom plus body heading.
struct Detection
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class HumanTrack
{
public:
  int track_id = 0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  int hits = 1;
  int camera_hits = 1;
  int lidar_hits = 0;
  int64_t created_ns = 0;
  int64_t last_update_ns = 0;
  int64_t last_camera_update_ns = 0;
  std::optional<int64_t> last_lidar_update_ns;

  /// Last measurement whose filtered speed exceeded `moving_speed_threshold`.
  std::optional<int64_t> last_moving_ns;

  /// Last time the track exceeded the lidar-only speed threshold. Kept separate
  /// from `last_moving_ns`, which isMoving() clears after the hold time (~0.5 s)
  /// and therefore cannot serve as the 3 s demotion timer.
  std::optional<int64_t> last_lidar_motion_ns;
  std::optional<double> last_motion_yaw;

  int source = kTrackSourceCamera;
  int mode = kTrackModeCameraOnly;
  std::optional<Eigen::Matrix4d> P;

  /// Deadline by which a coasting track must be re-associated (camera or lidar)
  /// or it is removed. Set when the track starts coasting and refreshed after
  /// every successful lidar association. Not derived from `last_update_ns`, which
  /// still points to the last CAMERA update and is already older than
  /// `track_timeout`, so it would expire the shorter coast timeout immediately.
  std::optional<int64_t> coast_deadline_ns;

  /// A lidar-only track is not published until its motion over several frames
  /// shows the compact cluster is not an unmapped static feature.
  std::optional<double> lidar_origin_x;
  std::optional<double> lidar_origin_y;
  double lidar_max_displacement = 0.0;
  bool lidar_only_confirmed = false;

  /// Consecutive ticks in which camera and lidar report the person at the same
  /// position. Counted separately from `lidar_hits`, which only increments when
  /// the lidar stamp is newer than `last_update_ns`; with a faster camera that
  /// rarely happens and a well-tracked person would hardly ever be confirmed.
  int fusion_hits = 0;

  /// Number of revivals from person memory, and `camera_hits` at the last
  /// revival. Prevents chains: a track revived by lidar that is not re-confirmed
  /// by the camera may not enter the memory again.
  int memory_revivals = 0;
  int camera_hits_at_revival = 0;

  /// Uses `sqrt(vx^2 + vy^2)`, not `hypot`, to match the Python reference bit for bit.
  double speed() const;

  bool isCoasting() const {return source == kTrackSourceLidarCoast;}
  bool isLidarOnly() const {return source == kTrackSourceLidarOnly;}

  /// WARNING: side effect -- clears `last_moving_ns` once the hold time expires
  /// (as in the Python reference, covered by tests). The node calls it twice per
  /// tick, so the second call can see the cleared state. Do not make it const.
  bool isMoving(
    int min_hits,
    double speed_threshold,
    std::optional<int64_t> now_ns = std::nullopt,
    double hold_time = 0.0,
    std::optional<double> exit_speed_threshold = std::nullopt);

  /// Constant-velocity extrapolation: p(t) = p + v * t.
  std::pair<double, double> predict(double dt) const;

  /// Predicted state covariance `dt` seconds after the last Kalman update.
  Eigen::Matrix4d covarianceAt(double dt, double process_noise_std) const;
};

struct HumanTrackerParams
{
  double association_gate = 0.75;

  /// Widen the association gate with the track's filtered speed times the time
  /// since its last update. A fixed gate assumes every track is re-detected each
  /// frame; a slow detector cycle or a change of speed/heading moves the
  /// extrapolated anchor outside it and spawns a new track instead.
  double association_gate_speed_factor = 0.5;
  int min_hits_for_velocity = 3;
  double max_plausible_speed = 2.0;
  double moving_speed_threshold = 0.20;
  double moving_exit_speed_threshold = 0.10;
  double moving_hold_time = 0.5;
  double track_timeout = 0.35;

  // Kalman noise model (constant-velocity state [x, y, vx, vy]).
  double kf_process_noise_std = 1.0;
  double kf_measurement_noise_std = 0.15;
  double kf_lidar_measurement_noise_std = 0.20;

  /// Reversal handling. A constant-velocity Kalman filter carries the old
  /// momentum when a person turns around, dragging the track past them and
  /// sometimes losing the id. When a measurement falls BEHIND the prediction
  /// along the walking direction, the velocity covariance is multiplied by
  /// `reversal_covariance_boost` so the velocity flips within 1-2 cycles and the
  /// id is kept. boost = 1.0 disables it.
  double reversal_speed_threshold = 0.25;
  double reversal_innovation_threshold = 0.10;
  double reversal_covariance_boost = 9.0;

  /// A new track is not published until it has this many hits. It still ages
  /// under the normal track_timeout / coast rules, so a short miss does not reset
  /// the id and a one-off false detection expires without ever being published.
  int track_min_hits_to_confirm = 3;

  // Lidar coasting: keep a track alive for a while after the camera loses it, as
  // long as raw lidar can still associate it.
  double track_lidar_coast_gate = 0.5;
  double track_coast_match_timeout = 0.5;
  double track_coast_timeout = 3.0;

  /// Three confirmation thresholds for lidar-only tracks. Looser values let
  /// lidar cluster jitter satisfy them within a few cycles and label ghosts.
  double lidar_only_association_gate = 0.25;
  int lidar_only_min_hits = 8;
  double lidar_only_min_displacement = 0.30;

  /// Time (s) a lidar-only track must live before it is published. When > 0 it
  /// replaces lidar_only_min_hits; <= 0 keeps the hit-count behaviour.
  ///
  /// A hit count depends on the update rate (15 hits are 3.75 s at 4 Hz but
  /// 1.5 s at 10 Hz), so raising the rate silently loosens the ghost filter.
  /// A duration is rate-independent. This value is also the delay before a
  /// person seen only by lidar is recognised.
  double lidar_only_min_duration = 0.0;

  /// Let lidar keep feeding a track the CAMERA has already confirmed, without
  /// requiring lidar_only_confirmed. Default false.
  ///
  /// lidar_only_confirmed requires both displacement and speed, which a standing
  /// person never satisfies. Once the camera loses them (out of view, or beyond
  /// 5 m) their track dies and new lidar tracks there are filtered out, so the
  /// standing person disappears from /planning/tracked_humans along with their
  /// social zone. This does not loosen the ghost filter: the camera must still
  /// confirm the person first.
  bool camera_vouched_lidar_hold = false;

  /// Also publish ghost tracks of people who have STOPPED, not only walking ones.
  ///
  /// A dangerous case is a person who walks past the robot and stops to turn
  /// around: their speed drops to ~0, they vanish from the message, the robot
  /// treats the space as free and the person walks back into it.
  ///
  /// Still bounded by person_memory_ttl, and the person memory still requires
  /// camera confirmation (see rememberTrack), so the ghost filter is not loosened.
  bool ghost_publish_stationary = false;
  double lidar_only_moving_speed_threshold = 0.20;

  /// Disable the lidar-only branch entirely (camera only). Useful for clean
  /// navigation tests or very cluttered environments.
  bool lidar_only_enabled = true;

  /// Revoke the "person" label when a lidar-only track stays still longer than
  /// this. Without it a confirmed lidar-only track is never demoted. Only applies
  /// to tracks not confirmed by the camera. 0.0 disables demotion.
  double lidar_only_confirm_decay_sec = 3.0;

  /// Co-located camera and lidar ticks needed to confirm a person for good.
  int fusion_confirm_min_hits = 3;
  double lidar_only_track_timeout = 0.5;

  // --- Spatial person memory ----------------------------------------------
  // Without it an expired track is deleted and the reappearing cluster gets a
  // NEW id with vx = vy = 0, while the controller predicts people from vx, vy.
  // Remembering recently lost people keeps ids (and velocities) stable.
  double person_memory_ttl = 5.0;
  double person_memory_radius_base = 0.5;
  double person_memory_radius_max = 2.5;
  double person_memory_growth_speed = 1.5;

  /// While a person is lost, only MOVING tracks keep being published as a ghost
  /// in PREDICTION mode, so AGHPM keeps a social zone that fades with the
  /// covariance. A lost standing person is only remembered. 0.0 disables ghosts.
  double ghost_publish_min_speed = 0.3;
};

/// One person-memory record: a dead track and its time of death.
struct MemoryEntry
{
  int track_id = 0;
  std::shared_ptr<HumanTrack> track;
  int64_t died_ns = 0;
};

using TrackPtr = std::shared_ptr<HumanTrack>;

/// NOT thread-safe: the owning node must lock, as in the Python reference.
class HumanTracker
{
public:
  explicit HumanTracker(const HumanTrackerParams & params);

  /// Camera path: associate -> update -> revive from memory or create ->
  /// reclassify / prune -> return confirmed tracks.
  std::vector<TrackPtr> update(const std::vector<Detection> & detections, int64_t now_ns);

  /// Lidar path: three association passes plus leftovers.
  std::vector<TrackPtr> coastWithLidar(const std::vector<Point2D> & lidar_points, int64_t now_ns);

  /// Confirmed tracks, without ageing them.
  std::vector<TrackPtr> getTracks();

  /// Age / prune when no sensor input arrives.
  std::vector<TrackPtr> tick(int64_t now_ns);

  /// Clear everything after the simulation clock was reset.
  void reset();

  const HumanTrackerParams & params() const {return params_;}

  /// Track removal reasons, for the node's periodic diagnostics.
  const std::map<std::string, int> & pruneReasons() const {return prune_reasons_;}
  std::size_t memorySize() const {return memory_.size();}

  // ---- Internals exposed for testing -----------------------------------------
  // One-to-one with the underscore methods of human_tracking.py, which the
  // Python tests call directly; they are part of the tested contract.
  TrackPtr createTrack(const Detection & detection, int64_t now_ns);
  TrackPtr createLidarTrack(double x, double y, int64_t now_ns);
  void updateTrack(
    const TrackPtr & track, const Detection & detection, int64_t now_ns,
    bool is_camera);
  /// Equivalent of `self._tracks[track.track_id] = track`: replace in place if
  /// present, otherwise append to the END of the iteration order.
  void adoptTrack(const TrackPtr & track);
  const std::vector<TrackPtr> & tracksForTest() const {return tracks_;}
  const std::vector<MemoryEntry> & memoryForTest() const {return memory_;}

private:
  std::vector<TrackPtr> confirmedTracks();
  void refreshTrackMode(const TrackPtr & track, int64_t now_ns);
  std::map<std::size_t, std::size_t> associate(
    const std::vector<Detection> & detections, int64_t now_ns);
  void reclassifyAndPrune(int64_t now_ns);
  void retireTrack(int track_id, int64_t now_ns, const std::string & reason);
  void expireMemory(int64_t now_ns);
  double memoryRadius(double dt) const;
  std::optional<std::size_t> memoryLookup(double x, double y, int64_t now_ns) const;
  TrackPtr reviveFromMemory(
    const Detection & detection, int64_t now_ns, bool is_camera,
    std::optional<int> only_id = std::nullopt);
  std::vector<std::size_t> reviveUnambiguousLidar(
    const std::vector<Point2D> & points, int64_t now_ns);
  std::vector<TrackPtr> ghostTracks();

  TrackPtr findTrack(int track_id) const;
  void eraseTrack(int track_id);

  HumanTrackerParams params_;
  /// INSERTION order, not id order. It becomes the row order of the Hungarian
  /// cost matrix and decides ties between equal-cost assignments; Python dicts
  /// iterate in insertion order, so a std::map would silently change matching.
  std::vector<TrackPtr> tracks_;
  std::vector<MemoryEntry> memory_;
  int next_id_ = 0;
  int64_t last_now_ns_ = 0;
  std::map<std::string, int> prune_reasons_;
};

}  // namespace social_nav_tracking_cpp

#endif  // SOCIAL_NAV_TRACKING_CPP__HUMAN_TRACKING_HPP_
