#include "social_nav_tracking_cpp/human_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include "social_nav_tracking_cpp/assignment.hpp"

namespace social_nav_tracking_cpp
{
namespace
{

constexpr int kStateDim = 4;

/// Measurement matrix: only (x, y) is observed.
Eigen::Matrix<double, 2, 4> measurementMatrix()
{
  Eigen::Matrix<double, 2, 4> H;
  H << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;
  return H;
}

/// Constant-velocity transition F and process noise Q discretised with a
/// white-noise acceleration model over dt. State layout [x, y, vx, vy]: x pairs
/// with vx and y with vy, x and y are independent, so Q only fills the index
/// pairs (0,2) and (1,3).
void cvTransition(
  double dt, double process_noise_std,
  Eigen::Matrix4d & F, Eigen::Matrix4d & Q)
{
  F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  const double q = process_noise_std * process_noise_std;
  const double q00 = q * (dt * dt * dt * dt / 4.0);
  const double q01 = q * (dt * dt * dt / 2.0);
  const double q11 = q * (dt * dt);

  Q = Eigen::Matrix4d::Zero();
  const int pairs[2][2] = {{0, 2}, {1, 3}};
  for (const auto & pair : pairs) {
    const int i = pair[0];
    const int j = pair[1];
    Q(i, i) = q00;
    Q(i, j) = q01;
    Q(j, i) = q01;
    Q(j, j) = q11;
  }
}

/// Globally optimal assignment (Hungarian) between anchors and candidates,
/// accepting only pairs inside each anchor's own gate. Returns
/// {anchor index -> candidate index}.
///
/// `fallback_anchors` allows a second anchor per track (distance = min of the
/// two). Every current caller passes none.
std::map<std::size_t, std::size_t> optimalMatch(
  const std::vector<Point2D> & anchors,
  const std::vector<Point2D> & candidates,
  const std::vector<double> & gates,
  const std::vector<Point2D> * fallback_anchors = nullptr)
{
  std::map<std::size_t, std::size_t> matches;
  if (anchors.empty() || candidates.empty()) {
    return matches;
  }

  double max_gate = 0.0;
  for (const double gate : gates) {
    max_gate = std::max(max_gate, gate);
  }
  // Pairs outside the gate all get the same finite sentinel (not inf), as in
  // the Python reference.
  const double sentinel = max_gate * 1e4 + 1.0;

  CostMatrix cost;
  cost.rows = anchors.size();
  cost.cols = candidates.size();
  cost.data.assign(cost.rows * cost.cols, sentinel);

  std::vector<double> true_distance(cost.rows * cost.cols, sentinel);

  for (std::size_t i = 0; i < anchors.size(); ++i) {
    const double gate = gates[i];
    Point2D fallback = anchors[i];
    if (fallback_anchors != nullptr && i < fallback_anchors->size()) {
      fallback = (*fallback_anchors)[i];
    }
    for (std::size_t j = 0; j < candidates.size(); ++j) {
      const double distance = std::min(
        std::hypot(anchors[i].x - candidates[j].x, anchors[i].y - candidates[j].y),
        std::hypot(fallback.x - candidates[j].x, fallback.y - candidates[j].y));
      true_distance[i * cost.cols + j] = distance;
      if (distance <= gate) {
        cost.at(i, j) = distance;
      }
    }
  }

  for (const auto & pair : solveAssignment(cost)) {
    if (true_distance[pair.first * cost.cols + pair.second] <= gates[pair.first]) {
      matches[pair.first] = pair.second;
    }
  }
  return matches;
}

}  // namespace

// ---------------------------------------------------------------------------
// HumanTrack
// ---------------------------------------------------------------------------

double HumanTrack::speed() const
{
  // (vx**2 + vy**2) ** 0.5 as in Python, not hypot: the two differ in the last
  // digit and this value feeds several threshold comparisons.
  return std::sqrt(vx * vx + vy * vy);
}

bool HumanTrack::isMoving(
  int min_hits,
  double speed_threshold,
  std::optional<int64_t> now_ns,
  double hold_time,
  std::optional<double> exit_speed_threshold)
{
  if (std::max(camera_hits, lidar_hits) < min_hits) {
    return false;
  }

  const double enter_threshold = std::max(0.0, speed_threshold);
  double exit_threshold = enter_threshold;
  if (exit_speed_threshold.has_value()) {
    exit_threshold = std::min(enter_threshold, std::max(0.0, exit_speed_threshold.value()));
  }

  // A standing track must exceed the HIGHER threshold to become moving.
  if (speed() >= enter_threshold) {
    return true;
  }

  // Without motion history, a speed between the two thresholds cannot start
  // the moving state.
  if (!last_moving_ns.has_value()) {
    return false;
  }

  // Once moving, it stays moving while above the LOWER threshold.
  if (speed() >= exit_threshold) {
    return true;
  }

  if (!now_ns.has_value()) {
    return false;
  }

  const double moving_age = static_cast<double>(now_ns.value() - last_moving_ns.value()) / 1e9;
  const bool held = moving_age >= 0.0 && moving_age <= std::max(0.0, hold_time);

  if (!held) {
    // Release the latch; the track must exceed the entry threshold again.
    // Intentional side effect, see the header.
    last_moving_ns.reset();
  }

  return held;
}

std::pair<double, double> HumanTrack::predict(double dt) const
{
  return {x + vx * dt, y + vy * dt};
}

Eigen::Matrix4d HumanTrack::covarianceAt(double dt, double process_noise_std) const
{
  if (!P.has_value()) {
    return Eigen::Matrix4d::Zero();
  }

  const double prediction_dt = std::max(0.0, dt);
  Eigen::Matrix4d F;
  Eigen::Matrix4d Q;
  cvTransition(prediction_dt, process_noise_std, F, Q);

  const Eigen::Matrix4d predicted = F * P.value() * F.transpose() + Q;

  // Symmetrise to remove round-off asymmetry in the covariance.
  return 0.5 * (predicted + predicted.transpose());
}

// ---------------------------------------------------------------------------
// HumanTracker
// ---------------------------------------------------------------------------

HumanTracker::HumanTracker(const HumanTrackerParams & params)
: params_(params)
{
  last_now_ns_ = 0;
}

void HumanTracker::reset()
{
  tracks_.clear();
  memory_.clear();
  next_id_ = 0;
  last_now_ns_ = 0;
  prune_reasons_.clear();
}

TrackPtr HumanTracker::findTrack(int track_id) const
{
  for (const auto & track : tracks_) {
    if (track->track_id == track_id) {
      return track;
    }
  }
  return nullptr;
}

void HumanTracker::adoptTrack(const TrackPtr & track)
{
  // Same semantics as `self._tracks[id] = track` on a Python dict: replace in
  // place if present, otherwise append at the END. The position matters: it is
  // the row order of the Hungarian cost matrix.
  for (auto & existing : tracks_) {
    if (existing->track_id == track->track_id) {
      existing = track;
      return;
    }
  }
  tracks_.push_back(track);
}

void HumanTracker::eraseTrack(int track_id)
{
  for (auto it = tracks_.begin(); it != tracks_.end(); ++it) {
    if ((*it)->track_id == track_id) {
      tracks_.erase(it);
      return;
    }
  }
}

std::vector<TrackPtr> HumanTracker::update(
  const std::vector<Detection> & detections, int64_t now_ns)
{
  const auto matches = associate(detections, now_ns);

  std::set<std::size_t> matched_detections;
  for (const auto & entry : matches) {
    // `matches` is keyed by ascending anchor (= track) index, the order in which
    // the Python dict was filled.
    const TrackPtr track = tracks_[entry.first];
    updateTrack(track, detections[entry.second], now_ns, true);
    matched_detections.insert(entry.second);
  }

  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (matched_detections.count(index) != 0) {
      continue;
    }
    // A person leaving and re-entering the view gets their old id back instead of
    // becoming a new person with zero velocity.
    if (reviveFromMemory(detections[index], now_ns, true) != nullptr) {
      continue;
    }
    const TrackPtr track = createTrack(detections[index], now_ns);
    adoptTrack(track);
  }

  reclassifyAndPrune(now_ns);
  return confirmedTracks();
}

std::vector<TrackPtr> HumanTracker::coastWithLidar(
  const std::vector<Point2D> & raw_lidar_points, int64_t now_ns)
{
  std::vector<Point2D> lidar_points;
  lidar_points.reserve(raw_lidar_points.size());
  for (const auto & point : raw_lidar_points) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      lidar_points.push_back(point);
    }
  }

  const int confirmation_threshold = params_.track_min_hits_to_confirm;

  // `available_indices` is a set in Python but always iterated through
  // sorted(...), so an ascending scan over a flag array is equivalent.
  std::vector<char> available(lidar_points.size(), 1);

  auto match_available =
    [&](const std::vector<int> & track_ids, double gate) {
      std::vector<std::size_t> candidate_indices;
      std::vector<Point2D> candidates;
      for (std::size_t index = 0; index < lidar_points.size(); ++index) {
        if (available[index]) {
          candidate_indices.push_back(index);
          candidates.push_back(lidar_points[index]);
        }
      }

      std::vector<Point2D> anchors;
      anchors.reserve(track_ids.size());
      for (const int track_id : track_ids) {
        const TrackPtr track = findTrack(track_id);
        const double dt = std::max(
          0.0, static_cast<double>(now_ns - track->last_update_ns) / 1e9);
        const auto predicted = track->predict(dt);
        anchors.push_back(Point2D{predicted.first, predicted.second});
      }

      const std::vector<double> gates(anchors.size(), gate);
      const auto matches = optimalMatch(anchors, candidates, gates);

      std::vector<std::pair<int, std::size_t>> resolved;
      resolved.reserve(matches.size());
      for (const auto & entry : matches) {
        resolved.emplace_back(track_ids[entry.first], candidate_indices[entry.second]);
      }
      for (const auto & entry : resolved) {
        available[entry.second] = 0;
      }
      return resolved;
    };

  std::vector<int> fresh_camera_track_ids;
  for (const auto & track : tracks_) {
    if (track->camera_hits > 0 &&
      static_cast<double>(now_ns - track->last_camera_update_ns) / 1e9 <= params_.track_timeout)
    {
      fresh_camera_track_ids.push_back(track->track_id);
    }
  }

  // Associate tracks currently seen by the camera FIRST, so the same lidar
  // cluster cannot spawn a duplicate. The new scan then acts as a second,
  // asynchronous Kalman correction for the same person.
  for (const auto & entry : match_available(
      fresh_camera_track_ids, params_.track_lidar_coast_gate))
  {
    const TrackPtr track = findTrack(entry.first);
    const Point2D & point = lidar_points[entry.second];

    updateTrack(track, Detection{point.x, point.y, track->yaw}, now_ns, false);

    // If the lidar stamp equals the camera stamp, updateTrack does not rewind the
    // filter, but the lidar observation of this track is still recorded.
    track->last_lidar_update_ns = now_ns;

    // --- Camera -> lidar handover ---------------------------------------------
    // Here the camera sees this person fresh and lidar has a cluster at the same
    // position: two independent sensors agree that this is a person.
    //
    // Counted here unconditionally, because updateTrack can return early when the
    // camera stamp is newer and then lidar_hits does not increase.
    //
    // After fusion_confirm_min_hits agreeing ticks, set lidar_only_confirmed:
    // when the person walks into the camera's blind spot the track continues in
    // the LIDAR_ONLY branch for as long as lidar follows it, instead of coasting
    // and being removed by track_coast_timeout. Ghosts (clusters never confirmed by
    // the camera, camera_hits == 0) cannot enter this branch.
    track->fusion_hits += 1;
    if (track->fusion_hits >= params_.fusion_confirm_min_hits) {
      track->lidar_only_confirmed = true;
    }
  }

  std::vector<int> coast_track_ids;
  for (const auto & track : tracks_) {
    if (!track->lidar_only_confirmed &&
      track->camera_hits >= confirmation_threshold &&
      static_cast<double>(now_ns - track->last_camera_update_ns) / 1e9 > params_.track_timeout)
    {
      coast_track_ids.push_back(track->track_id);
    }
  }
  for (const auto & entry : match_available(coast_track_ids, params_.track_lidar_coast_gate)) {
    const TrackPtr track = findTrack(entry.first);
    const Point2D & point = lidar_points[entry.second];
    updateTrack(track, Detection{point.x, point.y, track->yaw}, now_ns, false);
  }

  std::vector<int> lidar_only_track_ids;
  for (const auto & track : tracks_) {
    const bool in_fresh = std::find(
      fresh_camera_track_ids.begin(), fresh_camera_track_ids.end(),
      track->track_id) != fresh_camera_track_ids.end();
    const bool in_coast = std::find(
      coast_track_ids.begin(), coast_track_ids.end(),
      track->track_id) != coast_track_ids.end();
    if (track->lidar_origin_x.has_value() && !in_fresh && !in_coast) {
      lidar_only_track_ids.push_back(track->track_id);
    }
  }
  for (const auto & entry : match_available(
      lidar_only_track_ids, params_.lidar_only_association_gate))
  {
    const TrackPtr track = findTrack(entry.first);
    const Point2D & point = lidar_points[entry.second];
    updateTrack(track, Detection{point.x, point.y, track->yaw}, now_ns, false);
  }

  std::vector<std::size_t> leftover_indices;
  std::vector<Point2D> leftover_points;
  for (std::size_t index = 0; index < lidar_points.size(); ++index) {
    if (available[index]) {
      leftover_indices.push_back(index);
      leftover_points.push_back(lidar_points[index]);
    }
  }

  const auto revived_offsets = reviveUnambiguousLidar(leftover_points, now_ns);
  for (std::size_t offset = 0; offset < leftover_indices.size(); ++offset) {
    if (std::find(revived_offsets.begin(), revived_offsets.end(), offset) !=
      revived_offsets.end())
    {
      continue;
    }
    const Point2D & point = lidar_points[leftover_indices[offset]];
    const TrackPtr track = createLidarTrack(point.x, point.y, now_ns);
    adoptTrack(track);
  }

  reclassifyAndPrune(now_ns);
  return confirmedTracks();
}

std::vector<TrackPtr> HumanTracker::getTracks()
{
  return confirmedTracks();
}

std::vector<TrackPtr> HumanTracker::tick(int64_t now_ns)
{
  reclassifyAndPrune(now_ns);
  return confirmedTracks();
}

std::vector<TrackPtr> HumanTracker::confirmedTracks()
{
  const int threshold = params_.track_min_hits_to_confirm;
  std::vector<TrackPtr> result;
  for (const auto & track : tracks_) {
    if (track->camera_hits >= threshold || track->lidar_only_confirmed) {
      result.push_back(track);
    }
  }
  const auto ghosts = ghostTracks();
  result.insert(result.end(), ghosts.begin(), ghosts.end());
  return result;
}

void HumanTracker::refreshTrackMode(const TrackPtr & track, int64_t now_ns)
{
  const bool camera_fresh =
    track->camera_hits > 0 &&
    static_cast<double>(now_ns - track->last_camera_update_ns) / 1e9 <= params_.track_timeout;

  const bool lidar_fresh =
    track->last_lidar_update_ns.has_value() &&
    static_cast<double>(now_ns - track->last_lidar_update_ns.value()) / 1e9 <=
    params_.lidar_only_track_timeout;

  if (camera_fresh && lidar_fresh) {
    track->mode = kTrackModeFusion;
  } else if (camera_fresh) {
    track->mode = kTrackModeCameraOnly;
  } else if (lidar_fresh) {
    track->mode = kTrackModeLidarOnly;
  } else {
    track->mode = kTrackModePrediction;
  }
}

std::map<std::size_t, std::size_t> HumanTracker::associate(
  const std::vector<Detection> & detections, int64_t now_ns)
{
  std::vector<Point2D> anchors;
  std::vector<double> gates;
  anchors.reserve(tracks_.size());
  gates.reserve(tracks_.size());

  for (const auto & track : tracks_) {
    const double dt = std::max(
      0.0, static_cast<double>(now_ns - track->last_update_ns) / 1e9);
    const auto predicted = track->predict(dt);
    anchors.push_back(Point2D{predicted.first, predicted.second});
    gates.push_back(
      params_.association_gate +
      params_.association_gate_speed_factor * track->speed() * dt);
  }

  // A dual anchor (adding the last measured position as a second anchor) was
  // tried and dropped: it widened the effective gate of every track, so tracks
  // competed for detections and a stale standing track stole the detection of
  // the walking person (more identity switches than the extrapolated anchor alone).
  std::vector<Point2D> candidates;
  candidates.reserve(detections.size());
  for (const auto & detection : detections) {
    candidates.push_back(Point2D{detection.x, detection.y});
  }

  return optimalMatch(anchors, candidates, gates);
}

void HumanTracker::updateTrack(
  const TrackPtr & track, const Detection & detection, int64_t now_ns, bool is_camera)
{
  if (now_ns <= track->last_update_ns) {
    // A detection can arrive AFTER a newer lidar scan because inference is slower
    // than the scan callback. Never rewind the Kalman state, but still let a
    // matched camera observation re-confirm the track; otherwise repeatedly late
    // images could keep it coasting forever.
    if (is_camera && now_ns > track->last_camera_update_ns) {
      track->yaw = detection.yaw;
      track->hits += 1;
      track->camera_hits += 1;
      track->last_camera_update_ns = now_ns;
      track->coast_deadline_ns.reset();
    }
    return;
  }

  const double dt = std::max(0.0, static_cast<double>(now_ns - track->last_update_ns) / 1e9);

  Eigen::Matrix4d F;
  Eigen::Matrix4d Q;
  cvTransition(dt, params_.kf_process_noise_std, F, Q);

  Eigen::Vector4d state;
  state << track->x, track->y, track->vx, track->vy;
  const Eigen::Vector4d state_pred = F * state;
  const Eigen::Matrix4d P_current =
    track->P.has_value() ? track->P.value() : Eigen::Matrix4d::Zero();
  Eigen::Matrix4d P_pred = F * P_current * F.transpose() + Q;

  const double measurement_std = is_camera ?
    params_.kf_measurement_noise_std :
    params_.kf_lidar_measurement_noise_std;
  const double measurement_noise = std::pow(std::max(1e-6, measurement_std), 2);

  const Eigen::Matrix<double, 2, 4> H = measurementMatrix();
  const Eigen::Matrix2d R = measurement_noise * Eigen::Matrix2d::Identity();
  Eigen::Vector2d measurement;
  measurement << detection.x, detection.y;
  const Eigen::Vector2d innovation = measurement - H * state_pred;

  // Reversal: the measurement is behind the prediction -> boost the velocity
  // covariance. See HumanTrackerParams::reversal_* in the header.
  const double speed_pred = std::hypot(state_pred(2), state_pred(3));
  if (speed_pred >= params_.reversal_speed_threshold) {
    const double along =
      (innovation(0) * state_pred(2) + innovation(1) * state_pred(3)) / speed_pred;
    if (along <= -params_.reversal_innovation_threshold) {
      P_pred.block<2, 2>(2, 2) *= params_.reversal_covariance_boost;
    }
  }

  const Eigen::Matrix2d S = H * P_pred * H.transpose() + R;

  // LU decomposition instead of the adjugate formula of `S.inverse()`: here S is
  // always s*I, and the adjugate yields s/(s*s) while np.linalg.inv (LAPACK)
  // yields 1/s, which differ in the last unit. Both are correct, but we must
  // match the Python reference.
  const Eigen::Matrix2d S_inverse = S.partialPivLu().solve(Eigen::Matrix2d::Identity());
  const Eigen::Matrix<double, 4, 2> K = P_pred * H.transpose() * S_inverse;
  const Eigen::Vector4d state_new = state_pred + K * innovation;
  const Eigen::Matrix4d P_new = (Eigen::Matrix4d::Identity() - K * H) * P_pred;

  double vx = state_new(2);
  double vy = state_new(3);
  // Python uses math.hypot here (unlike the speed property, which uses **0.5).
  const double speed = std::hypot(vx, vy);
  if (speed > params_.max_plausible_speed && speed > 0.0) {
    const double scale = params_.max_plausible_speed / speed;
    vx *= scale;
    vy *= scale;
  }

  track->x = state_new(0);
  track->y = state_new(1);
  track->vx = vx;
  track->vy = vy;
  track->P = P_new;
  track->yaw = detection.yaw;
  track->hits += 1;
  track->last_update_ns = now_ns;

  if (is_camera) {
    track->camera_hits += 1;
    track->last_camera_update_ns = now_ns;
  } else {
    track->lidar_hits += 1;
    track->last_lidar_update_ns = now_ns;
    if (!track->lidar_origin_x.has_value()) {
      track->lidar_origin_x = detection.x;
      track->lidar_origin_y = detection.y;
    } else {
      const double displacement = std::hypot(
        detection.x - track->lidar_origin_x.value(),
        detection.y - track->lidar_origin_y.value());
      track->lidar_max_displacement = std::max(track->lidar_max_displacement, displacement);
    }
    const double timeout = (track->camera_hits == 0 || track->lidar_only_confirmed) ?
      params_.lidar_only_track_timeout :
      params_.track_coast_match_timeout;
    track->coast_deadline_ns = now_ns + static_cast<int64_t>(timeout * 1e9);
  }

  const bool enough_observations =
    std::max(track->camera_hits, track->lidar_hits) >= params_.min_hits_for_velocity;

  if (enough_observations && track->speed() >= params_.moving_speed_threshold) {
    track->last_moving_ns = now_ns;
    track->last_motion_yaw = std::atan2(track->vy, track->vx);
    if (track->lidar_origin_x.has_value() && !is_camera) {
      track->yaw = track->last_motion_yaw.value();
    }
  }

  if (track->speed() >= params_.lidar_only_moving_speed_threshold) {
    track->last_lidar_motion_ns = now_ns;
  }

  // The three lidar-only confirmation thresholds must hold together in the same
  // update. The "alive long enough" threshold is in SECONDS when
  // lidar_only_min_duration > 0, otherwise in update counts (see the header:
  // a hit count changes meaning with update_rate).
  const bool da_du_lau =
    params_.lidar_only_min_duration > 0.0 ?
    (static_cast<double>(now_ns - track->created_ns) / 1e9 >=
    params_.lidar_only_min_duration) :
    (track->lidar_hits >= params_.lidar_only_min_hits);

  const bool meets_lidar_only_thresholds =
    params_.lidar_only_enabled &&
    track->lidar_origin_x.has_value() &&
    da_du_lau &&
    track->lidar_max_displacement >= params_.lidar_only_min_displacement &&
    track->speed() >= params_.lidar_only_moving_speed_threshold;

  // Demotion: labelled, demotion enabled, never confirmed by the camera, camera
  // and lidar never agreed, and still for too long. The timer is
  // `last_lidar_motion_ns`, not `last_moving_ns`, which clears itself after
  // ~0.5 s and cannot measure 3 s.
  const bool camera_never_vouched =
    (track->camera_hits < params_.track_min_hits_to_confirm) && (track->fusion_hits == 0);
  const bool idle_past_decay_window =
    track->last_lidar_motion_ns.has_value() &&
    (static_cast<double>(now_ns - track->last_lidar_motion_ns.value()) >
    params_.lidar_only_confirm_decay_sec * 1e9);
  const bool should_decay_lidar_only_label =
    track->lidar_only_confirmed &&
    (params_.lidar_only_confirm_decay_sec > 0.0) &&
    camera_never_vouched &&
    idle_past_decay_window;

  if (meets_lidar_only_thresholds) {
    track->lidar_only_confirmed = true;
  } else if (should_decay_lidar_only_label) {
    // Still for too long and never confirmed by the camera: not a person. The
    // track is kept (it is still useful as an obstacle), it is just no longer
    // published as a PERSON.
    track->lidar_only_confirmed = false;
  }
}

TrackPtr HumanTracker::createTrack(const Detection & detection, int64_t now_ns)
{
  const double position_variance =
    params_.kf_measurement_noise_std * params_.kf_measurement_noise_std;
  const double velocity_variance = params_.max_plausible_speed * params_.max_plausible_speed;

  auto track = std::make_shared<HumanTrack>();
  track->track_id = next_id_;
  track->x = detection.x;
  track->y = detection.y;
  track->yaw = detection.yaw;
  track->hits = 1;
  track->camera_hits = 1;
  track->lidar_hits = 0;
  track->created_ns = now_ns;
  track->last_update_ns = now_ns;
  track->last_camera_update_ns = now_ns;
  track->mode = kTrackModeCameraOnly;
  track->source = kTrackSourceCamera;

  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = position_variance;
  P(1, 1) = position_variance;
  P(2, 2) = velocity_variance;
  P(3, 3) = velocity_variance;
  track->P = P;

  next_id_ += 1;
  return track;
}

TrackPtr HumanTracker::createLidarTrack(double x, double y, int64_t now_ns)
{
  const double position_variance =
    params_.kf_lidar_measurement_noise_std * params_.kf_lidar_measurement_noise_std;
  const double velocity_variance = params_.max_plausible_speed * params_.max_plausible_speed;

  auto track = std::make_shared<HumanTrack>();
  track->track_id = next_id_;
  track->x = x;
  track->y = y;
  track->yaw = 0.0;
  track->hits = 1;
  track->camera_hits = 0;
  track->lidar_hits = 1;
  track->created_ns = now_ns;
  track->last_update_ns = now_ns;
  track->last_camera_update_ns = 0;
  track->mode = kTrackModeLidarOnly;
  track->last_lidar_update_ns = now_ns;
  track->source = kTrackSourceLidarOnly;

  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P(0, 0) = position_variance;
  P(1, 1) = position_variance;
  P(2, 2) = velocity_variance;
  P(3, 3) = velocity_variance;
  track->P = P;

  track->coast_deadline_ns =
    now_ns + static_cast<int64_t>(params_.lidar_only_track_timeout * 1e9);
  track->lidar_origin_x = x;
  track->lidar_origin_y = y;

  next_id_ += 1;
  return track;
}

void HumanTracker::reclassifyAndPrune(int64_t now_ns)
{
  last_now_ns_ = now_ns;
  expireMemory(now_ns);

  std::vector<std::pair<int, std::string>> stale;

  for (const auto & track : tracks_) {
    refreshTrackMode(track, now_ns);

    const double camera_age = track->camera_hits > 0 ?
      static_cast<double>(now_ns - track->last_camera_update_ns) / 1e9 :
      std::numeric_limits<double>::infinity();

    if (track->camera_hits > 0 && camera_age <= params_.track_timeout) {
      track->source = kTrackSourceCamera;
      track->coast_deadline_ns.reset();
      continue;
    }

    // The camera already labelled this track as a person, so lidar may keep
    // feeding it without lidar_only_confirmed, which a STANDING person can never
    // satisfy. See the parameter in the header.
    const bool camera_da_bao_lanh =
      params_.camera_vouched_lidar_hold &&
      track->camera_hits >= params_.track_min_hits_to_confirm;

    if (track->lidar_origin_x.has_value() &&
      (track->camera_hits == 0 || track->lidar_only_confirmed || camera_da_bao_lanh))
    {
      track->source = kTrackSourceLidarOnly;
      const double lidar_age = static_cast<double>(now_ns - track->last_update_ns) / 1e9;
      if (lidar_age > params_.lidar_only_track_timeout) {
        stale.emplace_back(track->track_id, "lidar_only_track_timeout");
      }
      continue;
    }

    if (track->source == kTrackSourceCamera) {
      // Just became eligible for coasting: open a fresh grace window starting NOW,
      // not from last_update_ns.
      track->coast_deadline_ns =
        now_ns + static_cast<int64_t>(params_.track_coast_match_timeout * 1e9);
    }
    track->source = kTrackSourceLidarCoast;

    // The Python reference compares `now_ns > track.coast_deadline_ns` without a
    // None check. The current invariant guarantees a value here, but in C++ a
    // broken invariant would be undefined behaviour, so handle it explicitly:
    // no deadline = not expired.
    if (track->coast_deadline_ns.has_value() && now_ns > track->coast_deadline_ns.value()) {
      stale.emplace_back(track->track_id, "coast_deadline");
    } else if (camera_age > params_.track_coast_timeout) {
      stale.emplace_back(track->track_id, "track_coast_timeout");
    }
  }

  for (const auto & entry : stale) {
    retireTrack(entry.first, now_ns, entry.second);
  }
}

void HumanTracker::retireTrack(int track_id, int64_t now_ns, const std::string & reason)
{
  const TrackPtr track = findTrack(track_id);
  if (track == nullptr) {
    return;
  }
  eraseTrack(track_id);

  prune_reasons_[reason] += 1;
  if (params_.person_memory_ttl <= 0.0) {
    return;
  }

  // CONDITION 1 -- the label must come from the CAMERA. `lidar_only_confirmed` is
  // not enough: wall clusters sliding past a moving robot can satisfy the three
  // lidar thresholds. Only the camera tells people from walls.
  const bool labelled_by_camera =
    track->camera_hits >= params_.track_min_hits_to_confirm || track->fusion_hits > 0;

  // CONDITION 2 -- no chains: a track revived by lidar and never seen again by
  // the camera before dying may not leave a memory record.
  const bool not_chaining =
    track->memory_revivals == 0 || track->camera_hits > track->camera_hits_at_revival;

  if (!(labelled_by_camera && not_chaining)) {
    return;
  }

  bool replaced = false;
  for (auto & entry : memory_) {
    if (entry.track_id == track_id) {
      entry.track = track;
      entry.died_ns = now_ns;
      replaced = true;
      break;
    }
  }
  if (!replaced) {
    memory_.push_back(MemoryEntry{track_id, track, now_ns});
  }

  // Size cap: keep the newest records so the whole room is not remembered as people.
  if (memory_.size() > kMemoryMaxEntries) {
    // Python's `min(...)` returns the FIRST minimum in insertion order on ties;
    // this loop uses `<` to keep that behaviour.
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < memory_.size(); ++i) {
      if (memory_[i].died_ns < memory_[oldest].died_ns) {
        oldest = i;
      }
    }
    memory_.erase(memory_.begin() + static_cast<std::ptrdiff_t>(oldest));
  }
}

void HumanTracker::expireMemory(int64_t now_ns)
{
  const double ttl = params_.person_memory_ttl;
  for (auto it = memory_.begin(); it != memory_.end(); ) {
    const double dt = static_cast<double>(now_ns - it->died_ns) / 1e9;
    if (!(dt >= 0.0 && dt <= ttl)) {
      it = memory_.erase(it);
    } else {
      ++it;
    }
  }
}

double HumanTracker::memoryRadius(double dt) const
{
  // The memory radius grows with the maximum walking speed: after dt seconds the
  // person cannot be farther than base + growth_speed * dt. Capped at radius_max.
  return std::min(
    params_.person_memory_radius_max,
    params_.person_memory_radius_base + params_.person_memory_growth_speed * std::max(0.0, dt));
}

std::optional<std::size_t> HumanTracker::memoryLookup(double x, double y, int64_t now_ns) const
{
  // Match against BOTH the last observed position and its constant-velocity
  // extrapolation, so a person who stopped where they were lost and one who kept
  // walking are both recovered. Scores are normalised by radius so a newer
  // record (narrow zone) beats an older one (wide zone) when both cover a point.
  std::optional<std::size_t> best_index;
  std::optional<double> best_score;

  for (std::size_t i = 0; i < memory_.size(); ++i) {
    const double dt = static_cast<double>(now_ns - memory_[i].died_ns) / 1e9;
    if (!(dt >= 0.0 && dt <= params_.person_memory_ttl)) {
      continue;
    }
    const double radius = memoryRadius(dt);
    const auto predicted = memory_[i].track->predict(dt);
    const double distance = std::min(
      std::hypot(x - memory_[i].track->x, y - memory_[i].track->y),
      std::hypot(x - predicted.first, y - predicted.second));
    if (distance > radius) {
      continue;
    }
    const double score = distance / radius;
    // Strict `<`: on ties the record inserted first wins, as in Python.
    if (!best_score.has_value() || score < best_score.value()) {
      best_index = i;
      best_score = score;
    }
  }
  return best_index;
}

TrackPtr HumanTracker::reviveFromMemory(
  const Detection & detection, int64_t now_ns, bool is_camera, std::optional<int> only_id)
{
  // Return the old HumanTrack object itself, so id, vx, vy, hits and covariance
  // are kept. updateTrack handles the Kalman part (F, Q for the elapsed dt), so
  // the uncertainty grows with the time the person was lost.
  if (params_.person_memory_ttl <= 0.0 || memory_.empty()) {
    return nullptr;
  }

  std::optional<std::size_t> index;
  if (only_id.has_value()) {
    for (std::size_t i = 0; i < memory_.size(); ++i) {
      if (memory_[i].track_id == only_id.value()) {
        index = i;
        break;
      }
    }
  } else {
    index = memoryLookup(detection.x, detection.y, now_ns);
  }
  if (!index.has_value()) {
    return nullptr;
  }

  const TrackPtr track = memory_[index.value()].track;
  const int track_id = memory_[index.value()].track_id;
  memory_.erase(memory_.begin() + static_cast<std::ptrdiff_t>(index.value()));

  // The "person" label is inherited: the cluster does not have to pass the three
  // lidar_only_* thresholds again, and the track stays in the LIDAR_ONLY branch
  // of reclassifyAndPrune instead of the shorter coast branch.
  track->lidar_only_confirmed = true;
  track->coast_deadline_ns.reset();
  track->memory_revivals += 1;
  track->camera_hits_at_revival = track->camera_hits;

  // The track was removed from tracks_ when it retired, so this is a NEW
  // insertion and it goes to the END of the iteration order, which changes its
  // row in the Hungarian matrix. Kept identical to Python.
  (void)track_id;
  adoptTrack(track);
  updateTrack(track, detection, now_ns, is_camera);
  return track;
}

std::vector<std::size_t> HumanTracker::reviveUnambiguousLidar(
  const std::vector<Point2D> & points, int64_t now_ns)
{
  // Revive an old id from a leftover cluster ONLY when it is unambiguous. Taking
  // simply the nearest record let a person's label jump to an adjacent wall
  // cluster. Lidar cannot tell people from walls, so when a memory zone covers
  // more than one cluster (or a cluster lies in several zones), do not guess.
  std::vector<std::size_t> revived;
  if (params_.person_memory_ttl <= 0.0 || memory_.empty() || points.empty()) {
    return revived;
  }

  // Auxiliary table keeping first-touch order, like a Python dict.
  std::vector<std::pair<int, std::vector<std::size_t>>> candidates;
  std::vector<std::pair<std::size_t, std::vector<int>>> owners;

  auto candidate_slot = [&candidates](int track_id) -> std::vector<std::size_t> & {
      for (auto & entry : candidates) {
        if (entry.first == track_id) {
          return entry.second;
        }
      }
      candidates.emplace_back(track_id, std::vector<std::size_t>{});
      return candidates.back().second;
    };
  auto owner_slot = [&owners](std::size_t point_index) -> std::vector<int> & {
      for (auto & entry : owners) {
        if (entry.first == point_index) {
          return entry.second;
        }
      }
      owners.emplace_back(point_index, std::vector<int>{});
      return owners.back().second;
    };

  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    for (const auto & entry : memory_) {
      const double dt = static_cast<double>(now_ns - entry.died_ns) / 1e9;
      if (!(dt >= 0.0 && dt <= params_.person_memory_ttl)) {
        continue;
      }
      const double radius = memoryRadius(dt);
      const auto predicted = entry.track->predict(dt);
      const double distance = std::min(
        std::hypot(points[point_index].x - entry.track->x,
        points[point_index].y - entry.track->y),
        std::hypot(points[point_index].x - predicted.first,
        points[point_index].y - predicted.second));
      if (distance <= radius) {
        candidate_slot(entry.track_id).push_back(point_index);
        owner_slot(point_index).push_back(entry.track_id);
      }
    }
  }

  for (const auto & entry : candidates) {
    if (entry.second.size() != 1) {
      continue;  // one remembered person, several clusters -> do not guess
    }
    const std::size_t point_index = entry.second[0];
    std::size_t owner_count = 0;
    for (const auto & owner : owners) {
      if (owner.first == point_index) {
        owner_count = owner.second.size();
        break;
      }
    }
    if (owner_count != 1) {
      continue;  // one cluster, several remembered people -> do not guess
    }
    bool still_in_memory = false;
    for (const auto & memory_entry : memory_) {
      if (memory_entry.track_id == entry.first) {
        still_in_memory = true;
        break;
      }
    }
    if (!still_in_memory) {
      continue;
    }
    const Detection detection{points[point_index].x, points[point_index].y, 0.0};
    if (reviveFromMemory(detection, now_ns, false, entry.first) != nullptr) {
      revived.push_back(point_index);
    }
  }
  return revived;
}

std::vector<TrackPtr> HumanTracker::ghostTracks()
{
  // Ghost tracks for MOVING people just lost by both camera and lidar. A lost
  // standing person is only remembered; a walking one is still published in
  // PREDICTION mode with a growing covariance, so AghpmLayer1 (with
  // social_covariance_enabled) draws a social zone that widens and fades.
  const double min_speed = params_.ghost_publish_min_speed;
  const bool include_stationary = params_.ghost_publish_stationary;
  std::vector<TrackPtr> ghosts;
  if (memory_.empty() || (min_speed <= 0.0 && !include_stationary)) {
    return ghosts;
  }
  for (const auto & entry : memory_) {
    const double dt = static_cast<double>(last_now_ns_ - entry.died_ns) / 1e9;
    if (!(dt >= 0.0 && dt <= params_.person_memory_ttl)) {
      continue;
    }
    // A person who STOPPED is also kept when ghost_publish_stationary is set:
    // stopping to turn around is the most dangerous moment. See the header.
    if (!include_stationary && entry.track->speed() < min_speed) {
      continue;
    }
    // Modify the object stored in memory directly, as in Python.
    entry.track->source = kTrackSourceLidarCoast;
    entry.track->mode = kTrackModePrediction;
    ghosts.push_back(entry.track);
  }
  return ghosts;
}

}  // namespace social_nav_tracking_cpp
