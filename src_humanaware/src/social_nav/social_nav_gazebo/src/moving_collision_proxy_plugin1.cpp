// Gazebo Harmonic system plugin that makes walking actors visible to sensors.
//
// Actor meshes have no collision geometry, so lidar and depth camera cannot
// see them. Every tick this plugin moves an invisible cylinder ("proxy") to the
// actor's (x, y) position.
//
// The proxy model must be declared <static>true</static> in the SDF; otherwise
// physics owns its pose and fights with the pose written in PreUpdate.
//
// The actor's position is NOT read from the ECM: for actors driven by an SDF
// <script>, components::Pose stays at the static SDF pose,
// components::TrajectoryPose only exists for manually driven actors, and
// components::WorldPose depends on the rendering system. Instead the plugin
// reads the <script>/<trajectory> waypoints from the actor's sdf::Actor and
// interpolates the position over sim time. This matches the designed path
// exactly and does not depend on the GPU. If the target entity has no
// <script>, it falls back to WorldPose / TrajectoryPose.
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>

namespace social_nav
{

class MovingCollisionProxyPlugin1 :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*eventMgr*/) override
  {
    proxy_entity_ = entity;
    if (sdf->HasElement("actor_name")) {
      actor_name_ = sdf->Get<std::string>("actor_name");
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!ResolveActor(ecm)) {
      return;  // actor not in the ECM yet, retry next tick
    }
    if (!traj_loaded_) {
      LoadTrajectory(ecm);
      traj_loaded_ = true;
    }

    gz::math::Pose3d actor_pose;
    if (traj_valid_) {
      const double sim_sec =
        std::chrono::duration<double>(info.simTime).count();
      actor_pose = actor_origin_ * SampleTrajectory(sim_sec);
    } else if (have_cached_pose_) {
      actor_pose = cached_actor_pose_;  // fallback (see PostUpdate)
    } else {
      return;  // no reliable pose source yet -- KEEP the pose
               // written in the SDF; never write (0,0,0) onto the robot's position
    }

    // Only x, y: the proxy stays upright and follows the ground position.
    const gz::math::Pose3d proxy_pose(
      actor_pose.Pos().X(), actor_pose.Pos().Y(), 0.0, 0.0, 0.0, 0.0);

    ecm.SetComponentData<gz::sim::components::Pose>(proxy_entity_, proxy_pose);
    ecm.SetChanged(
      proxy_entity_, gz::sim::components::Pose::typeId,
      gz::sim::ComponentState::OneTimeChange);
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & /*info*/,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (traj_valid_ || actor_entity_ == gz::sim::kNullEntity) {
      return;  // interpolated from the script, no fallback needed
    }
    // Fallback: actor_name refers to an entity without <script>. Read the pose
    // written by other systems (WorldPose for rendered actors, TrajectoryPose for
    // manually driven actors, otherwise a regular model). Read in PostUpdate
    // because those systems write after PreUpdate; used on the next tick.
    const auto * world_pose =
      ecm.Component<gz::sim::components::WorldPose>(actor_entity_);
    if (world_pose != nullptr) {
      cached_actor_pose_ = world_pose->Data();
    } else {
      const auto * trajectory_pose =
        ecm.Component<gz::sim::components::TrajectoryPose>(actor_entity_);
      cached_actor_pose_ = (trajectory_pose != nullptr) ?
        trajectory_pose->Data() : gz::sim::worldPose(actor_entity_, ecm);
    }
    have_cached_pose_ = true;
  }

private:
  /// Find (or re-find) the actor entity by name. Returns false if not present.
  /// If the cached entity was removed from the ECM, forget its state, otherwise
  /// the proxy would follow a trajectory of an entity that no longer exists.
  bool ResolveActor(const gz::sim::EntityComponentManager & ecm)
  {
    if (actor_entity_ != gz::sim::kNullEntity && !ecm.HasEntity(actor_entity_)) {
      actor_entity_ = gz::sim::kNullEntity;
      traj_loaded_ = false;
      traj_valid_ = false;
      have_cached_pose_ = false;
      waypoints_.clear();
    }
    if (actor_entity_ == gz::sim::kNullEntity) {
      actor_entity_ = ecm.EntityByComponents(gz::sim::components::Name(actor_name_));
    }
    return actor_entity_ != gz::sim::kNullEntity;
  }

  /// Read <script>/<trajectory> from sdf::Actor once and cache the waypoints
  /// (time + pose). Only the first trajectory is used (one per actor here).
  void LoadTrajectory(const gz::sim::EntityComponentManager & ecm)
  {
    const auto * actor_comp =
      ecm.Component<gz::sim::components::Actor>(actor_entity_);
    if (actor_comp == nullptr) {
      return;  // not an <actor>: use the PostUpdate fallback
    }
    const sdf::Actor & actor = actor_comp->Data();
    actor_origin_ = actor.RawPose();
    loop_ = actor.ScriptLoop();
    delay_start_ = actor.ScriptDelayStart();
    auto_start_ = actor.ScriptAutoStart();

    if (actor.TrajectoryCount() == 0) {
      return;
    }
    const sdf::Trajectory * traj = actor.TrajectoryByIndex(0);
    if (traj == nullptr || traj->WaypointCount() < 2) {
      return;
    }
    for (uint64_t i = 0; i < traj->WaypointCount(); ++i) {
      const sdf::Waypoint * wp = traj->WaypointByIndex(i);
      if (wp != nullptr) {
        waypoints_.emplace_back(wp->Time(), wp->Pose());
      }
    }
    if (waypoints_.size() < 2) {
      waypoints_.clear();
      return;
    }
    traj_duration_ = waypoints_.back().first - waypoints_.front().first;
    traj_valid_ = traj_duration_ > 1e-6;
  }

  /// Linear interpolation of position (and Slerp of orientation) between the two
  /// waypoints around the current time. gz-sim uses a tension spline; linear is
  /// sufficient for a moving obstacle.
  gz::math::Pose3d SampleTrajectory(double sim_sec) const
  {
    double t = sim_sec - delay_start_;
    if (!auto_start_ || t <= 0.0) {
      return waypoints_.front().second;
    }
    const double t_start = waypoints_.front().first;
    if (loop_) {
      t = t_start + std::fmod(t, traj_duration_);
    } else if (t >= waypoints_.back().first) {
      return waypoints_.back().second;
    }

    for (size_t i = 1; i < waypoints_.size(); ++i) {
      if (t <= waypoints_[i].first) {
        const auto & a = waypoints_[i - 1];
        const auto & b = waypoints_[i];
        const double span = b.first - a.first;
        const double ratio = (span > 1e-9) ? (t - a.first) / span : 0.0;
        return gz::math::Pose3d(
          a.second.Pos() + (b.second.Pos() - a.second.Pos()) * ratio,
          gz::math::Quaterniond::Slerp(ratio, a.second.Rot(), b.second.Rot(), true));
      }
    }
    return waypoints_.back().second;
  }

  std::string actor_name_{"actor"};
  gz::sim::Entity proxy_entity_{gz::sim::kNullEntity};
  gz::sim::Entity actor_entity_{gz::sim::kNullEntity};

  // Trajectory interpolated from the SDF (main path)
  std::vector<std::pair<double, gz::math::Pose3d>> waypoints_;
  gz::math::Pose3d actor_origin_{gz::math::Pose3d::Zero};
  double traj_duration_{0.0};
  double delay_start_{0.0};
  bool loop_{true};
  bool auto_start_{true};
  bool traj_loaded_{false};
  bool traj_valid_{false};

  // Fallback (entity without <script>)
  gz::math::Pose3d cached_actor_pose_{gz::math::Pose3d::Zero};
  bool have_cached_pose_{false};
};

}  // namespace social_nav

GZ_ADD_PLUGIN(
  social_nav::MovingCollisionProxyPlugin1,
  gz::sim::System,
  social_nav::MovingCollisionProxyPlugin1::ISystemConfigure,
  social_nav::MovingCollisionProxyPlugin1::ISystemPreUpdate,
  social_nav::MovingCollisionProxyPlugin1::ISystemPostUpdate)
