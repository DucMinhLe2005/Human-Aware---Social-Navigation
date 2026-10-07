// Publishes the ground-truth pose of the models listed in the SDF, so that
// social_nav_metrics can measure collisions, time and visibility without
// looking at Gazebo.
//
// A dedicated plugin is needed because the SceneBroadcaster Pose_V has no
// header data, so ros_gz_bridge outputs transforms with empty child_frame_id.
// Here frame_id and child_frame_id are written explicitly. /odom/unfiltered
// cannot serve as ground truth because wheel odometry drifts.
//
// Read in PostUpdate (after physics), so the robot pose is from the physics
// step just executed. The pedestrian proxy is the static model positioned by
// MovingCollisionProxyPlugin1 in PreUpdate -- the shape physics uses for contacts.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace social_nav
{

class GroundTruthPosePublisher1 :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & /*entity*/,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & /*ecm*/,
    gz::sim::EventManager & /*eventMgr*/) override
  {
    if (sdf->HasElement("topic")) {
      topic_ = sdf->Get<std::string>("topic");
    }
    if (sdf->HasElement("update_rate")) {
      const double rate = sdf->Get<double>("update_rate");
      if (rate > 0.0) {
        period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rate));
      }
    }
    for (auto elem = sdf->FindElement("model"); elem; elem = elem->GetNextElement("model")) {
      names_.push_back(elem->Get<std::string>());
    }
    entities_.assign(names_.size(), gz::sim::kNullEntity);
    publisher_ = node_.Advertise<gz::msgs::Pose_V>(topic_);
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    // Sim time went backwards (world reset): publish immediately.
    if (have_published_ && info.simTime >= last_publish_ &&
      info.simTime - last_publish_ < period_)
    {
      return;
    }
    last_publish_ = info.simTime;
    have_published_ = true;

    const gz::msgs::Time stamp = gz::msgs::Convert(info.simTime);
    gz::msgs::Pose_V msg;
    *msg.mutable_header()->mutable_stamp() = stamp;

    for (size_t i = 0; i < names_.size(); ++i) {
      // Retry every tick until found: the robot is spawned AFTER the world starts.
      if (entities_[i] == gz::sim::kNullEntity || !ecm.HasEntity(entities_[i])) {
        entities_[i] = ecm.EntityByComponents(
          gz::sim::components::Name(names_[i]), gz::sim::components::Model());
      }
      if (entities_[i] == gz::sim::kNullEntity) {
        continue;
      }
      auto * pose = msg.add_pose();
      pose->set_name(names_[i]);
      auto * header = pose->mutable_header();
      *header->mutable_stamp() = stamp;
      auto * frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value("world");
      auto * child = header->add_data();
      child->set_key("child_frame_id");
      child->add_value(names_[i]);
      gz::msgs::Set(pose, gz::sim::worldPose(entities_[i], ecm));
    }
    publisher_.Publish(msg);
  }

private:
  std::string topic_{"/social_nav/ground_truth"};
  std::chrono::steady_clock::duration period_{std::chrono::milliseconds(50)};
  std::chrono::steady_clock::duration last_publish_{0};
  bool have_published_{false};
  std::vector<std::string> names_;
  std::vector<gz::sim::Entity> entities_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};

}  // namespace social_nav

GZ_ADD_PLUGIN(
  social_nav::GroundTruthPosePublisher1,
  gz::sim::System,
  social_nav::GroundTruthPosePublisher1::ISystemConfigure,
  social_nav::GroundTruthPosePublisher1::ISystemPostUpdate)
