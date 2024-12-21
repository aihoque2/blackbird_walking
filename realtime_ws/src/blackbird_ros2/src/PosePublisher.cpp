#include "PosePublisher.h"

namespace blackbird_ros2{

    BlackbirdPosePublisher::BlackbirdPosePublisher(){
        node_ = rclcpp::Node::make_shared("pose_publisher");
        pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("torso_pose", 10);
        x = 0.0;
        y = 0.0;
        z = 0.0;
        r = 0.0;
        p = 0.0;
        w = 0.0;
        msg_.resize(35); // state space is a 35-vector


    }

    vector<bool> BlackbirdPosePublisher::GetContacts(const gz::sim::EntityComponentManager &ecm){
        /*TODO*/
    }

    BlackbirdPosePublisher::~BlackbirdPosePublisher(){
        rclcpp::shutdown();
    }

    void BlackbirdPosePublisher::UpdatePoses(const gz::sim::EntityComponentManager &ecm){
        auto blackbird_ent = ecm.EntityByComponents(gz::sim::components::Name("blackbird"));
        auto* pose_comp = ecm.Component<gz::sim::components::Pose>(blackbird_ent);
        if (pose_comp == nullptr){
            throw std::runtime_error("blackbird_ros2 PosePublisher: pose_comp ptr is NULL");
        }
        auto pose = pose_comp->Data();
        gz::math::Vector3 position = pose.Pos();
        x = position.X();
        y = position.Y();
        z = position.Z();

        gz::math::Quaternion quat = pose.Rot();
        r = quat.Roll();
        p = quat.Pitch();
        w = quat.Yaw();


        

    }

    void BlackbirdPosePublisher::Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&,
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& event_mgr)
    {
        // TODO: Initialize velocity to zero
        gz::sim::Entity torso_ent = ecm.EntityByComponents(gz::sim::components::Name("torso"), gz::sim::components::Link());

        if (torso_ent == gz::sim::kNullEntity)
        {
            throw std::runtime_error("StateUpdater::Configure() Torso link entity not found.");
        } 

        gz::sim::Link torso_link(torso_ent);
        auto world_pose_opt = torso_link.WorldPose(ecm);
        if (!world_pose_opt)
        {
            throw std::runtime_error("StateUpdater::Configure() Failed to get WorldPose for the torso link.");
        }
        const gz::math::Pose3d &initial_pose = *world_pose_opt;

        x = initial_pose.Pos().X();
        y = initial_pose.Pos().Y();
        z = initial_pose.Pos().Z();
        r = initial_pose.Rot().Roll();
        p = initial_pose.Rot().Pitch();
        w = initial_pose.Rot().Yaw();



    }

   void BlackbirdPosePublisher::PostUpdate(const gz::sim::UpdateInfo &_info,
                                    const gz::sim::EntityComponentManager &ecm)
    {
        // TODO: update velocities before establishing new poses
        
        UpdatePoses(ecm);
        pub_->publish(msg_);
        
    }
}
GZ_ADD_PLUGIN(
    blackbird_ros2::BlackbirdPosePublisher,
    gz::sim::System,
    blackbird_ros2::BlackbirdPosePublisher::ISystemConfigure,
    blackbird_ros2::BlackbirdPosePublisher::ISystemPostUpdate
    
)