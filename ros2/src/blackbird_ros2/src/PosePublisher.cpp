#include "PosePublisher.h"

namespace blackbird_ros2{

    BlackbirdPosePublisher::BlackbirdPosePublisher(){
        node_ = rclcpp::Node::make_shared("pose_publisher");
        pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("torso_pose", 10);
        x = 0.0;
        y = 0.0;
        z = 0.0;
        q1 = 0.0;
        q2 = 0.0;
        q3 = 0.0;
        w = 0.0;
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
        q1 = quat.X();
        q2 = quat.Y();
        q3 = quat.Z();
        w = quat.W(); 

        // position
        msg_.position.x = x;
        msg_.position.y = y;
        msg_.position.z = z;

        msg_.orientation.x = q1;
        msg_.orientation.y = q2;
        msg_.orientation.z = q3;
        msg_.orientation.w = w;
    }

    void BlackbirdPosePublisher::Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&,
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& event_mgr)
    {
        UpdatePoses(ecm);
        // TODO: Initialize velocity to zero
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