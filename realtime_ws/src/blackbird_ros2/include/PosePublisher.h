
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/System.hh>

#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#ifndef BLACKBIRDROS2_POSEPUBLISHER_HH_
#define BLACKBIRDROS2_POSEPUBLISHER_HH_

namespace blackbird_ros2{
class BlackbirdPosePublisher : public gz::sim::System,
                                public gz::sim::ISystemConfigure, 
                                public gz::sim::ISystemPostUpdate
{
    public:
        BlackbirdPosePublisher();
        ~BlackbirdPosePublisher();

        void Configure(const gz::sim::Entity& entity,
                       const std::shared_ptr<const sdf::Element>&,
                       gz::sim::EntityComponentManager& ecm,
                       gz::sim::EventManager& event_mgr);
        
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &ecm);
        
        void UpdatePoses(const gz::sim::EntityComponentManager &ecm);
        vector<bool> GetContacts(const gz::sim::EntityComponentManager &ecm);
        
        
        // position
        double x;
        double y;
        double z;

        // orientation
        double r; // roll
        double p; // pitch
        double w; // yaw but y is already in use


    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        std_msgs::msg::Float64MultiArray msg_;
};
}
#endif