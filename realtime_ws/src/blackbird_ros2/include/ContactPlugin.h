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
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/System.hh>

#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


#ifndef BLACKBIRDROS2_CONTACTPLUGIN_HH_
#define BLACKBIRDROS2_CONTACTPLUGIN_HH_

auto LINK_NAMES = {"torso", "l_foot", "r_foot"}

namespace blackbird_ros2{
class BlackbirdContactPlugin : public gz::sim::System,
                                public gz::sim::ISystemConfigure, 
                                public gz::sim::ISystemPostUpdate
{
    public:
        BlackbirdContactPlugin();
        ~BlackbirdContactPlugin();

        void Configure(const gz::sim::Entity& entity,
                       const std::shared_ptr<const sdf::Element>&,
                       gz::sim::EntityComponentManager& ecm,
                       gz::sim::EventManager& event_mgr);
        
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                                const gz::sim::EntityComponentManager &ecm);

    private:
        bool contacts_[3]; // torso, footL, footR
        std::unordered_map<std::string, std::vector<std::string>> link_map; // map link names to their collision names


};
}
#endif