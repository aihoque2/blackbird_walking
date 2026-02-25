#include "ContactPlugin.h"

namespace blackbird_ros2{
    BlackbirdContactPlugin::BlackbirdContactPlugin(): gz::sim::System()
    {
        link_map = {{"torso", {}},{"l_foot", {}}, {"r_foot", {}}};
        node_ = rclcpp::Node::make_shared("contact_plugin_node");
        pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("contacts", 10);
        msg_.data.resize(3);
    }

    BlackbirdContactPlugin::~BlackbirdContactPlugin(){
        rclcpp::shutdown();
    }

    void BlackbirdContactPlugin::Configure(const gz::sim::Entity& ,
                        const std::shared_ptr<const sdf::Element>&,
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager&)
    {
        for (auto link_name: LINK_NAMES){
            auto link_ent = ecm.EntityByComponents(gz::sim::components::Name(link_name), gz::sim::components::Link());
            std::vector<gz::sim::Entity> collisions = ecm.ChildrenByComponents(link_ent, gz::sim::components::Collision());
            
            if (collisions.empty())
            {
                throw std::runtime_error("BlackbirdContactPlugin::Configure() parent_ent has no collisions...wtf");
            }

            for (auto collision: collisions)
            {
                auto collision_name_opt = ecm.ComponentData<gz::sim::components::Name>(collision);
                std::string collision_name = collision_name_opt.value();

                auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
                if (contact == nullptr)
                {
                    std::cout << "ContactPlugin::Configure() creating contact component for: " << link_name  << std::endl;
                    ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
                }

                link_map[link_name].push_back(collision_name); // ez debugging this method
            }
        }
    }

    void BlackbirdContactPlugin::PostUpdate(const gz::sim::UpdateInfo&,
                            const gz::sim::EntityComponentManager& ecm)
    {
        if (!ecm.HasComponentType(gz::sim::components::ContactSensorData::typeId))
        {
            throw std::runtime_error("BlackBirdContactPlugin::PostUpdate() no ContactSensor found ...wtf");
        }
        std::vector<double> contact_data;

        for (auto& link_name : LINK_NAMES){
            bool contacted = 0;
            gz::sim::Entity link_ent = ecm.EntityByComponents(gz::sim::components::Name(link_name), gz::sim::components::Link());
            std::vector<gz::sim::Entity> collision_ents = ecm.ChildrenByComponents(link_ent, gz::sim::components::Collision());
            std::vector<std::string>& collision_names = link_map[link_name];

            for (auto& collision: collision_ents){
                auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
                if (contact == nullptr){
                    throw std::runtime_error("BlackbirdContactPlugin::PostUpdate() contact not found for collision");
                }

                if (contact->Data().contact_size() > 0){
                    contacted |= 1;
                }
                else{
                    contacted |= 0;
                }
            }
            // std::cout << "here's contacted for link " << link_name << ": " << contacted << std::endl;
            contact_data.push_back((double)contacted);

        }
        msg_.data = std::move(contact_data);
        pub_->publish(msg_);

    }
}
GZ_ADD_PLUGIN(
    blackbird_ros2::BlackbirdContactPlugin,
    gz::sim::System,
    blackbird_ros2::BlackbirdContactPlugin::ISystemConfigure,
    blackbird_ros2::BlackbirdContactPlugin::ISystemPostUpdate
    
)