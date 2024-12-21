#include "ContactPlugin.h"

BlackbirdContactPlugin::BlackbirdContactPlugin(): gz::sim::System()
{
    link_map = {{"torso", {}},{"l_foot", {}}, {"r_foot", {}}};
}

BlackbirdContactPlugin::~BlackbirdContactPlugin(){
    rclcpp::shutdown();
}

BlackbirdContactPlugin::Configure()(const gz::sim::Entity& entity,
                       const std::shared_ptr<const sdf::Element>&,
                       gz::sim::EntityComponentManager& ecm,
                       gz::sim::EventManager& event_mgr)
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
            auto collision_name_opt = ecm.ComponentData<gz::sim::components::Name>(collision_ent);
            std::string collision_name = collision_name_opt.value()

            auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
            if (contact == nullptr)
            {
                std::cout << "ContactPlugin::Configure() creating contact component for: " << link_name + "\n";
                ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
            }

            link_map[link_name].push_back(collision_name); // ez debugging this method
        }
    }
}

BlackbirdContactPlugin::PostUpdate(const gz::sim::UpdateInfo& info,
                        const gz::sim::EntityComponentManager& ecm)
{
    if (!ecm.HasComponentType(gz::sim::components::ContactSensorData::typeId))
    {
        throw std::runtime_error("BlackBirdContactPlugin::PostUpdate() no ContactSensor found ...wtf");
    }
    std::vector<float> contact_data;

    for (auto link_name : LINK_NAMES){
        double contacted = 0.0;
        gz::sim::Entity link_ent = ecm.EntityByComponents(gz::sim::components::Name(link_name), gz::sim::components::Link());
        std::vector<gz::sim::Entity> collision_ents = ecm.ChildrenByComponents(link_ent, gz::sim::components::)
        std::vector<std::string> collision_names = link_map[link_name];

        for (auto collision: collision_ents){
            auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision);
            if (contact == nullptr){
                throw std::runtime_error("BlackbirdContactPlugin::PostUpdate() contact not found for collision");
            }

            if (contact->Data().contact_size() > 0){
                contacted |= 1.0;
            }
            else{
                contacted |= 0.0;
            }
        }
        contact_data.push_back(contacted);

    }
    msg_.data = std::move(contact_data);
    pub_.publish(contact_data);
    
}