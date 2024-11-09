#include "BipedalContact.h"

BipedalContact::BipedalContact(std::mutex& contactMutex, std::shared_ptr<bool[]> contacted)
: gz::sim::System(), contact_mutex_(contactMutex), contacted_(contacted)
{
    link_map = {{"torso", {}},{"l_foot", {}}, {"r_foot", {}}};
}

BipedalContact::~BipedalContact(){/* Documentation Inherited */}

void BipedalContact::Configure(const gz::sim::Entity&,
                        const std::shared_ptr<const sdf::Element>&, // doc-inherited
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager&)
{
    if (!ecm.HasComponentType(gz::sim::components::ContactSensor::typeId)){
        throw std::runtime_error("BipedalContact::Configure() no ContactSensor found...wtf");
    }


    ecm.EachNew<gz::sim::components::ContactSensor>(
        [&](const gz::sim::Entity& contact_ent_, const gz::sim::components::ContactSensor * contact_comp_)-> bool
    {

        // declare this lambda before EachNew() call for cleaner code
        auto* parent_ent = ecm.Component<gz::sim::components::ParentEntity>(contact_ent_); // this dude is the Link Entity object


        auto* link_ent = ecm.Component<gz::sim::components::Link>(parent_ent->Data());
        if (link_ent == nullptr){
            throw std::runtime_error("BipedalContact::EachNew() Parent entity of contact sensor is not a link");
        }
        auto link_name_opt = ecm.ComponentData<gz::sim::components::Name>(parent_ent->Data());
        std::string link_name = link_name_opt.value();

        // debug
        std::vector<gz::sim::Entity> collisions = ecm.ChildrenByComponents(parent_ent->Data(), gz::sim::components::Collision());
        if (collisions.empty())
        {
            throw std::runtime_error("BipedalContact::Configure() parent_ent has no collisions...wtf");
        }

        for (auto collision_ent : collisions)
        {
            auto collision_name_opt = ecm.ComponentData<gz::sim::components::Name>(collision_ent);
            std::string collision_name = collision_name_opt.value();

            auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision_ent);
            if (contact == nullptr)
            {
                std::cout << "BiepdalContact::Configure() creating contact component for: " << link_name + "\n";
                ecm.CreateComponent(collision_ent, gz::sim::components::ContactSensorData());
            }

            link_map[link_name].push_back(collision_name);
        }

        return true;
    });
}

void BipedalContact::PostUpdate(const gz::sim::UpdateInfo& info,
                        const gz::sim::EntityComponentManager& ecm){
    /*
    check if each link has made a collision, and update the sensor data in contacted_
    */
    if (!ecm.HasComponentType(gz::sim::components::ContactSensorData::typeId)){
        throw std::runtime_error("BipedalContact::PostUpdate() no ContactSensor found ...wtf");
    }

    std::lock_guard<std::mutex> lock(contact_mutex_);
    int i = 0; // i <  3 through this whole loop because CONTACT_LINKS.size() == 3
    for (std::string link_name : CONTACT_LINKS){
        bool contacted = 0;
        std::vector<std::string> collision_names = link_map[link_name];
        

        // loop 2
        for (auto collision_name : collision_names){
            auto collision_ent = ecm.EntityByComponents(gz::sim::components::Collision(), gz::sim::components::Name(collision_name));
            auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision_ent);
            if (contact == nullptr){
                throw std::runtime_error("BipedalContact::PostUpdate() contact returned nullptr for collision: " + collision_name);
            }

            else if (contact->Data().contact_size() > 0){
                contacted |= 1;
            }
            else{
                contacted |= 0;
            }
        }
        contacted_[i] = contacted;
        // std::cout << "BipedalContact state for link " + link_name +": " << contacted_[i] << std:: endl; 
        i++;
    }

}

void BipedalContact::Reset(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm)
{
    if (!ecm.HasComponentType(gz::sim::components::ContactSensor::typeId)){
        throw std::runtime_error("BipedalContact::Reset() no ContactSensor found...wtf");
    }

    ecm.EachNew<gz::sim::components::ContactSensor>(
        [&](const gz::sim::Entity& contact_ent_, const gz::sim::components::ContactSensor * contact_comp_)-> bool
    {

        // declare this lambda before EachNew() call for cleaner code
        auto* parent_ent = ecm.Component<gz::sim::components::ParentEntity>(contact_ent_); // this dude is the Link Entity object

        auto* link_ent = ecm.Component<gz::sim::components::Link>(parent_ent->Data());
        if (link_ent == nullptr){
            throw std::runtime_error("BipedalContact::Reset EachNew() Parent entity of contact sensor is not a link");
        }
        auto link_name_opt = ecm.ComponentData<gz::sim::components::Name>(parent_ent->Data());
        std::string link_name = link_name_opt.value();

        // debug
        std::vector<gz::sim::Entity> collisions = ecm.ChildrenByComponents(parent_ent->Data(), gz::sim::components::Collision());
        if (collisions.empty())
        {
            throw std::runtime_error("BipedalContact::Reset() parent_ent has no collisions...wtf");
        }

        for (auto collision_ent : collisions)
        {
            auto collision_name_opt = ecm.ComponentData<gz::sim::components::Name>(collision_ent);
            std::string collision_name = collision_name_opt.value();

            auto contact = ecm.Component<gz::sim::components::ContactSensorData>(collision_ent);
            if (contact == nullptr)
            {
                ecm.CreateComponent(collision_ent, gz::sim::components::ContactSensorData());
            }

        }

        return true;
    });

    std::lock_guard guard(contact_mutex_);
    for (int i = 0 ; i < 3; ++i){
        contacted_[i] = 0;
    }
}
