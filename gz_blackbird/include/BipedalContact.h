/*
BipedalContact.h

ContactSensor system to manage the blackbird's contacts

TODO: REGISTER THIS PLUGIN TO BE ABLE TO USE IN SDF FILES
*/

#include <gz/sim/Server.hh>
#include <gz/common/Console.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <unordered_map>
#include <memory>
#include <vector>
#include <string>

static const std::vector<std::string> CONTACT_LINKS = 
{"torso", "l_foot", "r_foot"};

#ifndef BIPEDALCONTACT_HPP
#define BIPEDALCONTACT_HPP

class BipedalContact : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate,
                        public gz::sim::ISystemReset
{
    public:
        BipedalContact(std::mutex& contactMutex, std::shared_ptr<bool[]> contacted);
        ~BipedalContact();
        

        // inherited from ISystemConfigure ABC
        void Configure(const gz::sim::Entity& _entity,
                        const std::shared_ptr<const sdf::Element>& _sdf, //doc-inherited
                        gz::sim::EntityComponentManager& _ecm,
                        gz::sim::EventManager& _eventMgr);

        // inherited from ISystemPreUpdate ABC
        void PostUpdate(const gz::sim::UpdateInfo& info,
                        const gz::sim::EntityComponentManager& ecm);
                
        void Reset(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm);

    private:
        std::shared_ptr<bool[]> contacted_; // {torso, footL, footR} in that order {0, 1, 2}
        std::mutex& contact_mutex_; // thread-safe
        std::unordered_map<std::string, std::vector<std::string>> link_map; // map link names to their collision names
};

#endif
