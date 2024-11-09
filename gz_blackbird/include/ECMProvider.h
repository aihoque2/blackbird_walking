#include <gz/sim/Server.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/common/Console.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <memory>



/*ECMProvider.h

*/
#ifndef ECMPROVIDER_HPP
#define ECMPROVIDER_HPP

/*

*/
class ECMProvider: public gz::sim::System, 
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemReset
{
    public:
        ECMProvider(); // create the ignition gazebo system
        ~ECMProvider();

        void Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&, 
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& eventMgr);

        void PreUpdate(const gz::sim::UpdateInfo& info,
                        gz::sim::EntityComponentManager& ecm);
        
        void Reset(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

        gz::sim::EntityComponentManager* getECM();
        gz::sim::EventManager* getEvtMgr();
        

        
    
    private:
        gz::sim::EntityComponentManager* ecm_; // not the owner
        gz::sim::EventManager* evtmgr_; // not the owner
        std::string world_name_;

};


#endif