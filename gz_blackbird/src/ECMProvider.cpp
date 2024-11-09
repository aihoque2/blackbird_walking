#include "ECMProvider.h"

#include <iostream> // I use cout because I'm a peasant


/*
Constructor
*/
ECMProvider::ECMProvider(): gz::sim::System(){}

ECMProvider::~ECMProvider()
{
    ecm_ = nullptr;
    evtmgr_ = nullptr;
}


/*
function that is run when the server does 
AddSystem() on this system
*/
void ECMProvider::Configure(const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr)
{

    if (!ecm.EntityHasComponentType(entity, 
    gz::sim::components::World::typeId)){
        std::cerr << "ECMProvider system was not inserted in a world entity" << std::endl;
        return;
    }

    // get our world name optional
    auto world_opt = ecm.ComponentData<gz::sim::components::Name>(entity);
    if (world_opt.has_value()){ // world_optional has string
        this->world_name_ = *std::move(world_opt);
    }
    else{
        std::cerr << "ECMProvider could not give the world name, even though we passed world entity" << std::endl;
        return;
    }
    

    this->ecm_ = &ecm; // point to the addy
    this->evtmgr_ = &event_mgr; // point to the addy
    std::cout << "SUCCESS: obtained EntityComponentManager & EventManager from World: " << this->world_name_ << std::endl;
}

void ECMProvider::PreUpdate(const gz::sim::UpdateInfo& info,
                        gz::sim::EntityComponentManager& ecm)
{
    // update the existing shared ptr
    ecm_ = &ecm;
}

void ECMProvider::Reset(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm)
{
    ecm_ = &ecm;
}


gz::sim::EntityComponentManager* ECMProvider::getECM()
{
    return ecm_;
}

gz::sim::EventManager* ECMProvider::getEvtMgr()
{
    return evtmgr_;
}
