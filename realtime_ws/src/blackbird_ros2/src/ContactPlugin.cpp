#include "ContactPlugin.h"

BlackbirdContactPlugin::BlackbirdContactPlugin(){}

BlackbirdContactPlugin::~BlackbirdContactPlugin(){}

BlackbirdContactPlugin::Configure()(const gz::sim::Entity& entity,
                       const std::shared_ptr<const sdf::Element>&,
                       gz::sim::EntityComponentManager& ecm,
                       gz::sim::EventManager& event_mgr)
{
    for (auto link_name: LINK_NAMES){
        auto* link_ent
    }
}