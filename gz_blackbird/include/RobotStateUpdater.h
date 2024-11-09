#include <gz/sim/Server.hh>
#include <gz/common/Console.hh>
#include <gz/sim/ServerConfig.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <memory>
#include <vector>
#include <string>

// mafs
#include <gz/math.hh>

/*
StateUpdater.h

I'm copying part of this:
https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/joint_state_publisher/JointStatePublisher.hh

but I'm not making a gz-transport node to publish the data.
*/

#ifndef STATEUPDATER_HPP
#define STATEUPDATER_HPP
class StateUpdater: public gz::sim::System, 
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPostUpdate,
                    public gz::sim::ISystemReset 
{
    public:
        StateUpdater(std::mutex& stateMutex, std::shared_ptr<double[]> state);
        ~StateUpdater();

        // inherited from ISystemConfigure ABC
        void Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&, 
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& eventMgr);

        // inherited from ISystemPostUpdate ABC
        void PostUpdate(const gz::sim::UpdateInfo& info,
                        const gz::sim::EntityComponentManager& ecm);

        // inherited from ISystemReset ABC
        void Reset(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm);

    private:
        std::mutex& state_mutex_;
        std::shared_ptr<double[]> state_; // joint states vector, as sepcified in docs
        
        // for velocity updates
        double x; 
        double y;
        double z; 

        double r; 
        double p; 
        double w; 
};

#endif