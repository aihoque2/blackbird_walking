#include "EffortController.h"

namespace components = gz::sim::components;

EffortController::EffortController(std::mutex& axnMutex, std::shared_ptr<double[]> axn) 
: gz::sim::System(), axn_mutex_(axnMutex), axn_(axn), forceCompCreation(0)
{
    for (auto name: JOINT_NAMES){
        std::cout << "here's name of a joint: " << name << '\n';
    }
}

EffortController::~EffortController()
{
    std::cout << "num times ForceCompCreation was called outside of Configure(): " << forceCompCreation << std::endl;
}

void EffortController::Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&, //doc-inherited
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& eventMgr)
{
    /*
    Verify that the robot controller will match the numberr of joints.
    */
    auto joints = ecm.EntitiesByComponents(gz::sim::components::Joint());
    // std::cout << "size of joints array: " << joints.size() << std::endl;
    // verify joint array
    if (joints.size() != 10){
        throw std::runtime_error("EffortController::Configure(): joints array is not of size 10. It is instead of size: " + joints.size());
    }

    /*
    verify each joint name
    scope brackets to keep this set's 
    memory temporary for O(1) lookup
    */
    {
        std::unordered_set<std::string> name_set(JOINT_NAMES.begin(), JOINT_NAMES.end());
        for (auto joint_name : JOINT_NAMES){
        gz::sim::Entity joint = ecm.EntityByComponents(gz::sim::components::Name(joint_name), gz::sim::components::Joint());
            if (joint == gz::sim::kNullEntity){
                throw std::runtime_error("EffortController::Configure(): Unkown joint name found: " + joint_name);
            }
            auto force = ecm.Component<gz::sim::components::JointForceCmd>(joint);
            if (force == nullptr){
                std::cout << "EffortController::Configure(): entity: " + joint_name + " has NULL force Component ptr. Adding force component..." << std::endl;
                ecm.CreateComponent(joint, gz::sim::components::JointForceCmd({0.0}));
            }
        }
    }

    std::cout << "EffortController::Configure() Joint Controller Verification completed" << std::endl << std::flush;
}

void EffortController::PreUpdate(const gz::sim::UpdateInfo& info,
                        gz::sim::EntityComponentManager& ecm)
{

    /*
    Torque msg updates
    We know our joint names.
    We can just look them up.
    */
    
    std::lock_guard<std::mutex> lock(axn_mutex_); // lock our axn array
    for (int i = 0; i < JOINT_NAMES.size(); i++){
        std::string jnt_name = JOINT_NAMES[i];
        gz::sim::Entity joint = ecm.EntityByComponents(gz::sim::components::Name(jnt_name), gz::sim::components::Joint());


        if (!ecm.HasEntity(joint)){
            throw std::runtime_error("EffortController::PreUpdate(): ECM doesn't have joint: " + jnt_name);
        }

    
        // send force command
        auto force = ecm.Component<gz::sim::components::JointForceCmd>(joint);
        if (force == nullptr){
            std::cout << "EffortController::PreUpdate(): entity: " + jnt_name + " has NULL force Component ptr. Adding force component..." << std::endl << std::flush;
            ecm.CreateComponent(joint, gz::sim::components::JointForceCmd({axn_[i]}));
            forceCompCreation++;
        }

        else{ 
            force->Data()[0] = axn_[i];
        }
        axn_[i] = 0.0;
    }
}

void EffortController::Reset(const gz::sim::UpdateInfo& info,
                        gz::sim::EntityComponentManager& ecm)
{
        for (auto joint_name : JOINT_NAMES){
            auto joint = ecm.EntityByComponents(gz::sim::components::Joint(), 
                                                gz::sim::components::Name(joint_name));
            /*reset each joint state */ 

            // theta reset (position)
            auto * joint_reset = ecm.Component<gz::sim::components::JointPositionReset>(joint);
            
            if (joint_reset == nullptr)
                ecm.CreateComponent(joint, gz::sim::components::JointPositionReset({0.0}));
            else 
                ecm.SetComponentData<gz::sim::components::JointPositionReset>(joint, {0.0});
            

            // theta-dot
            auto* joint_vel = ecm.Component<gz::sim::components::JointVelocityReset>(joint);
            
            if (joint_vel == nullptr){
                ecm.CreateComponent(joint, gz::sim::components::JointVelocityReset({0.0}));
            }
            else{
                // set the components pose
                ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint, {0.0});
            }

            // theta-double-dot???
            auto* force = ecm.Component<gz::sim::components::JointForceCmd>(joint);
            if (force == nullptr){
                ecm.CreateComponent(joint, gz::sim::components::JointForceCmd({0.0}));
            }
            else{
                force->Data()[0] = 0.0;
            }
            // announcing changed states
            ecm.SetChanged(joint, gz::sim::components::JointPositionReset::typeId, gz::sim::ComponentState::OneTimeChange);
            ecm.SetChanged(joint, gz::sim::components::JointVelocityReset::typeId, gz::sim::ComponentState::OneTimeChange);
            ecm.SetChanged(joint, gz::sim::components::JointForceCmd::typeId, gz::sim::ComponentState::OneTimeChange);
                        std::cout << "EffortController::Reset() passed SetChanged() for: " << joint_name << std::endl;

    }
}

