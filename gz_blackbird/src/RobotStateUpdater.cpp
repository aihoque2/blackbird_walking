#include "RobotStateUpdater.h"
#include "EffortController.h" // for the JOINT_NAMES list
/*
implement that header stuff here
*/

StateUpdater::StateUpdater(std::mutex& stateMutex, std::shared_ptr<double[]> state)
:gz::sim::System(), state_mutex_(stateMutex), state_(state), x(0.0), y(0.0), z(0.0), r(0.0), p(0.0), w(0.0)
{
}

StateUpdater::~StateUpdater(){

}

void StateUpdater::Configure(const gz::sim::Entity& entity,
                        const std::shared_ptr<const sdf::Element>&,
                        gz::sim::EntityComponentManager& ecm,
                        gz::sim::EventManager& eventMgr)
{   
    // read the github code and see what's
    // needed to be done here.

    gz::sim::Entity blackbird_ent = ecm.EntityByComponents(gz::sim::components::Name("blackbird"));
    auto* pose_comp = ecm.Component<gz::sim::components::Pose>(blackbird_ent);
    if (pose_comp == nullptr){
        throw std::runtime_error("StateUpdater::Configure() Pose component returned nullptr");
    }

    for (auto joint_name: JOINT_NAMES){
        gz::sim::Entity joint_ent = ecm.EntityByComponents(gz::sim::components::Joint(), gz::sim::components::Name(joint_name));
        auto* j_pos = ecm.Component<gz::sim::components::JointPosition>(joint_ent);
        auto* j_vel = ecm.Component<gz::sim::components::JointVelocity>(joint_ent);

        if (j_pos == nullptr){
            ecm.CreateComponent(joint_ent, gz::sim::components::JointPosition({0.0}));
        }
        if (j_vel == nullptr){
            ecm.CreateComponent(joint_ent, gz::sim::components::JointVelocity({0.0}));
        }
    }

}

void StateUpdater::PostUpdate(const gz::sim::UpdateInfo &info,
                                const gz::sim::EntityComponentManager &ecm)
{
    gz::sim::Entity blackbird_ent = ecm.EntityByComponents(gz::sim::components::Name("blackbird"));
    int i = 0;

    double dub_dt = std::chrono::duration<double>(info.dt).count(); // get the time in seconds

    if (dub_dt > 0.0){

        // first 6 dimensions of state_
        auto* pose_comp = ecm.Component<gz::sim::components::Pose>(blackbird_ent);
        auto pose = pose_comp->Data();
        state_[0] = pose.Pos().X();
        state_[6] = (state_[0] - x)/dub_dt; // velx
        x = state_[0]; // update to new state

        state_[1] = pose.Pos().Y();
        state_[7] = (state_[1] - y)/dub_dt; // vel_y
        y = state_[1];

        state_[2] = pose.Pos().Z(); 
        double dz = state_[2] - z;
        state_[8] = (dz)/dub_dt; // vel_z
        z = state_[2];
        
        // angular coords
        state_[3] = pose.Rot().Roll();
        state_[9] = (state_[3] - r)/dub_dt;
        r = state_[3];

        state_[4] = pose.Rot().Pitch();
        state_[10] = (state_[4] - p)/dub_dt;
        p = state_[4];

        state_[5] = pose.Rot().Yaw();
        state_[11] = (state_[5] - w)/dub_dt;
        w = state_[5];
    }
    /*
    linear an angular velocities 
    cover indicies 6-11.
    */


    // Joint position states and vels
    i = 12;
    for (auto joint_name: JOINT_NAMES){
        gz::sim::Entity joint_ent = ecm.EntityByComponents(gz::sim::components::Joint(), gz::sim::components::Name(joint_name));
        auto* j_pos = ecm.Component<gz::sim::components::JointPosition>(joint_ent);
        if (j_pos == nullptr){
            throw std::runtime_error("Joint " + joint_name + " has no component JointPosition");
        }
        auto* j_vel = ecm.Component<gz::sim::components::JointVelocity>(joint_ent);
        if (j_vel == nullptr){
            throw std::runtime_error("Joint " + joint_name + " has no component JointVelocity");
        }

        // set joint position
        state_[i] = j_pos->Data()[0];

        // set joint velocity
        state_[i+10] = j_vel->Data()[0];
        i++; // i == 22 at the end of this loop, so we never access beyond idx 31
    }
}

void StateUpdater::Reset(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm)
{
    gz::sim::Entity blackbird_ent = ecm.EntityByComponents(gz::sim::components::Name("blackbird"));
    auto* pose_comp = ecm.Component<gz::sim::components::Pose>(blackbird_ent);
    if (pose_comp == nullptr){
        throw std::runtime_error("StateUpdater::Reset() Pose component returned nullptr");
    }

    gz::math::Vector3d zero_vel(0, 0, 0);

    // linear velocity check
    if (!ecm.EntityHasComponentType(blackbird_ent, gz::sim::components::LinearVelocity().TypeId())){
        ecm.CreateComponent(blackbird_ent, gz::sim::components::LinearVelocity(zero_vel));
    }

    // angular velocity check
    if (!ecm.EntityHasComponentType(blackbird_ent, gz::sim::components::AngularVelocity().TypeId())){
        ecm.CreateComponent(blackbird_ent, gz::sim::components::AngularVelocity(zero_vel));
    }

    for (auto joint_name: JOINT_NAMES){
        gz::sim::Entity joint_ent = ecm.EntityByComponents(gz::sim::components::Joint(), gz::sim::components::Name(joint_name));
        auto* j_pos = ecm.Component<gz::sim::components::JointPosition>(joint_ent);
        auto* j_vel = ecm.Component<gz::sim::components::JointVelocity>(joint_ent);

        if (j_pos == nullptr){
            ecm.CreateComponent(joint_ent, gz::sim::components::JointPosition({0.0}));
        }
        if (j_vel == nullptr){
            ecm.CreateComponent(joint_ent, gz::sim::components::JointVelocity({0.0}));
        }
    }
}