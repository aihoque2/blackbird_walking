
// local includes
#include "ECMProvider.h"
#include "EffortController.h"
#include "BipedalContact.h"
#include "RobotStateUpdater.h"

// higher lvl includes
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>

// component includes
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>

// acceleration components
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>

#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/CanonicalLink.hh>

// mafs
#include <gz/math.hh>

// data structures
#include <boost/process.hpp>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cstdio>
#include <iostream>



/*
NOTE: NO ROS2 to WORRY about here. only have to worry

about being able to pull full state out in valid manner

!!!The ROS2 nodes to step and publish will be written with rclpy!!!
*/

#ifndef TRAINSIMULATOR_H
#define TRAINSIMULATOR_H

#define WORLD_IDX 0

#define STATE_SIZE 32
#define ACTION_SIZE 10

/*Sim.h*/

class TrainSimulator{
    /*
    Hello, I am TrainSimulator
    you shall attach me to a gym
    environment wrapper via pybind11
    */

    public:

        /*
        functions
        */
        TrainSimulator(bool gui, const std::string file_path); // constructor
        ~TrainSimulator(); // desctructor

        // should this be void, or should this return state values?
        void Step(std::vector<double> inputAction); // 10-array of joint commands to send each joint and step environment with joint action
        void StepFew(std::vector<double> inputAction, int axnSteps, int afterSteps);
        void SetAction(std::vector<double> action); // mainly a helper for gym's step() to make modifying axn_ thread-safe       
        void Pause();
        void Unpause();
        void ResetSim(); // make action_cb ignore actions during the reset, then reset torque/velocities of joints and torso
        bool isTerminal(); // check if the robot is in a terminal state
        std::vector<double> GetState(); //
        void KillPIDs(std::string process_name);



        /*
        variables 
        */
        std::unique_ptr<gz::sim::Server> server_;
        std::unique_ptr<boost::process::child> gui_; // note that gui of ignition simulation runs as a separate process.

        // "Simulation Resources"
        gz::sim::EventManager* event_mgr_; // not the owner, so raw ptr OK
        gz::sim::EntityComponentManager* ecm_; // not the owner, so raw ptr OK

        // "PhsyicsData" not sure if needed
        double rtf = -1;
        double maxStepSize = -1;
        double realtimeUpdateRate= -1;
        
        bool resetting;

    private:
        // shoutouts to gz-harmonic screwing up from gz-fortress

        gz::sim::ServerConfig serverConfig;
        std::string sdfFile; // filename for the sdf to spawn
        std::string worldFile; // filename for the world

        std::shared_ptr<double[]> state_; // 1D 30-vector to represent our state
        std::shared_ptr<double[]> axn_; // 1D 10-vector to represent torques we send on each joint
        std::shared_ptr<bool[]> contacted_;// 1D 10 vector to show our individual contact states
        
        std::mutex axnMutex; // lock used for set_action() and get_action()
        std::mutex stateMutex; // lock used for publishing state info
        std::mutex contactMutex; // lock used for publishing contact sensor data
 
        // we need to keep this plugin
        std::shared_ptr<ECMProvider> provider_;
        bool hasGUI;
};

#endif