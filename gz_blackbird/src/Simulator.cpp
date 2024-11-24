#include "Simulator.h"

TrainSimulator::TrainSimulator(bool gui, const std::string file_path){
    
    resetting = false;
    axn_ = std::shared_ptr<double[]>(new double[ACTION_SIZE], std::default_delete<double[]>());
    state_ = std::shared_ptr<double[]>(new double[STATE_SIZE], std::default_delete<double[]>());
    contacted_ = std::shared_ptr<bool[]>(new bool[3], std::default_delete<bool[]>());
    
    for (int i = 0; i < ACTION_SIZE; i++){
        axn_[i] = 0.0;
    }
    
    for (int i = 0; i < STATE_SIZE; i++){
        state_[i] = 0.0; // state is defined by the LaTeX document
    }

    worldFile = file_path;
    gz::common::Console::SetVerbosity(4);
    serverConfig.SetSdfFile(worldFile); //member var
    
    server_ = std::make_unique<gz::sim::Server>(serverConfig);

    // adding ECMProvider idx
    provider_ = std::make_shared<ECMProvider>();
    const auto gotECM = server_->AddSystem(provider_, WORLD_IDX);
    if (!gotECM){
        throw std::runtime_error("could not integrate ECMProvider into server");
    }

    // EffortController plugin
    auto controller = std::make_shared<EffortController>(axnMutex, axn_);
    const auto gotCtrl = server_->AddSystem(controller, WORLD_IDX);
    if (!gotCtrl){
        throw std::runtime_error("could not integrate EffortController into server");
    }
    
    // Contact Sensor plugin
    auto contact_sensor = std::make_shared<BipedalContact>(contactMutex, contacted_);
    const auto gotContact = server_->AddSystem(contact_sensor, WORLD_IDX);
    if (!gotContact){
        throw std::runtime_error("could not integrate BipedalContact into server");
    }

    auto state_updater = std::make_shared<StateUpdater>(stateMutex, state_);
    const auto gotState = server_->AddSystem(state_updater, WORLD_IDX);
    if (!gotState){
        throw std::runtime_error("could not integrate State Updater into server");
    }

    /*
    gui code
    */
    hasGUI = gui; // need this so destructor doesn't segfault in headless
    if (gui){
        // Spawn a new process with the GUI
        gui_ = std::make_unique<boost::process::child>(
            "gz sim world/empty.world -v 0");

        bool guiServiceExists = false;
        std::unique_ptr<gz::transport::Node> node =  std::make_unique<gz::transport::Node>();
        std::vector<std::string> serviceList;

        do {
            std::cout << "Waiting for GUI to show up... " << std::endl;
            node->ServiceList(serviceList);

            for (const auto& serviceName : serviceList) {
                if (serviceName.find("/gui/") == 0) {
                    guiServiceExists = true;
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } while (!guiServiceExists);

        // end gui bracket
    } // endif

    ecm_ = provider_->getECM();
    event_mgr_ = provider_->getEvtMgr();

}


/*Destructor*/
TrainSimulator::~TrainSimulator(){
    ecm_ = nullptr;
    event_mgr_ = nullptr;
    std::cout << "Stopping TrainSimulator server..." << std::endl;
    server_->Stop();
    if (hasGUI && gui_){

        if(gui_->running()){
            std::cerr << "TRAINSIMULATOR GUI FAILED TO EXIT WITHIN A TIMELY MANNER" << std::endl << std::flush;
            std::cerr << "Forcing shutdown of gui..."<< std::endl << std::flush;
            gui_->terminate();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            gui_->wait();
            std::cerr << "Finished waiting on subprocess"<< std::endl << std::flush;
            KillPIDs("gz\\ sim\\");

        }
    }
}

/* KillPIDs() 
* kill gz_sim processes at the end
* of the TrainSimulator's lifespan
*/
void TrainSimulator::KillPIDs(std::string process_name){
    std::array<char, 128> buffer;
    std::vector<std::string> pids;
    std::string command = "ps aux | grep " + process_name + " ";

    const char* cmd = command.c_str();
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe){
        throw std::runtime_error("[ERROR] GetProcessIDs() in TrinSimulator.cpp: popen() failed!");
    }

    // red the output and get our data
    while(fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr){
        const std::string result = buffer.data();

        std::istringstream iss(result);
        std::string token;
        int token_count = 0;
        while (iss >> token){
            if (token_count == 1){ // idx 1 gives the PID
                int pid = std::stoi(token);
                kill(pid, SIGTERM);
                break; // next loop iter
            }
            token_count++;
        }

        pids.push_back(result);

    }

} 

/* Step() 
* Given our inputAction
* step 1 ms in the simulation
*/
void TrainSimulator::Step(std::vector<double> inputAction){
    if (inputAction.size() != 10){
        throw std::runtime_error("invalid inpput action size: " + std::to_string(inputAction.size()) + ". Expected size 10");
    }

    SetAction(inputAction);

    // run our action
    bool stepped = server_->RunOnce(false); // false for running the simulation steps unpaused.
}    


/* stepFew()
* given our inputAction and numSteps
* step (numSteps) milliseconds in the 
* simulation with the given inputAction 
* put for the first frame of these steps.
*
* I made this function because this
* simulation
* is for force control.
*/
void TrainSimulator::StepFew(std::vector<double> inputAction, int axnSteps, int afterSteps){
    if (inputAction.size() != 10){
        throw std::runtime_error("StepFew() invalid inpput action size: " + std::to_string(inputAction.size()) + ". Expected size 10");
    }
    
    bool stepped = false;
    // run our action steps
    for (int i = 0; i < axnSteps; i++){    
        // set our action and lock our mutex within scope
        SetAction(inputAction);
        stepped = server_->RunOnce(false);
        if (!stepped) {std::cout << "robot's action steppes returned false at iteration: "<< i << std::endl;}
    }

    // run our normal steps
    stepped = server_->Run(true, afterSteps, false); // true for running the non-blocking.
    if (!stepped){ std::cout << "robot did not step afterSteps" << std::endl;}
}    


/* pause()

* pause the simulation
*/
void TrainSimulator::Pause(){
    auto running = this->server_->Running();
    if (!running){
        std::cout << "TrainSimulator already paused. ignoring pause() call ..." << std::endl;
        return;
    }

    // pause server
    bool hasPaused = this->server_->SetPaused(true, WORLD_IDX);
    if (!hasPaused){
        throw std::runtime_error("SetPaused() returned false...world does not exist?");
    }
}

/* unpause()
* unpause the simulation
*/

void TrainSimulator::Unpause(){
    auto running = this->server_->Running();
    if (running){
        return;
    }

    // upause server
    bool unPaused = this->server_->SetPaused(false, WORLD_IDX);
    if (!unPaused){
        throw std::runtime_error("SetPaused() returned false...world does not exist?");
    }

}

void TrainSimulator::ResetSim(){
    resetting = true;
    gz::transport::Node node;

    // Create a message to reset the world
    gz::msgs::WorldControl req;
    req.mutable_reset()->set_all(true); // Reset everything (pose, velocities, etc.)

    // Publish the reset message to the control service
    
    gz::msgs::Boolean rep;
    std::string service = "/world/empty/control";
    bool result = false;
    unsigned int timeout = 5000; // Timeout in milliseconds

    // Request reset through the transport node
    bool success = node.Request(service, req, timeout, rep, result);

    if (!success || !result || !rep.data()) {
        std::cerr << "Failed to reset the world using transport service." << std::endl;
    } else {   
        resetting = false;

    }
}

/*
check if our robot is in a terminal state
*/
bool TrainSimulator::isTerminal(){
    if (contacted_[0] == 1){
        return true;
    }

    // TODO: see more terminal states
    // if robot moves out-of-bounds?
    return false;
}

void TrainSimulator::SetAction(std::vector<double> action){
    std::lock_guard<std::mutex> guard(axnMutex);        
    for (int k = 0; k < 10; ++k){
        axn_[k] = action[k];
    }

}

std::vector<double> TrainSimulator::GetState(){
    std::vector<double> ret;
    {
        std::lock_guard<std::mutex> state_guard(stateMutex);
        
        // Copy state_ data to the combined buffer
        // std::copy(state_.get(), state_.get() + STATE_SIZE, ret.begin());
        for (int i = 0; i < STATE_SIZE; i++){
            ret.push_back(state_[i]);
        }
        
    }

    {
        std::lock_guard<std::mutex> contact_guard(contactMutex);
        for (int i = 0; i < 3; i++){
            ret.push_back(contacted_[i]);
        }
    }
    return ret;
}