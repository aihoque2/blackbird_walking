#include <gz/sim/Server.hh>
#include <gz/common/Console.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/rendering.hh>
#include <gz/gui.hh>
#include <iostream>
#include "Simulator.h"

#include <gz/sim/Entity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <string>
#include <random>

int main(int argc, char **argv)
{
    std::string file_path = "world/empty.world";
    TrainSimulator my_sim(true, file_path);
    bool hasBlackbird = my_sim.server_->HasEntity("blackbird", 0);
    std::cout << "hasBlackbird: " << hasBlackbird << "\n";
    auto paused = my_sim.server_->Paused(); // need to auto bc it returns std::optional
    std::cout << "server paused: " << *paused << std::endl;


    // run the server
    std::vector<double> axn = {0.0, 0.0, 0.0, 0.0, 0.0, /* Left Js */   
                                        0.0, 0.0, 0.0, 0.0, 0.0 /* Right Js */};


    double lower_bound = -10.0;
    double upper_bound = 10.0;
 
    std::uniform_real_distribution<double> unif(lower_bound,
                                           upper_bound);
 
    std::default_random_engine re;

    bool resetting = false;
    while (true){
    
    for (int i = 0; i < 10; i++){
        // Getting a random double value
       double random_double = unif(re);
        axn[i] = random_double;
    }

    my_sim.Step(axn);
    
    if (my_sim.isTerminal()){
        std::cout << "reached terminal case" << std::endl;
        my_sim.ResetSim();
    }
    

    }

    return 0;
}