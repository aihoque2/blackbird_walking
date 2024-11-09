/*
TODO: Uneccessary bc blackbird is already declared in `empty.world`

allow us to spawn the blackbird without 
explicitly declaring it in the world file?

useful for inserting some arbitrary robot
*/

#include <memory>

#ifndef WORLD_H
#define WORLD_H

class World{
    public:
        bool insertModel()

    private:
        std::shared_ptr<gz::sim::SdfEntityCreator> sdfEntityCreator;
};

#endif