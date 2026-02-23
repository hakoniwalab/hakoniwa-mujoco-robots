#pragma once

#include <memory>
#include <mutex>

#include "physics.hpp"

class ForkliftSimulationLoop {
public:
    ForkliftSimulationLoop(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        std::mutex& data_mutex,
        bool& running_flag);

    int run();

private:
    std::shared_ptr<hako::robots::physics::IWorld> world_;
    std::mutex& data_mutex_;
    bool& running_flag_;
};
