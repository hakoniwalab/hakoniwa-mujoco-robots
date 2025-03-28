#pragma once

#include "primitive_types.hpp"
#include "pid.hpp"

namespace hako::robots::controller {

class SliderController {
private:
    PID pid;

public:
    SliderController(double kp, double ki, double kd)
        : pid(kp, ki, kd) {}

    double update(const hako::robots::types::Position& current_position,
                  const double target_z,
                  const double dt)
    {
        return pid.update(target_z, current_position.z, dt);
    }

    void reset() {
        pid.reset();
    }
};

}
