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

    double update(const double current_pos,
                  const double target_pos,
                  const double dt)
    {
        return pid.update(target_pos, current_pos, dt);
    }

    void reset() {
        pid.reset();
    }
};

}
