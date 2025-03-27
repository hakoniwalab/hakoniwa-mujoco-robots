#pragma once

#include "actuator.hpp"
#include <mujoco/mujoco.h>
#include "primitive_types.hpp"
#include <memory>
#include <string>

namespace hako::robots::actuator::impl
{
    class TorqueActuatorImpl : public ITorqueActuator
    {
    private:
        mjModel* model;
        mjData* data;
        int actuator_id;
    public:
        TorqueActuatorImpl(mjModel* m, mjData* d, const std::string& actuator_name): model(m), data(d)
        {
            actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
            if (actuator_id < 0) {
                throw std::runtime_error("Actuator not found: " + actuator_name);
            }
        }
        void SetTorque(double torque) override
        {
            data->ctrl[actuator_id] = torque;
        }
    };
}
