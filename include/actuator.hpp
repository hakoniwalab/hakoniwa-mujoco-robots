#pragma once

namespace hako::robots::actuator
{
    class ITorqueActuator
    {
    public:
        virtual ~ITorqueActuator() {}
        virtual void SetTorque(double torque) = 0;
    };
}
