#pragma once

#include <string>

namespace hako::robots::actuator
{
    class ITorqueActuator
    {
    public:
        virtual ~ITorqueActuator() {}
        virtual void SetTorque(double torque) = 0;
    };

    enum class ActuatorType {
        Position,
        Velocity,
        Torque
    };

    struct JointActuatorConfig {
        std::string joint_name;
        ActuatorType type;
        struct {
            double lower {0.0};
            double upper {0.0};
            double effort {0.0};
            double velocity {0.0};
            bool has_limits {false};
        } limit;
        struct {
            double damping {0.0};
            double friction {0.0};
        } dynamics;
        std::string actuator_name;
    };

    struct JointActuatorTarget {
        double value {0.0};
    };

    class IJointActuator
    {
    public:
        virtual ~IJointActuator() {}
        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const JointActuatorConfig& GetConfig() const = 0;
        virtual void SetTarget(double target) = 0;
        virtual void SetTarget(const JointActuatorTarget& target)
        {
            SetTarget(target.value);
        }
    };
}
