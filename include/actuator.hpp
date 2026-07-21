#pragma once

#include <string>
#include <vector>

namespace hako::robots::actuator
{
    class IActuator
    {
    public:
        virtual ~IActuator() {}
        virtual bool ShouldUpdate(double delta_sec)
        {
            (void)delta_sec;
            return true;
        }
    };

    class ITorqueActuator : public IActuator
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
        struct {
            std::string pdu_name;
            double update_rate_hz {0.0};
            std::string message_type;
        } pdu_config;
    };

    struct JointActuatorTarget {
        double value {0.0};
    };

    class IJointActuator : public IActuator
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

    struct JointTrajectoryJointConfig {
        std::string name;
        ActuatorType type {ActuatorType::Position};
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

    struct JointTrajectoryActuatorConfig {
        std::vector<JointTrajectoryJointConfig> joints;
        struct {
            std::string pdu_name;
            double update_rate_hz {0.0};
            std::string message_type;
        } pdu_config;
    };

    struct JointTrajectoryPointTarget {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> effort;
        double time_from_start_sec {0.0};
    };

    struct JointTrajectoryTarget {
        std::vector<std::string> joint_names;
        std::vector<JointTrajectoryPointTarget> points;
    };

    class IJointTrajectoryActuator : public IActuator
    {
    public:
        virtual ~IJointTrajectoryActuator() {}
        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const JointTrajectoryActuatorConfig& GetConfig() const = 0;
        virtual bool SetTrajectory(const JointTrajectoryTarget& trajectory) = 0;
        virtual void Update() = 0;
        virtual void ClearTrajectory() = 0;
    };
}