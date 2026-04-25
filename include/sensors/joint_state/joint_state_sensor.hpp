#pragma once

#include "sensor.hpp"

namespace hako::robots::sensor
{
    struct JointBinding
    {
        std::string name {};
        std::string mjcf_joint {};
    };
    struct JointStateConfig
    {
        OutputBinding output {};
        std::vector<JointBinding> joints {};
    };
    struct JointStateFrame
    {
        MessageHeader header {};
        std::vector<std::string> names {};
        std::vector<double> position {};
        std::vector<double> velocity {};
        std::vector<double> effort {};
    };
    class IJointStateSensor : public ISensor
    {
    public:
        virtual ~IJointStateSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const JointStateConfig& GetConfig() const = 0;
        virtual void Build(JointStateFrame& out) = 0;
    };
}
