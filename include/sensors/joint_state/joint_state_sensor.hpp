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


}