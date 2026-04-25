#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "primitive_types.hpp"

namespace hako::robots::sensor
{
    struct MessageHeader
    {
        double stamp_sec {0.0};
        std::string frame_id {};
    };

    struct Quaternion
    {
        double x {0.0};
        double y {0.0};
        double z {0.0};
        double w {1.0};
    };

    struct Pose3D
    {
        hako::robots::types::Position position {};
        Quaternion orientation {};
    };

    struct Twist3D
    {
        hako::robots::types::Vector3 linear {};
        hako::robots::types::Vector3 angular {};
    };

    struct OutputBinding
    {
        std::string name {};
        std::string pdu_name {};
        double update_rate_hz {10.0};
    };

    class ISensor
    {
    public:
        virtual ~ISensor() = default;
        virtual void Reset() = 0;
        virtual double GetUpdatePeriodSec() const = 0;
        virtual bool ShouldUpdate(double delta_sec) = 0;
    };

    class IStatePublisher
    {
    public:
        virtual ~IStatePublisher() = default;
        virtual void Reset() = 0;
        virtual double GetUpdatePeriodSec() const = 0;
        virtual bool ShouldUpdate(double delta_sec) = 0;
    };

}
