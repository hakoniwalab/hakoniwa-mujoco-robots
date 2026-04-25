#pragma once

#include "sensor.hpp"

namespace hako::robots::sensor
{
    struct ImuNoiseConfig
    {
        AxisNoiseConfig angular_velocity {};
        AxisNoiseConfig linear_acceleration {};
    };
    struct ImuConfig
    {
        OutputBinding output {};
        LinkBinding link {};
        std::string mode {"ground_truth"};
        ImuNoiseConfig noise {};
    };

    struct ImuFrame
    {
        MessageHeader header {};
        Quaternion orientation {};
        hako::robots::types::Vector3 angular_velocity {};
        hako::robots::types::Vector3 linear_acceleration {};
    };
    class IImuSensor : public ISensor
    {
    public:
        virtual ~IImuSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const ImuConfig& GetConfig() const = 0;
        virtual void Read(ImuFrame& out) = 0;
    };

    class IJointStateSensor : public ISensor
    {
    public:
        virtual ~IJointStateSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const JointStateConfig& GetConfig() const = 0;
        virtual void Read(JointStateFrame& out) = 0;
    };

}