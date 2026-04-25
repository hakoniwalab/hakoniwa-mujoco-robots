#pragma once

#include <memory>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/noise/noise.hpp"
#include "sensors/noise/noise_config.hpp"

namespace hako::robots::sensor
{
    struct ImuNoiseConfig
    {
        noise::AxisNoiseConfig angular_velocity {};
        noise::AxisNoiseConfig linear_acceleration {};
    };

    struct ImuConfig
    {
        OutputBinding output {};
        std::string frame_id {"imu_link"};
        std::string parent_body {};
        std::string source_body {};
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
        virtual void Build(ImuFrame& out) = 0;
    };

    class ImuSensor : public IImuSensor
    {
    public:
        explicit ImuSensor(std::shared_ptr<hako::robots::physics::IWorld> world);

        bool LoadConfig(const std::string& config_path) override;
        const ImuConfig& GetConfig() const override;
        void Build(ImuFrame& out) override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

    private:
        void RebuildNoisePipeline();

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::shared_ptr<hako::robots::physics::IRigidBody> source_body_;
        ImuConfig config_ {};
        double elapsed_sec_ {0.0};
        hako::robots::types::BodyVelocity prev_body_velocity_ {};
        bool has_prev_velocity_ {false};
        noise::AxisNoisePipeline angular_velocity_noise_;
        noise::AxisNoisePipeline linear_acceleration_noise_;
    };
}
