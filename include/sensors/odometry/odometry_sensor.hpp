#pragma once

#include <memory>

#include "physics.hpp"
#include "sensor.hpp"

namespace hako::robots::sensor
{
    struct OdometryConfig
    {
        OutputBinding output {};
        std::string frame_id {"odom"};
        std::string child_frame_id {"base_footprint"};
        std::string source_body {};
        std::string mode {"ground_truth"};
    };

    struct OdometryFrame
    {
        MessageHeader header {};
        std::string child_frame_id {"base_footprint"};
        Pose3D pose {};
        Twist3D twist {};
    };

    class IOdometryPublisher : public IStatePublisher
    {
    public:
        virtual ~IOdometryPublisher() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const OdometryConfig& GetConfig() const = 0;
        virtual void Build(OdometryFrame& out) = 0;
    };

    class OdometryPublisher : public IOdometryPublisher
    {
    public:
        explicit OdometryPublisher(std::shared_ptr<hako::robots::physics::IWorld> world);

        bool LoadConfig(const std::string& config_path) override;
        const OdometryConfig& GetConfig() const override;
        void Build(OdometryFrame& out) override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

    private:
        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::shared_ptr<hako::robots::physics::IRigidBody> source_body_;
        OdometryConfig config_ {};
        double elapsed_sec_ {0.0};
    };
}
