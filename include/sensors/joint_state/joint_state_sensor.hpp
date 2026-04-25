#pragma once

#include <memory>
#include <vector>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/common/update_scheduler.hpp"

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

    class JointStateSensor : public IJointStateSensor
    {
    public:
        explicit JointStateSensor(std::shared_ptr<hako::robots::physics::IWorld> world);

        bool LoadConfig(const std::string& config_path) override;
        const JointStateConfig& GetConfig() const override;
        void Build(JointStateFrame& out) override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

    private:
        void ResolveJointIds();

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        JointStateConfig config_ {};
        std::vector<int> joint_ids_ {};
        common::UpdateScheduler scheduler_ {};
    };
}
