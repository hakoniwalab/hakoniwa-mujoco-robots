#pragma once

#include <memory>
#include <unordered_map>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/binding/frame_binding.hpp"

namespace hako::robots::sensor
{
    struct TfConfig
    {
        OutputBinding output {};
        std::vector<binding::TransformBinding> transforms {};
    };

    struct TransformFrame
    {
        MessageHeader header {};
        std::string child_frame_id {};
        Pose3D transform {};
    };

    struct TfFrame
    {
        std::vector<TransformFrame> transforms {};
    };

    class ITfPublisher : public IStatePublisher
    {
    public:
        virtual ~ITfPublisher() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const TfConfig& GetConfig() const = 0;
        virtual void Build(TfFrame& out) = 0;
    };

    class TfPublisher : public ITfPublisher
    {
    public:
        explicit TfPublisher(std::shared_ptr<hako::robots::physics::IWorld> world);

        bool LoadConfig(const std::string& config_path) override;
        const TfConfig& GetConfig() const override;
        void Build(TfFrame& out) override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

    private:
        std::shared_ptr<hako::robots::physics::IWorld> world_;
        TfConfig config_ {};
        double elapsed_sec_ {0.0};
        std::unordered_map<std::string, std::shared_ptr<hako::robots::physics::IRigidBody>> body_cache_ {};
        std::unordered_map<std::string, std::string> child_to_body_ {};
    };
}
