#pragma once

#include "sensor.hpp"

namespace hako::robots::sensor
{
    struct TfConfig
    {
        OutputBinding output {};
        std::vector<TransformBinding> transforms {};
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
}
