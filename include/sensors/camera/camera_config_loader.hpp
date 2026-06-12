#pragma once

#include <string>

#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera
{
    struct CameraMjcfBinding
    {
        std::string camera_name {};
        std::string body_name {};
        std::string freejoint_name {};
    };

    struct CameraPduConfig
    {
        std::string pdu_name {};
        double update_rate_hz {0.0};
    };

    struct CameraProfileConfig
    {
        CameraConfig spec {};
        CameraMjcfBinding mjcf_binding {};
        CameraPduConfig pdu_config {};
    };

    bool LoadCameraConfigFromJson(const std::string& path, CameraConfig& out);
    bool LoadCameraProfileConfigFromJson(const std::string& path, CameraProfileConfig& out);
    bool LoadDepthCameraConfigFromJson(const std::string& path, DepthCameraConfig& out);
    bool LoadRgbdCameraConfigFromJson(const std::string& path, RgbdCameraConfig& out);
    bool LoadStereoCameraConfigFromJson(const std::string& path, StereoCameraConfig& out);
}
