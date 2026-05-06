#pragma once

#include <string>

#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera
{
    bool LoadCameraConfigFromJson(const std::string& path, CameraConfig& out);
    bool LoadDepthCameraConfigFromJson(const std::string& path, DepthCameraConfig& out);
    bool LoadRgbdCameraConfigFromJson(const std::string& path, RgbdCameraConfig& out);
    bool LoadStereoCameraConfigFromJson(const std::string& path, StereoCameraConfig& out);
}
