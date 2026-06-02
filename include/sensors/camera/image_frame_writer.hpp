#pragma once

#include "sensors/camera/camera_sensor.hpp"

#include <filesystem>

namespace hako::robots::sensor::camera
{
    bool WriteImageFrameToPng(
        const ImageFrame& frame,
        const std::filesystem::path& path);
}
