#pragma once

#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera
{
    // Forward declaration from mujoco_camera_renderer.hpp
    struct RawCameraFrame;

    bool EncodeImage(const RawCameraFrame& raw, const CameraConfig& config, ImageFrame& out);
    bool EncodeDepth(const RawCameraFrame& raw, const DepthCameraConfig& config, DepthFrame& out);
    void ClearImageFrame(ImageFrame& out);
    void ClearDepthFrame(DepthFrame& out);
}
