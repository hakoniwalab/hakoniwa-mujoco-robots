#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/camera_config_loader.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hako::robots::sensor::camera
{

DepthCameraSensor::DepthCameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name)
    : renderer_(renderer), camera_name_(camera_name)
{
    if (!renderer) {
        throw std::invalid_argument("Renderer is null");
    }
}

DepthCameraSensor::~DepthCameraSensor() = default;

bool DepthCameraSensor::LoadConfig(const std::string& path)
{
    DepthCameraConfig config {};
    if (!LoadDepthCameraConfigFromJson(path, config)) {
        return false;
    }
    if (!LoadConfig(config)) {
        std::cerr << "Failed to validate DepthCameraConfig loaded from '" << path << "'" << std::endl;
        return false;
    }
    return true;
}

bool DepthCameraSensor::LoadConfig(const DepthCameraConfig& config)
{
    if (config.image.width <= 0 || config.image.height <= 0) {
        std::cerr << "Invalid depth camera image size" << std::endl;
        return false;
    }
    if (config.update_rate_hz <= 0.0) {
        std::cerr << "Invalid depth camera update_rate_hz: " << config.update_rate_hz << std::endl;
        return false;
    }
    if (config.horizontal_fov <= 0.0 || config.horizontal_fov > M_PI) {
        std::cerr << "Invalid depth camera horizontal_fov: " << config.horizontal_fov << std::endl;
        return false;
    }
    if (config.clip.near <= 0.0 || config.clip.far <= config.clip.near) {
        std::cerr << "Invalid depth camera clip range: near=" << config.clip.near
                  << ", far=" << config.clip.far << std::endl;
        return false;
    }
    if (config.image.format != "DEPTH_F32_M" && config.image.format != "DEPTH_U16_MM") {
        std::cerr << "Invalid depth camera image format: " << config.image.format << std::endl;
        return false;
    }

    config_ = config;
    StartScheduler(config_.update_rate_hz);
    return true;
}

const DepthCameraConfig& DepthCameraSensor::GetConfig() const
{
    return config_;
}

void DepthCameraSensor::Capture(DepthFrame& out)
{
    RawCameraFrame raw;
    const bool success = renderer_->Render(
        camera_name_,
        config_.image.width,
        config_.image.height,
        config_.horizontal_fov,
        config_.clip.near,
        config_.clip.far,
        false,
        true,
        raw);

    if (!success) {
        ClearDepthFrame(out);
        return;
    }

    // TODO: apply config_.noise after the validated depth path and the future
    // serialization/noise contracts are finalized together.
    if (!EncodeDepth(raw, config_, out)) {
        std::cerr << "Failed to encode depth frame" << std::endl;
        ClearDepthFrame(out);
    }
}

}
