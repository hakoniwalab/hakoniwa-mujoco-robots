#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "sensors/camera/camera_encoding_utils.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <cstddef>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hako::robots::sensor::camera
{

CameraSensor::CameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name)
    : renderer_(renderer), camera_name_(camera_name)
{
    if (!renderer) {
        throw std::invalid_argument("Renderer is null");
    }
}

CameraSensor::~CameraSensor()
{
}

bool CameraSensor::LoadConfig(const std::string& path)
{
    CameraConfig config {};
    if (!LoadCameraConfigFromJson(path, config)) {
        return false;
    }
    if (!LoadConfig(config)) {
        std::cerr << "Failed to validate CameraConfig loaded from '" << path << "'" << std::endl;
        return false;
    }
    return true;
}

bool CameraSensor::LoadConfig(const CameraConfig& config)
{
    if (config.image.width <= 0 || config.image.height <= 0) {
        std::cerr << "Invalid camera image size" << std::endl;
        return false;
    }
    if (config.update_rate <= 0.0) {
        std::cerr << "Invalid camera update_rate: " << config.update_rate << std::endl;
        return false;
    }
    if (config.horizontal_fov <= 0.0 || config.horizontal_fov > M_PI) {
        std::cerr << "Invalid horizontal_fov: " << config.horizontal_fov << std::endl;
        return false;
    }
    if (config.clip.near <= 0.0 || config.clip.far <= config.clip.near) {
        std::cerr << "Invalid clip range: near=" << config.clip.near
                  << ", far=" << config.clip.far << std::endl;
        return false;
    }
    if (config.image.format != "R8G8B8" &&
        config.image.format != "B8G8R8" &&
        config.image.format != "L8")
    {
        std::cerr << "Invalid camera image format: " << config.image.format << std::endl;
        return false;
    }

    config_ = config;
    StartScheduler(config_.update_rate);
    return true;
}

const CameraConfig& CameraSensor::GetConfig() const
{
    return config_;
}

void CameraSensor::Capture(ImageFrame& out)
{
    RawCameraFrame raw;
    bool success = renderer_->Render(
        camera_name_,
        config_.image.width,
        config_.image.height,
        config_.horizontal_fov,
        config_.clip.near,
        config_.clip.far,
        true,   // need_rgb
        false,  // need_depth
        raw
    );

    if (!success) {
        ClearImageFrame(out);
        return;
    }

    // TODO: apply noise from config_.noise after the image noise contract is finalized.

    if (!EncodeImage(raw, config_, out)) {
        std::cerr << "Unsupported camera image format: " << config_.image.format << std::endl;
        ClearImageFrame(out);
    }
}

}
