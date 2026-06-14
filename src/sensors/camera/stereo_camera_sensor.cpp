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
namespace
{
constexpr double kFovMatchEpsilon = 1.0e-9;

bool ValidateStereoSideConfig(const CameraConfig& config, const char* side_name)
{
    if (config.image.width <= 0 || config.image.height <= 0) {
        std::cerr << "Invalid " << side_name << " stereo image size" << std::endl;
        return false;
    }
    if (config.update_rate <= 0.0) {
        std::cerr << "Invalid " << side_name << " stereo update_rate: " << config.update_rate << std::endl;
        return false;
    }
    if (config.horizontal_fov <= 0.0 || config.horizontal_fov > M_PI) {
        std::cerr << "Invalid " << side_name << " stereo horizontal_fov: " << config.horizontal_fov << std::endl;
        return false;
    }
    if (config.clip.near <= 0.0 || config.clip.far <= config.clip.near) {
        std::cerr << "Invalid " << side_name << " stereo clip range: near=" << config.clip.near
                  << ", far=" << config.clip.far << std::endl;
        return false;
    }
    if (config.image.format != "R8G8B8" &&
        config.image.format != "B8G8R8" &&
        config.image.format != "L8")
    {
        std::cerr << "Invalid " << side_name << " stereo image format: " << config.image.format << std::endl;
        return false;
    }
    return true;
}
}

StereoCameraSensor::StereoCameraSensor(
    std::shared_ptr<MujocoCameraRenderer> renderer,
    const std::string& left_camera_name,
    const std::string& right_camera_name)
    : renderer_(renderer)
    , left_camera_name_(left_camera_name)
    , right_camera_name_(right_camera_name)
{
    if (!renderer_) {
        throw std::invalid_argument("Renderer is null");
    }
}

StereoCameraSensor::~StereoCameraSensor() = default;

bool StereoCameraSensor::LoadConfig(const std::string& path)
{
    StereoCameraConfig config {};
    if (!LoadStereoCameraConfigFromJson(path, config)) {
        return false;
    }
    if (!LoadConfig(config)) {
        std::cerr << "Failed to validate StereoCameraConfig loaded from '" << path << "'" << std::endl;
        return false;
    }
    return true;
}

bool StereoCameraSensor::LoadConfig(const StereoCameraConfig& config)
{
    if (!ValidateStereoSideConfig(config.left, "left")) {
        return false;
    }
    if (!ValidateStereoSideConfig(config.right, "right")) {
        return false;
    }
    if (config.left.image.width != config.right.image.width ||
        config.left.image.height != config.right.image.height)
    {
        std::cerr << "Stereo left/right image dimensions must match" << std::endl;
        return false;
    }
    if (config.left.image.format != config.right.image.format) {
        std::cerr << "Stereo left/right image formats must match" << std::endl;
        return false;
    }
    if (std::abs(config.left.horizontal_fov - config.right.horizontal_fov) > kFovMatchEpsilon) {
        std::cerr << "Stereo left/right horizontal_fov must match" << std::endl;
        return false;
    }
    if (config.left.update_rate != config.right.update_rate) {
        std::cerr << "Stereo left/right update_rate must match" << std::endl;
        return false;
    }
    if (config.baseline <= 0.0) {
        std::cerr << "Stereo baseline must be > 0: " << config.baseline << std::endl;
        return false;
    }

    config_ = config;
    StartScheduler(config_.left.update_rate);
    return true;
}

const StereoCameraConfig& StereoCameraSensor::GetConfig() const
{
    return config_;
}

void StereoCameraSensor::Capture(ImageFrame& left_out, ImageFrame& right_out)
{
    RawCameraFrame left_raw;
    RawCameraFrame right_raw;
    const bool left_ok = renderer_->Render(
        left_camera_name_,
        config_.left.image.width,
        config_.left.image.height,
        config_.left.horizontal_fov,
        config_.left.clip.near,
        config_.left.clip.far,
        true,
        false,
        left_raw);
    const bool right_ok = renderer_->Render(
        right_camera_name_,
        config_.right.image.width,
        config_.right.image.height,
        config_.right.horizontal_fov,
        config_.right.clip.near,
        config_.right.clip.far,
        true,
        false,
        right_raw);

    if (!left_ok || !right_ok) {
        ClearImageFrame(left_out);
        ClearImageFrame(right_out);
        return;
    }

    const bool left_encoded = EncodeImage(left_raw, config_.left, left_out);
    const bool right_encoded = EncodeImage(right_raw, config_.right, right_out);
    if (!left_encoded || !right_encoded) {
        if (!left_encoded) {
            std::cerr << "Failed to encode left stereo frame" << std::endl;
        }
        if (!right_encoded) {
            std::cerr << "Failed to encode right stereo frame" << std::endl;
        }
        ClearImageFrame(left_out);
        ClearImageFrame(right_out);
        return;
    }

    if (left_out.timestamp != right_out.timestamp) {
        std::cerr << "Warning: stereo timestamps differ: left=" << left_out.timestamp
                  << " right=" << right_out.timestamp << std::endl;
    }
}

}
