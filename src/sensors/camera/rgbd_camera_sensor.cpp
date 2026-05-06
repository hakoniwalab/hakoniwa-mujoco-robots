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
}

RgbdCameraSensor::RgbdCameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name)
    : renderer_(renderer), camera_name_(camera_name)
{
    if (!renderer) {
        throw std::invalid_argument("Renderer is null");
    }
}

RgbdCameraSensor::~RgbdCameraSensor() = default;

bool RgbdCameraSensor::LoadConfig(const std::string& path)
{
    RgbdCameraConfig config {};
    if (!LoadRgbdCameraConfigFromJson(path, config)) {
        return false;
    }
    if (!LoadConfig(config)) {
        std::cerr << "Failed to validate RgbdCameraConfig loaded from '" << path << "'" << std::endl;
        return false;
    }
    return true;
}

bool RgbdCameraSensor::LoadConfig(const RgbdCameraConfig& config)
{
    if (config.rgb.image.width <= 0 || config.rgb.image.height <= 0) {
        std::cerr << "Invalid RGB image size" << std::endl;
        return false;
    }
    if (config.rgb.update_rate <= 0.0) {
        std::cerr << "Invalid RGB update_rate: " << config.rgb.update_rate << std::endl;
        return false;
    }
    if (config.rgb.horizontal_fov <= 0.0 || config.rgb.horizontal_fov > M_PI) {
        std::cerr << "Invalid RGB horizontal_fov: " << config.rgb.horizontal_fov << std::endl;
        return false;
    }
    if (config.rgb.clip.near <= 0.0 || config.rgb.clip.far <= config.rgb.clip.near) {
        std::cerr << "Invalid RGB clip range: near=" << config.rgb.clip.near
                  << ", far=" << config.rgb.clip.far << std::endl;
        return false;
    }
    if (config.rgb.image.format != "R8G8B8" &&
        config.rgb.image.format != "B8G8R8" &&
        config.rgb.image.format != "L8")
    {
        std::cerr << "Invalid RGB image format: " << config.rgb.image.format << std::endl;
        return false;
    }

    if (config.depth.image.width <= 0 || config.depth.image.height <= 0) {
        std::cerr << "Invalid depth image size" << std::endl;
        return false;
    }
    if (config.depth.update_rate <= 0.0) {
        std::cerr << "Invalid depth update_rate: " << config.depth.update_rate << std::endl;
        return false;
    }
    if (config.depth.horizontal_fov <= 0.0 || config.depth.horizontal_fov > M_PI) {
        std::cerr << "Invalid depth horizontal_fov: " << config.depth.horizontal_fov << std::endl;
        return false;
    }
    if (config.depth.clip.near <= 0.0 || config.depth.clip.far <= config.depth.clip.near) {
        std::cerr << "Invalid depth clip range: near=" << config.depth.clip.near
                  << ", far=" << config.depth.clip.far << std::endl;
        return false;
    }
    if (config.depth.image.format != "DEPTH_F32_M" &&
        config.depth.image.format != "DEPTH_U16_MM")
    {
        std::cerr << "Invalid depth image format: " << config.depth.image.format << std::endl;
        return false;
    }

    if (config.rgb.image.width != config.depth.image.width ||
        config.rgb.image.height != config.depth.image.height)
    {
        std::cerr << "RGB and depth image dimensions must match" << std::endl;
        return false;
    }
    if (std::abs(config.rgb.horizontal_fov - config.depth.horizontal_fov) > kFovMatchEpsilon) {
        std::cerr << "RGB and depth horizontal_fov must match" << std::endl;
        return false;
    }
    if (config.rgb.update_rate != config.depth.update_rate) {
        std::cerr << "Warning: RGB and depth update_rate differ. Using RGB update_rate." << std::endl;
    }

    config_ = config;
    StartScheduler(config_.rgb.update_rate);
    return true;
}

const RgbdCameraConfig& RgbdCameraSensor::GetConfig() const
{
    return config_;
}

void RgbdCameraSensor::Capture(ImageFrame& rgb_out, DepthFrame& depth_out)
{
    RawCameraFrame raw;
    const bool success = renderer_->Render(
        camera_name_,
        config_.rgb.image.width,
        config_.rgb.image.height,
        config_.rgb.horizontal_fov,
        true,
        true,
        raw);

    if (!success) {
        ClearImageFrame(rgb_out);
        ClearDepthFrame(depth_out);
        return;
    }

    // TODO: apply RGB/depth noise after both stream contracts are finalized.
    if (!EncodeImage(raw, config_.rgb, rgb_out)) {
        std::cerr << "Failed to encode RGB frame" << std::endl;
        ClearImageFrame(rgb_out);
    }
    if (!EncodeDepth(raw, config_.depth, depth_out)) {
        std::cerr << "Failed to encode depth frame" << std::endl;
        ClearDepthFrame(depth_out);
    }
}

}
