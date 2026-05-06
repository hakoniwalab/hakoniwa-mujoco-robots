#include "sensors/camera/camera_config_loader.hpp"

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "sensors/common/json_utils.hpp"

namespace hako::robots::sensor::camera
{
namespace
{
using common::json;

bool LoadRootJson(const std::string& path, json& root)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to load camera config JSON: cannot open file '" << path << "'" << std::endl;
        return false;
    }
    try {
        ifs >> root;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse JSON file '" << path << "': " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool RequireObjectField(const json& root, const char* key, const std::string& path, json& out)
{
    if (!root.contains(key)) {
        std::cerr << "Failed to load camera config JSON: missing field '" << key
                  << "' in '" << path << "'" << std::endl;
        return false;
    }
    if (!root.at(key).is_object()) {
        std::cerr << "Failed to load camera config JSON: field '" << key
                  << "' must be an object in '" << path << "'" << std::endl;
        return false;
    }
    out = root.at(key);
    return true;
}

bool RequireArrayField(const json& root, const char* key, const std::string& path, json& out)
{
    if (!root.contains(key)) {
        std::cerr << "Failed to load camera config JSON: missing field '" << key
                  << "' in '" << path << "'" << std::endl;
        return false;
    }
    if (!root.at(key).is_array()) {
        std::cerr << "Failed to load camera config JSON: field '" << key
                  << "' must be an array in '" << path << "'" << std::endl;
        return false;
    }
    out = root.at(key);
    return true;
}

bool RequireStringField(const json& root, const char* key, const std::string& path, std::string& out)
{
    if (!root.contains(key)) {
        std::cerr << "Failed to load camera config JSON: missing field '" << key
                  << "' in '" << path << "'" << std::endl;
        return false;
    }
    if (!root.at(key).is_string()) {
        std::cerr << "Failed to load camera config JSON: field '" << key
                  << "' must be a string in '" << path << "'" << std::endl;
        return false;
    }
    out = root.at(key).get<std::string>();
    return true;
}

bool RequireNumberField(const json& root, const char* key, const std::string& path, double& out)
{
    if (!root.contains(key)) {
        std::cerr << "Failed to load camera config JSON: missing field '" << key
                  << "' in '" << path << "'" << std::endl;
        return false;
    }
    if (!root.at(key).is_number()) {
        std::cerr << "Failed to load camera config JSON: field '" << key
                  << "' must be a number in '" << path << "'" << std::endl;
        return false;
    }
    out = root.at(key).get<double>();
    return true;
}

bool RequireIntField(const json& root, const char* key, const std::string& path, int& out)
{
    if (!root.contains(key)) {
        std::cerr << "Failed to load camera config JSON: missing field '" << key
                  << "' in '" << path << "'" << std::endl;
        return false;
    }
    if (!root.at(key).is_number_integer()) {
        std::cerr << "Failed to load camera config JSON: field '" << key
                  << "' must be an integer in '" << path << "'" << std::endl;
        return false;
    }
    out = root.at(key).get<int>();
    return true;
}

void LoadNoiseConfigIfPresent(const json& root, CameraNoiseConfig& out)
{
    if (!root.contains("noise") || !root.at("noise").is_object()) {
        return;
    }

    const auto& noise = root.at("noise");
    if (noise.contains("type") && noise.at("type").is_string()) {
        out.type = noise.at("type").get<std::string>();
    }
    if (noise.contains("mean") && noise.at("mean").is_number()) {
        out.mean = noise.at("mean").get<double>();
    }
    if (noise.contains("stddev") && noise.at("stddev").is_number()) {
        out.stddev = noise.at("stddev").get<double>();
    }
}

bool ParseCameraConfigJson(const json& root, const std::string& path, CameraConfig& out)
{
    CameraConfig config {};
    if (!RequireStringField(root, "frame_id", path, config.frame_id)) {
        return false;
    }
    if (!RequireNumberField(root, "update_rate", path, config.update_rate)) {
        return false;
    }
    if (!RequireNumberField(root, "horizontal_fov", path, config.horizontal_fov)) {
        return false;
    }

    json image;
    if (!RequireObjectField(root, "image", path, image)) {
        return false;
    }
    if (!RequireIntField(image, "width", path + ":image", config.image.width)) {
        return false;
    }
    if (!RequireIntField(image, "height", path + ":image", config.image.height)) {
        return false;
    }
    if (!RequireStringField(image, "format", path + ":image", config.image.format)) {
        return false;
    }

    json clip;
    if (!RequireObjectField(root, "clip", path, clip)) {
        return false;
    }
    if (!RequireNumberField(clip, "near", path + ":clip", config.clip.near)) {
        return false;
    }
    if (!RequireNumberField(clip, "far", path + ":clip", config.clip.far)) {
        return false;
    }

    LoadNoiseConfigIfPresent(root, config.noise);
    out = config;
    return true;
}

bool ParseDepthCameraConfigJson(const json& root, const std::string& path, DepthCameraConfig& out)
{
    DepthCameraConfig config {};
    if (!RequireStringField(root, "frame_id", path, config.frame_id)) {
        return false;
    }
    if (!RequireNumberField(root, "update_rate", path, config.update_rate)) {
        return false;
    }
    if (!RequireNumberField(root, "horizontal_fov", path, config.horizontal_fov)) {
        return false;
    }

    json image;
    if (!RequireObjectField(root, "image", path, image)) {
        return false;
    }
    if (!RequireIntField(image, "width", path + ":image", config.image.width)) {
        return false;
    }
    if (!RequireIntField(image, "height", path + ":image", config.image.height)) {
        return false;
    }
    if (!RequireStringField(image, "format", path + ":image", config.image.format)) {
        return false;
    }

    json clip;
    if (!RequireObjectField(root, "clip", path, clip)) {
        return false;
    }
    if (!RequireNumberField(clip, "near", path + ":clip", config.clip.near)) {
        return false;
    }
    if (!RequireNumberField(clip, "far", path + ":clip", config.clip.far)) {
        return false;
    }

    LoadNoiseConfigIfPresent(root, config.noise);
    out = config;
    return true;
}
}

bool LoadCameraConfigFromJson(const std::string& path, CameraConfig& out)
{
    json root;
    if (!LoadRootJson(path, root)) {
        return false;
    }
    return ParseCameraConfigJson(root, path, out);
}

bool LoadDepthCameraConfigFromJson(const std::string& path, DepthCameraConfig& out)
{
    json root;
    if (!LoadRootJson(path, root)) {
        return false;
    }
    return ParseDepthCameraConfigJson(root, path, out);
}

bool LoadRgbdCameraConfigFromJson(const std::string& path, RgbdCameraConfig& out)
{
    json root;
    if (!LoadRootJson(path, root)) {
        return false;
    }

    json rgb;
    if (!RequireObjectField(root, "rgb", path, rgb)) {
        return false;
    }
    json depth;
    if (!RequireObjectField(root, "depth", path, depth)) {
        return false;
    }

    RgbdCameraConfig config {};
    if (!ParseCameraConfigJson(rgb, path + ":rgb", config.rgb)) {
        return false;
    }
    if (!ParseDepthCameraConfigJson(depth, path + ":depth", config.depth)) {
        return false;
    }

    out = config;
    return true;
}

bool LoadStereoCameraConfigFromJson(const std::string& path, StereoCameraConfig& out)
{
    json root;
    if (!LoadRootJson(path, root)) {
        return false;
    }

    json cameras;
    if (!RequireArrayField(root, "cameras", path, cameras)) {
        return false;
    }
    if (cameras.size() < 2) {
        std::cerr << "Failed to load camera config JSON: field 'cameras' must contain at least two entries in '"
                  << path << "'" << std::endl;
        return false;
    }

    StereoCameraConfig config {};
    for (size_t i = 0; i < 2; ++i) {
        if (!cameras.at(i).is_object()) {
            std::cerr << "Failed to load camera config JSON: cameras[" << i
                      << "] must be an object in '" << path << "'" << std::endl;
            return false;
        }
    }

    json left_profile;
    if (!RequireObjectField(cameras.at(0), "camera_profile", path + ":cameras[0]", left_profile)) {
        return false;
    }
    json right_profile;
    if (!RequireObjectField(cameras.at(1), "camera_profile", path + ":cameras[1]", right_profile)) {
        return false;
    }
    if (!ParseCameraConfigJson(left_profile, path + ":cameras[0].camera_profile", config.left)) {
        return false;
    }
    if (!ParseCameraConfigJson(right_profile, path + ":cameras[1].camera_profile", config.right)) {
        return false;
    }

    json left_pose;
    if (!RequireObjectField(cameras.at(0), "pose", path + ":cameras[0]", left_pose)) {
        return false;
    }
    json right_pose;
    if (!RequireObjectField(cameras.at(1), "pose", path + ":cameras[1]", right_pose)) {
        return false;
    }
    json left_position;
    if (!RequireObjectField(left_pose, "position", path + ":cameras[0].pose", left_position)) {
        return false;
    }
    json right_position;
    if (!RequireObjectField(right_pose, "position", path + ":cameras[1].pose", right_position)) {
        return false;
    }

    double left_x = 0.0;
    double left_y = 0.0;
    double left_z = 0.0;
    double right_x = 0.0;
    double right_y = 0.0;
    double right_z = 0.0;
    if (!RequireNumberField(left_position, "x", path + ":cameras[0].pose.position", left_x) ||
        !RequireNumberField(left_position, "y", path + ":cameras[0].pose.position", left_y) ||
        !RequireNumberField(left_position, "z", path + ":cameras[0].pose.position", left_z) ||
        !RequireNumberField(right_position, "x", path + ":cameras[1].pose.position", right_x) ||
        !RequireNumberField(right_position, "y", path + ":cameras[1].pose.position", right_y) ||
        !RequireNumberField(right_position, "z", path + ":cameras[1].pose.position", right_z))
    {
        return false;
    }

    const double dx = left_x - right_x;
    const double dy = left_y - right_y;
    const double dz = left_z - right_z;
    config.baseline = std::sqrt(dx * dx + dy * dy + dz * dz);

    out = config;
    return true;
}
}
