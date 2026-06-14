#include "robots/tb3/tb3_runtime_config_loader.hpp"

#include <cstdlib>
#include <filesystem>

namespace hako::robots::tb3
{
namespace
{
    std::filesystem::path repo_root_path()
    {
        const char* env = std::getenv("HAKO_TB3_ROOT");
        if (env != nullptr && env[0] != '\0') {
            return std::filesystem::path(env).lexically_normal();
        }

        auto path = std::filesystem::current_path().lexically_normal();
        while (!path.empty()) {
            if (std::filesystem::exists(path / "config/assets/tb3-hakoniwa-asset.json")) {
                return path;
            }
            const auto parent = path.parent_path();
            if (parent == path) {
                break;
            }
            path = parent;
        }

        return std::filesystem::current_path().lexically_normal();
    }

    std::string resolve_repo_path(const std::string& path)
    {
        const std::filesystem::path candidate(path);
        if (candidate.is_absolute()) {
            return candidate.lexically_normal().string();
        }
        return (repo_root_path() / candidate).lexically_normal().string();
    }

    std::string get_env_string(const char* name, const std::string& default_value)
    {
        const char* env = std::getenv(name);
        if (env == nullptr || env[0] == '\0') {
            return default_value;
        }
        return std::string(env);
    }

    double get_env_double(const char* name, double default_value)
    {
        const char* env = std::getenv(name);
        if (env == nullptr || env[0] == '\0') {
            return default_value;
        }
        try {
            return std::stod(env);
        } catch (...) {
            return default_value;
        }
    }

    double get_env_double_compat(const char* name, const char* legacy_name, double default_value)
    {
        const char* env = std::getenv(name);
        if (env != nullptr && env[0] != '\0') {
            return get_env_double(name, default_value);
        }
        return get_env_double(legacy_name, default_value);
    }

    std::string manifest_component_config(
        const hako::robots::config::AssetManifest& manifest,
        const std::string& component_id,
        const char* env_name)
    {
        const std::string default_path = manifest.ComponentConfig(component_id);
        const std::string selected_path = get_env_string(env_name, default_path);
        if (selected_path.empty()) {
            return {};
        }
        return resolve_repo_path(selected_path);
    }
}

std::string GetTb3ManifestPathFromEnvironment()
{
    const std::string default_path =
        (repo_root_path() / "config/assets/tb3-hakoniwa-asset.json").string();
    return resolve_repo_path(get_env_string("HAKO_TB3_MANIFEST_PATH", default_path));
}

Tb3RuntimeConfig LoadTb3RuntimeConfig(
    const hako::robots::config::AssetManifest& manifest,
    const Tb3RuntimeConfigOverrides& overrides)
{
    Tb3RuntimeConfig config {};
    config.endpoint_path = get_env_string("HAKO_TB3_ENDPOINT_CONFIG_PATH", manifest.endpoint);
    config.endpoint_name = get_env_string("HAKO_TB3_ENDPOINT_NAME", "tb3_sim_endpoint");
    config.lidar_config = manifest_component_config(manifest, "lidar", "HAKO_TB3_LIDAR_CONFIG_PATH");
    if (!overrides.lidar_config.empty()) {
        config.lidar_config = resolve_repo_path(overrides.lidar_config);
    }
    config.imu_config = manifest_component_config(manifest, "imu", "HAKO_TB3_IMU_CONFIG_PATH");
    config.joint_state_config =
        manifest_component_config(manifest, "wheel_joint_states", "HAKO_TB3_JOINT_STATE_CONFIG_PATH");
    config.odom_config = manifest_component_config(manifest, "odometry", "HAKO_TB3_ODOM_CONFIG_PATH");
    config.tf_config = manifest_component_config(manifest, "tf", "HAKO_TB3_TF_CONFIG_PATH");
    config.camera_config = manifest_component_config(manifest, "color_camera", "HAKO_TB3_CAMERA_CONFIG_PATH");
    config.left_wheel_actuator_config =
        manifest_component_config(manifest, "left_wheel_actuator", "HAKO_TB3_LEFT_ACTUATOR_CONFIG_PATH");
    config.right_wheel_actuator_config =
        manifest_component_config(manifest, "right_wheel_actuator", "HAKO_TB3_RIGHT_ACTUATOR_CONFIG_PATH");
    config.max_linear_velocity = get_env_double_compat(
        "HAKO_TB3_MAX_LINEAR_VELOCITY",
        "HAKO_TB3_DRIVE_GAIN",
        0.22);
    config.max_yaw_rate = get_env_double_compat(
        "HAKO_TB3_MAX_YAW_RATE",
        "HAKO_TB3_TURN_GAIN",
        1.2);
    config.max_linear_acceleration = get_env_double(
        "HAKO_TB3_MAX_LINEAR_ACCELERATION",
        0.1);
    config.max_yaw_acceleration = get_env_double(
        "HAKO_TB3_MAX_YAW_ACCELERATION",
        0.5);
    config.command_deadzone = get_env_double(
        "HAKO_TB3_COMMAND_DEADZONE",
        0.1);
    config.wheel_radius = get_env_double("HAKO_TB3_WHEEL_RADIUS", 0.033);
    config.wheel_separation = get_env_double("HAKO_TB3_WHEEL_SEPARATION", 0.16);
    config.max_wheel_angular_velocity = get_env_double_compat(
        "HAKO_TB3_MAX_WHEEL_ANGULAR_VELOCITY",
        "HAKO_TB3_MAX_TORQUE",
        12.0);
    config.max_wheel_angular_acceleration = get_env_double(
        "HAKO_TB3_MAX_WHEEL_ANGULAR_ACCELERATION",
        25.0);
    config.lidar_yaw_bias_deg = get_env_double("HAKO_TB3_LIDAR_YAW_BIAS_DEG", 0.0);
    config.lidar_origin_offset = get_env_double("HAKO_TB3_LIDAR_ORIGIN_OFFSET", 0.0);
    config.asset_name = get_env_string("HAKO_ASSET_NAME", "tb3_sim");
    config.asset_config_path = get_env_string("HAKO_ASSET_CONFIG_PATH", manifest.pdu_def);
    return config;
}
}
