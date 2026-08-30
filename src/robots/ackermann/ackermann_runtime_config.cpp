#include "robots/ackermann/ackermann_runtime_config.hpp"

#include <fstream>
#include <cmath>
#include <nlohmann/json.hpp>

namespace hako::robots::ackermann
{
namespace
{
using json = nlohmann::json;

bool read_string(const json& value, const char* key, std::string& out, std::string& error)
{
    if (!value.contains(key) || !value.at(key).is_string() || value.at(key).get<std::string>().empty()) {
        error = std::string("missing or invalid string: ") + key;
        return false;
    }
    out = value.at(key).get<std::string>();
    return true;
}

bool read_number(const json& value, const char* key, double& out, std::string& error)
{
    if (!value.contains(key) || !value.at(key).is_number()) {
        error = std::string("missing or invalid number: ") + key;
        return false;
    }
    out = value.at(key).get<double>();
    return true;
}
}

bool LoadRuntimeConfigFromJson(
    const std::string& path,
    RuntimeConfig& out,
    std::string* error_message)
{
    std::ifstream input(path);
    if (!input) {
        if (error_message) {
            *error_message = "failed to open Ackermann runtime config: " + path;
        }
        return false;
    }
    try {
        json root;
        input >> root;
        RuntimeConfig config {};
        std::string error;
        const auto& geometry = root.at("geometry");
        if (!read_number(geometry, "wheelbase_m", config.geometry.wheelbase, error) ||
            !read_number(geometry, "track_width_m", config.geometry.track_width, error) ||
            !read_number(geometry, "wheel_radius_m", config.geometry.wheel_radius, error) ||
            !read_number(geometry, "max_steering_angle_rad", config.geometry.max_steering_angle, error) ||
            !read_number(geometry, "max_wheel_angular_velocity_rad_s", config.geometry.max_wheel_angular_velocity, error)) {
            throw std::runtime_error("geometry." + error);
        }
        const auto& bindings = root.at("bindings");
        if (!read_string(bindings, "asset_name", config.bindings.asset_name, error) ||
            !read_string(bindings, "pdu_name", config.bindings.pdu_name, error) ||
            !read_string(bindings, "endpoint_name", config.bindings.endpoint_name, error) ||
            !read_string(bindings, "base_freejoint", config.bindings.base_freejoint, error)) {
            throw std::runtime_error("bindings." + error);
        }
        const auto& components = bindings.at("components");
        const auto& joints = bindings.at("joints");
        const auto& actuators = bindings.at("actuators");
        const std::array<const char*, 4> roles {"steering_left", "steering_right", "drive_left", "drive_right"};
        for (std::size_t index = 0; index < roles.size(); ++index) {
            if (!read_string(components, roles[index], config.bindings.component_ids[index], error)) {
                throw std::runtime_error("bindings.components." + error);
            }
        }
        if (!read_string(joints, "steering_left", config.bindings.steering_left_joint, error) ||
            !read_string(joints, "steering_right", config.bindings.steering_right_joint, error) ||
            !read_string(joints, "drive_left", config.bindings.drive_left_joint, error) ||
            !read_string(joints, "drive_right", config.bindings.drive_right_joint, error) ||
            !read_string(actuators, "steering_left", config.bindings.steering_left_actuator, error) ||
            !read_string(actuators, "steering_right", config.bindings.steering_right_actuator, error) ||
            !read_string(actuators, "drive_left", config.bindings.drive_left_actuator, error) ||
            !read_string(actuators, "drive_right", config.bindings.drive_right_actuator, error)) {
            throw std::runtime_error("bindings semantic name: " + error);
        }
        const auto& control = root.at("control");
        if (!read_number(control, "max_linear_velocity_m_s", config.control.max_linear_velocity, error) ||
            !read_number(control, "deadzone", config.control.deadzone, error) ||
            !read_number(control, "max_linear_acceleration_m_s2", config.control.max_linear_acceleration, error) ||
            !read_number(control, "max_steering_rate_rad_s", config.control.max_steering_rate, error)) {
            throw std::runtime_error("control." + error);
        }
        config.control.realtime_sync_steps = control.at("realtime_sync_steps").get<int>();
        const auto& smoke = root.at("smoke");
        if (!read_number(smoke, "duration_sec", config.smoke.duration_sec, error) ||
            !read_number(smoke, "straight_speed_m_s", config.smoke.straight_speed_m_s, error) ||
            !read_number(smoke, "turn_speed_m_s", config.smoke.turn_speed_m_s, error) ||
            !read_number(smoke, "turn_steering_rad", config.smoke.turn_steering_rad, error) ||
            !read_number(smoke, "idle_max_horizontal_drift_m", config.smoke.idle_max_horizontal_drift_m, error) ||
            !read_number(smoke, "straight_min_progress_ratio", config.smoke.straight_min_progress_ratio, error) ||
            !read_number(smoke, "turn_min_displacement_m", config.smoke.turn_min_displacement_m, error) ||
            !read_number(smoke, "turn_min_abs_yaw_rad", config.smoke.turn_min_abs_yaw_rad, error) ||
            !read_number(smoke, "steering_tracking_error_rad", config.smoke.steering_tracking_error_rad, error)) {
            throw std::runtime_error("smoke." + error);
        }
        if (config.geometry.wheelbase <= 0.0 || config.geometry.track_width <= 0.0 ||
            config.geometry.wheel_radius <= 0.0 || config.control.realtime_sync_steps <= 0 ||
            config.geometry.max_steering_angle <= 0.0 ||
            config.geometry.max_wheel_angular_velocity <= 0.0 ||
            config.control.max_linear_velocity <= 0.0 ||
            config.control.deadzone < 0.0 || config.control.deadzone >= 1.0 ||
            config.control.max_linear_acceleration <= 0.0 ||
            config.control.max_steering_rate <= 0.0 ||
            config.smoke.duration_sec <= 0.0 ||
            config.smoke.turn_steering_rad > config.geometry.max_steering_angle) {
            throw std::runtime_error("positive geometry, timing, and duration values are required");
        }
        out = std::move(config);
        return true;
    } catch (const std::exception& exception) {
        if (error_message) {
            *error_message = "failed to parse Ackermann runtime config " + path + ": " + exception.what();
        }
        return false;
    }
}
}
