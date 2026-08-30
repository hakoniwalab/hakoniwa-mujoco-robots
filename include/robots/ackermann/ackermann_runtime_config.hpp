#pragma once

#include "robots/ackermann/ackermann_kinematics.hpp"

#include <array>
#include <string>

namespace hako::robots::ackermann
{
struct RuntimeBindings
{
    std::string asset_name {};
    std::string pdu_name {};
    std::string endpoint_name {};
    std::array<std::string, 4> component_ids {};
    std::string base_freejoint {};
    std::string steering_left_joint {};
    std::string steering_right_joint {};
    std::string drive_left_joint {};
    std::string drive_right_joint {};
    std::string steering_left_actuator {};
    std::string steering_right_actuator {};
    std::string drive_left_actuator {};
    std::string drive_right_actuator {};
};

struct RuntimeControl
{
    double max_linear_velocity {3.5};
    double deadzone {0.06};
    double max_linear_acceleration {6.0};
    double max_steering_rate {5.0};
    int realtime_sync_steps {1};
};

struct RuntimeSmoke
{
    double duration_sec {3.0};
    double straight_speed_m_s {1.0};
    double turn_speed_m_s {1.0};
    double turn_steering_rad {0.25};
    double idle_max_horizontal_drift_m {0.02};
    double straight_min_progress_ratio {0.25};
    double turn_min_displacement_m {0.25};
    double turn_min_abs_yaw_rad {0.20};
    double steering_tracking_error_rad {0.08};
};

struct RuntimeConfig
{
    Geometry geometry {};
    RuntimeBindings bindings {};
    RuntimeControl control {};
    RuntimeSmoke smoke {};
};

bool LoadRuntimeConfigFromJson(
    const std::string& path,
    RuntimeConfig& out,
    std::string* error_message = nullptr);
}
