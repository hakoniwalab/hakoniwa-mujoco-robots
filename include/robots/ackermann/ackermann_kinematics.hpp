#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hako::robots::ackermann
{
struct Geometry
{
    double wheelbase {1.55};
    double track_width {1.04};
    double wheel_radius {0.25};
    double max_steering_angle {0.70};
    double max_wheel_angular_velocity {20.0};
};

struct Targets
{
    double front_left_steering {0.0};
    double front_right_steering {0.0};
    double rear_left_wheel_velocity {0.0};
    double rear_right_wheel_velocity {0.0};
};

inline Targets ToJointTargets(
    double linear_velocity,
    double steering_angle,
    const Geometry& geometry)
{
    const double delta = std::clamp(
        steering_angle,
        -geometry.max_steering_angle,
        geometry.max_steering_angle);
    const double base_wheel_velocity = std::clamp(
        linear_velocity / geometry.wheel_radius,
        -geometry.max_wheel_angular_velocity,
        geometry.max_wheel_angular_velocity);

    Targets out {};
    out.rear_left_wheel_velocity = base_wheel_velocity;
    out.rear_right_wheel_velocity = base_wheel_velocity;
    if (std::abs(delta) < 1.0e-9) {
        return out;
    }

    const double turn_radius = geometry.wheelbase / std::tan(delta);
    out.front_left_steering = std::atan2(
        geometry.wheelbase,
        turn_radius - geometry.track_width / 2.0);
    out.front_right_steering = std::atan2(
        geometry.wheelbase,
        turn_radius + geometry.track_width / 2.0);

    // atan2 returns the opposite branch for a negative turn radius. Keep both
    // wheel angles continuous around zero and bounded by the physical stops.
    if (delta < 0.0) {
        if (out.front_left_steering > 0.0) {
            out.front_left_steering -= std::numbers::pi;
        }
        if (out.front_right_steering > 0.0) {
            out.front_right_steering -= std::numbers::pi;
        }
    }
    out.front_left_steering = std::clamp(
        out.front_left_steering,
        -geometry.max_steering_angle,
        geometry.max_steering_angle);
    out.front_right_steering = std::clamp(
        out.front_right_steering,
        -geometry.max_steering_angle,
        geometry.max_steering_angle);

    const double left_scale = 1.0 - geometry.track_width / (2.0 * turn_radius);
    const double right_scale = 1.0 + geometry.track_width / (2.0 * turn_radius);
    out.rear_left_wheel_velocity = std::clamp(
        base_wheel_velocity * left_scale,
        -geometry.max_wheel_angular_velocity,
        geometry.max_wheel_angular_velocity);
    out.rear_right_wheel_velocity = std::clamp(
        base_wheel_velocity * right_scale,
        -geometry.max_wheel_angular_velocity,
        geometry.max_wheel_angular_velocity);
    return out;
}
}
