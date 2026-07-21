#pragma once

#include <algorithm>

namespace hako::robots::rover
{
    struct DifferentialDriveKinematics
    {
        double wheel_radius {0.0};
        double wheel_separation {0.0};
        double left_wheel_sign {1.0};
        double right_wheel_sign {1.0};
        double max_wheel_angular_velocity {0.0};
    };

    struct WheelVelocityTargets
    {
        double left {0.0};
        double right {0.0};
    };

    inline WheelVelocityTargets ToWheelVelocityTargets(
        double linear_x,
        double angular_z,
        const DifferentialDriveKinematics& kinematics)
    {
        if (kinematics.wheel_radius <= 0.0) {
            return {};
        }

        WheelVelocityTargets targets {
            kinematics.left_wheel_sign *
                (linear_x - angular_z * kinematics.wheel_separation * 0.5) /
                kinematics.wheel_radius,
            kinematics.right_wheel_sign *
                (linear_x + angular_z * kinematics.wheel_separation * 0.5) /
                kinematics.wheel_radius,
        };

        if (kinematics.max_wheel_angular_velocity > 0.0) {
            targets.left = std::clamp(
                targets.left,
                -kinematics.max_wheel_angular_velocity,
                kinematics.max_wheel_angular_velocity);
            targets.right = std::clamp(
                targets.right,
                -kinematics.max_wheel_angular_velocity,
                kinematics.max_wheel_angular_velocity);
        }
        return targets;
    }
}
