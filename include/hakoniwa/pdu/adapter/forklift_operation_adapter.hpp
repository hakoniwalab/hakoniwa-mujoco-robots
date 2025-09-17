#pragma once
#include "primitive_types.hpp"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include <cmath>

namespace hako::robots::pdu::adapter {

    // Output command structure for forklift
    struct ForkliftCommand {
        double linear_velocity;  // forward/backward speed (m/s)
        double yaw_rate;         // angular velocity (rad/s)
        double lift_position;    // desired lift height (m)
        bool emergency_stop;     // emergency flag (true = stop immediately)
    };

    class ForkliftOperationCommand {
    private:
        // Max values (can be customized by user)
        double max_linear_vel;
        double max_yaw_rate;
        double max_lift;
        double deadzone_threshold;

        // Axis and button index mapping
        int AXIS_YAW       = 2;  // Left stick LR (yaw control)
        static constexpr int AXIS_LIFT      = 1;  // Left stick UD (lift control)
        static constexpr int AXIS_FORWARD   = 3;  // Right stick UD (move forward/backward)

        // Deadzone filter
        double applyDeadzone(const Hako_float64& input) const {
            return (std::abs(input) < deadzone_threshold) ? 0.0 : input;
        }

    public:
        // Constructor with optional parameter tuning
        ForkliftOperationCommand(double max_v = 1.0,
                                 double max_w = 1.0,
                                 double max_l = 0.4,
                                 double deadzone = 0.1)
            : max_linear_vel(max_v),
              max_yaw_rate(max_w),
              max_lift(max_l),
              deadzone_threshold(deadzone)
        {}
        void setAxisYawIndex(int index) {
            AXIS_YAW = index;
        }

        // Convert raw GamePad input into forklift command
        ForkliftCommand convert(const HakoCpp_GameControllerOperation& data) const {
            
            ForkliftCommand cmd;
            cmd.linear_velocity = - applyDeadzone(data.axis[AXIS_FORWARD]) * max_linear_vel;
            cmd.yaw_rate        = - applyDeadzone(data.axis[AXIS_YAW])     * max_yaw_rate;
            cmd.lift_position   = - applyDeadzone(data.axis[AXIS_LIFT])    * max_lift;

            if (cmd.emergency_stop) {
                // Immediately stop motion when emergency is triggered
                cmd.linear_velocity = 0.0;
                cmd.yaw_rate = 0.0;
            }

            return cmd;
        }
    };
}
