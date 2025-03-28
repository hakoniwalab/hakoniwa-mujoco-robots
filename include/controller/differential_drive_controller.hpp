#pragma once

#include "primitive_types.hpp"
#include "pid.hpp"

namespace hako::robots::controller {
    class DifferentialDriveController {
    private:
        PID left_pid;
        PID right_pid;
        double tread_width;
    
    public:
        DifferentialDriveController(double kp, double ki, double kd, double tread)
            : left_pid(kp, ki, kd), right_pid(kp, ki, kd), tread_width(tread) {}
    
        void update(double target_v, double target_w,
                    double current_v_left, double current_v_right,
                    double dt,
                    double& out_torque_left,
                    double& out_torque_right)
        {
            double v_left_target = target_v - (tread_width / 2.0) * target_w;
            double v_right_target = target_v + (tread_width / 2.0) * target_w;
    
            out_torque_left  = left_pid.update(v_left_target, current_v_left, dt);
            out_torque_right = right_pid.update(v_right_target, current_v_right, dt);
        }
    };
}
