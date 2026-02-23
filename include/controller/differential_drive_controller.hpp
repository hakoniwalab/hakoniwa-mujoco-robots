#pragma once

#include "primitive_types.hpp"
#include "pid.hpp"

namespace hako::robots::controller {
    class DifferentialDriveController {
    public:
        struct State {
            PID::State v_pid;
            PID::State w_pid;
        };
    private:
        PID v_pid;
        PID w_pid;
        double tread_width;
    
    public:
        DifferentialDriveController(double kp, double ki, double kd, double tread)
            : v_pid(kp, ki, kd), w_pid(kp, ki, kd), tread_width(tread) {}
    
        void update(double target_v, double target_w,
                    double current_v, double current_w,
                    double dt,
                    double& out_torque_left,
                    double& out_torque_right)
        {
            auto v_force  = v_pid.update(target_v, current_v, dt);
            auto w_torque = w_pid.update(target_w, current_w, dt);
            out_torque_left = v_force - w_torque * tread_width / 2.0;
            out_torque_right = v_force + w_torque * tread_width / 2.0;
        }

        State get_state() const {
            State s;
            s.v_pid = v_pid.get_state();
            s.w_pid = w_pid.get_state();
            return s;
        }

        void set_state(const State& s) {
            v_pid.set_state(s.v_pid);
            w_pid.set_state(s.w_pid);
        }
    };
}
