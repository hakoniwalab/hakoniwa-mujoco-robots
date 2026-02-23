#pragma once

namespace hako::robots::controller {
    class PID {
    public:
        struct State {
            double integral {0.0};
            double prev_error {0.0};
        };
    private:
        double kp;
        double ki;
        double kd;
        double integral = 0.0;
        double prev_error = 0.0;
    
    public:
        PID(double kp, double ki, double kd)
            : kp(kp), ki(ki), kd(kd) {}
    
        double update(double target, double current, double dt) {
            double error = target - current;
            integral += error * dt;
            double derivative = (error - prev_error) / dt;
            prev_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    
        void reset() {
            integral = 0.0;
            prev_error = 0.0;
        }

        State get_state() const {
            State s;
            s.integral = integral;
            s.prev_error = prev_error;
            return s;
        }

        void set_state(const State& s) {
            integral = s.integral;
            prev_error = s.prev_error;
        }
    };
}
