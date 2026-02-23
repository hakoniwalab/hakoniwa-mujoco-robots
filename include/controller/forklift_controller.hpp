#pragma once

#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "robots/forklift.hpp"

namespace hako::robots::controller {

    class ForkliftController {
    public:
        struct InternalState {
            double target_lift_z {0.0};
            double target_linear_vel {0.0};
            double target_yaw_rate {0.0};
            PID::State lift_pid;
            PID::State drive_v_pid;
            PID::State drive_w_pid;
        };
    private:
        Forklift forklift;
        controller::SliderController lift_ctrl;
        controller::DifferentialDriveController drive_ctrl;
    
        double dt;
        double target_lift_z;
        double target_linear_vel;
        double target_yaw_rate;
    
        double target_lift_z_max = 1.0;
        double target_lift_z_min = -0.05;
        double delta_pos = 0.01;
        double last_left_torque {0.0};
        double last_right_torque {0.0};
        double last_lift_torque {0.0};
    public:
        ForkliftController(std::shared_ptr<physics::IWorld> world)
            : forklift(world),
                lift_ctrl(1.0, 0.0, 0.5),
                drive_ctrl(5.0, 0.0, 1.0, forklift.getTreadWidth()),
                target_lift_z(0.0), target_linear_vel(0.0), target_yaw_rate(0.0)
        {
            dt = world->getModel()->opt.timestep;
        }
        Forklift& getForklift() { return forklift; }
        void set_delta_pos(double delta) { delta_pos = delta; }
        void update_target_lift_z(double ctrl_value) 
        {
            if (ctrl_value > 0) {
                target_lift_z += delta_pos;
                if (target_lift_z > target_lift_z_max) {
                    target_lift_z = target_lift_z_max;
                }
            } else if (ctrl_value < 0) {
                target_lift_z -= delta_pos;
                if (target_lift_z < target_lift_z_min) {
                    target_lift_z = target_lift_z_min;
                }
            }
            else {
                // do nothing
            }
        }
        void setLiftTarget(double z) { target_lift_z = z; }
        void setVelocityCommand(double v, double yaw_rate) {
            target_linear_vel = v;
            target_yaw_rate = yaw_rate;
        }
        double getLiftTarget() const { return target_lift_z; }
        double getTargetLinearVel() const { return target_linear_vel; }
        double getTargetYawRate() const { return target_yaw_rate; }
        double getLastLeftTorque() const { return last_left_torque; }
        double getLastRightTorque() const { return last_right_torque; }
        double getLastLiftTorque() const { return last_lift_torque; }

        InternalState get_internal_state() const {
            InternalState s;
            s.target_lift_z = target_lift_z;
            s.target_linear_vel = target_linear_vel;
            s.target_yaw_rate = target_yaw_rate;
            s.lift_pid = lift_ctrl.get_pid_state();
            auto drive_state = drive_ctrl.get_state();
            s.drive_v_pid = drive_state.v_pid;
            s.drive_w_pid = drive_state.w_pid;
            return s;
        }

        void set_internal_state(const InternalState& s) {
            target_lift_z = s.target_lift_z;
            target_linear_vel = s.target_linear_vel;
            target_yaw_rate = s.target_yaw_rate;
            lift_ctrl.set_pid_state(s.lift_pid);
            controller::DifferentialDriveController::State d;
            d.v_pid = s.drive_v_pid;
            d.w_pid = s.drive_w_pid;
            drive_ctrl.set_state(d);
        }

        void update() {
            double left_torque = 0.0;
            double right_torque = 0.0;
            drive_ctrl.update(
                target_linear_vel,
                target_yaw_rate,
                forklift.getBodyVelocity().x,
                forklift.getBodyAngularVelocity().z,
                dt,
                left_torque, right_torque
            );
            last_left_torque = left_torque;
            last_right_torque = right_torque;
#if false
            std::cout << "euler: " << forklift.getEuler().to_string() << std::endl;
            std::cout << "world velocity: " << forklift.getVelocity().to_string() << std::endl;
            std::cout << "body velocity: " << forklift.getBodyVelocity().to_string() << std::endl;
            std::cout << "left_torque: " << left_torque << ", right_torque: " << right_torque << std::endl;
#endif
            forklift.drive_motor(left_torque, right_torque);
            //std::cout << "lift position: " << forklift.getLiftPosition().z << std::endl;
            double lift_torque = lift_ctrl.update(forklift.getLiftPosition().z, target_lift_z, dt);
            last_lift_torque = lift_torque;
            forklift.drive_lift(lift_torque);
        }
    
    };
}
