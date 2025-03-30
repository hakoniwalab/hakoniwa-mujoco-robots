#pragma once

#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "robots/forklift.hpp"

namespace hako::robots::controller {

    class ForkliftController {
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
#if false
            std::cout << "euler: " << forklift.getEuler().to_string() << std::endl;
            std::cout << "world velocity: " << forklift.getVelocity().to_string() << std::endl;
            std::cout << "body velocity: " << forklift.getBodyVelocity().to_string() << std::endl;
            std::cout << "left_torque: " << left_torque << ", right_torque: " << right_torque << std::endl;
#endif
            forklift.drive_motor(left_torque, right_torque);
            //std::cout << "lift position: " << forklift.getLiftPosition().z << std::endl;
            double lift_torque = lift_ctrl.update(forklift.getLiftPosition().z, target_lift_z, dt);
            forklift.drive_lift(lift_torque);
        }
    
    };
}
