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
    
    public:
        ForkliftController(std::shared_ptr<physics::IWorld> world)
            : forklift(world),
                lift_ctrl(3.0, 1.5, 1.0),
                drive_ctrl(1.5, 0.0, 0.0, forklift.getTreadWidth()),
                target_lift_z(0.0), target_linear_vel(0.0), target_yaw_rate(0.0)
        {
            dt = world->getModel()->opt.timestep;
        }
        Forklift& getForklift() { return forklift; }
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
            forklift.drive_motor(left_torque, right_torque);
    
            hako::robots::types::Position target_lift_pos(0, 0, target_lift_z);
            double lift_torque = lift_ctrl.update(target_lift_pos, target_lift_z, dt);
            forklift.drive_lift(lift_torque);
        }
    
    };
}
