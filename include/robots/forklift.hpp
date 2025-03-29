#pragma once

#include "physics.hpp"
#include "physics_object.hpp"

namespace hako::robots {
    class Forklift: public PhysicsObject  {
    private:
        std::shared_ptr<hako::robots::physics::IRigidBody> lift;
        std::shared_ptr<hako::robots::actuator::ITorqueActuator> left_motor;
        std::shared_ptr<hako::robots::actuator::ITorqueActuator> right_motor;
        std::shared_ptr<hako::robots::actuator::ITorqueActuator> lift_motor;
        std::shared_ptr<hako::robots::physics::IRigidBody> left_wheel;
        std::shared_ptr<hako::robots::physics::IRigidBody> right_wheel;
    
    public:
        Forklift(std::shared_ptr<hako::robots::physics::IWorld> world) 
            : hako::robots::PhysicsObject(world, "forklift_base")  
        {
            lift   = world->getRigidBody("lift_arm");        
            left_motor  = world->getTorqueActuator("left_motor");
            right_motor = world->getTorqueActuator("right_motor");
            lift_motor  = world->getTorqueActuator("lift_motor");
            left_wheel  = world->getRigidBody("left_wheel");
            right_wheel = world->getRigidBody("right_wheel");
        }
    
        void drive_motor(double left, double right) {
            left_motor->SetTorque(left);
            right_motor->SetTorque(right);
        }
    
        void drive_lift(double torque) {
            lift_motor->SetTorque(torque);
        }
        hako::robots::types::Position getLiftPosition() const {
            //std::cout << "lift position: " << lift->GetPosition().z << std::endl;
            //std::cout << "left wheel position: " << left_wheel->GetPosition().to_string() << std::endl;
            //std::cout << "right wheel position: " << right_wheel->GetPosition().to_string() << std::endl;
            return lift->GetPosition() - left_wheel->GetPosition();
        }
        hako::robots::types::BodyVelocity getLiftBodyVelocity() const {
            return lift->GetBodyVelocity();
        }
        double getTreadWidth() const {
            double left_y  = left_wheel->GetPosition().y;
            double right_y = right_wheel->GetPosition().y;
            //std::cout << "left_y: " << left_y << ", right_y: " << right_y << std::endl;
            return std::abs(left_y - right_y);
        }
        
    };
}
