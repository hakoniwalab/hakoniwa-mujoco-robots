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
            return lift->GetPosition() - left_wheel->GetPosition();
        }
        hako::robots::types::Position getLiftWorldPosition() const {
            return lift->GetPosition();
        }
        hako::robots::types::Position getLiftEuler() const {
            return lift->GetEuler();
        }
        hako::robots::types::BodyVelocity getLiftBodyVelocity() const {
            return lift->GetBodyVelocity();
        }
        double getTreadWidth() const {
            auto left_pos  = left_wheel->GetPosition();
            auto right_pos = right_wheel->GetPosition();
            double dx = left_pos.x - right_pos.x;
            double dy = left_pos.y - right_pos.y;
            double dz = left_pos.z - right_pos.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        
        
    };
}
