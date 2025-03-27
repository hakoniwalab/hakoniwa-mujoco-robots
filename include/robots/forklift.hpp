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
    
    public:
        Forklift(std::shared_ptr<hako::robots::physics::IWorld> world) 
            : hako::robots::PhysicsObject(world, "forklift_base")  
        {
            lift   = world->getRigidBody("lift_arm");        
            left_motor  = world->getTorqueActuator("left_motor");
            right_motor = world->getTorqueActuator("right_motor");
            lift_motor  = world->getTorqueActuator("lift_motor");
        }
    
        void drive_motor(double left, double right) {
            left_motor->SetTorque(left);
            right_motor->SetTorque(right);
        }
    
        void drive_lift(double torque) {
            lift_motor->SetTorque(torque);
        }
        hako::robots::types::Position getLiftPosition() const {
            return lift->GetPosition();
        }
    };
}
