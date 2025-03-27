#pragma once

#include "physics.hpp"

namespace hako::robots {
    class PhysicsObject {
    private:
        std::shared_ptr<hako::robots::physics::IRigidBody> base;        
    public:
        PhysicsObject(std::shared_ptr<hako::robots::physics::IWorld> world, const std::string& model_name) {
            base   = world->getRigidBody(model_name);
        }
        virtual ~PhysicsObject() = default;

        hako::robots::types::Position getPosition() const {
            return base->GetPosition();
        }
        hako::robots::types::Euler getEuler() const {
            return base->GetEuler();
        }
        hako::robots::types::Velocity getVelocity() const {
            return base->GetVelocity();
        }
        hako::robots::types::EulerRate getEulerRate() const {
            return base->GetEulerRate();
        }
        hako::robots::types::BodyVelocity getBodyVelocity() const {
            return base->GetBodyVelocity();
        }
        hako::robots::types::BodyAngularVelocity getBodyAngularVelocity() const {
            return base->GetBodyAngularVelocity();
        }
    };
}