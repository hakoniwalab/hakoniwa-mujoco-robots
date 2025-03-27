#pragma once

#include "primitive_types.hpp"
#include <memory>
#include <string>

namespace hako::robots::physics
{

    class IWorld
    {
    protected:
        mjModel* model = nullptr;
        mjData* data = nullptr;
    public:
        virtual ~IWorld()
        {
            if (data) {
                mj_deleteData(data);
                data = nullptr;
            }
            if (model) {
                mj_deleteModel(model);
                model = nullptr;
            }
        }
        virtual void loadModel(const std::string& model_file) = 0;
        virtual void advanceTimeStep() = 0;
        virtual std::shared_ptr<IRigidBody> getRigidBody(const std::string& model_name) = 0;
    };

    class IRigidBody
    {
    public:
        virtual hako::robots::types::Position GetPosition() = 0;
        virtual hako::robots::types::Euler GetEuler() = 0;
        virtual hako::robots::types::Velocity GetVelocity() = 0;
        virtual hako::robots::types::EulerRate GetEulerRate() = 0;
        virtual hako::robots::types::BodyVelocity GetBodyVelocity() = 0;
        virtual hako::robots::types::BodyAngularVelocity GetBodyAngularVelocity() = 0;

        virtual void SetTorque(const std::string& joint_name, double torque) = 0;
        virtual void SetForce(const std::string& body_name, const hako::robots::types::Vector3& force) = 0;

    };
}
