#pragma once

#include "physics.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace hako::examples::sensors
{
    class ExampleWorld final : public hako::robots::physics::IWorld
    {
    public:
        void loadModel(const std::string& model_file) override
        {
            char error[1024] {};
            model = mj_loadXML(model_file.c_str(), nullptr, error, sizeof(error));
            if (model == nullptr) {
                throw std::runtime_error(
                    std::string("failed to load MuJoCo model: ") +
                    model_file +
                    "\n" +
                    error);
            }

            data = mj_makeData(model);
            if (data == nullptr) {
                throw std::runtime_error("failed to allocate MuJoCo data");
            }

            mj_forward(model, data);
        }

        void advanceTimeStep() override
        {
            if (model != nullptr && data != nullptr) {
                mj_step(model, data);
            }
        }

        std::shared_ptr<hako::robots::physics::IRigidBody>
        getRigidBody(const std::string& /*model_name*/) override
        {
            return nullptr;
        }

        std::shared_ptr<hako::robots::actuator::ITorqueActuator>
        getTorqueActuator(const std::string& /*name*/) override
        {
            return nullptr;
        }
    };
}
