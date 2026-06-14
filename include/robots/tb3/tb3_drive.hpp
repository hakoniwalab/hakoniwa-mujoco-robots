#pragma once

#include <memory>
#include <string>

#include "physics.hpp"

namespace hako::robots::tb3
{
    class Tb3Drive
    {
    public:
        explicit Tb3Drive(std::shared_ptr<hako::robots::physics::IWorld> world);

        bool LoadConfig(const std::string& left_config_path, const std::string& right_config_path);
        void SetWheelVelocityTarget(double left, double right);

        hako::robots::types::Position BasePosition() const;
        hako::robots::types::Euler BaseEuler() const;
        hako::robots::types::BodyVelocity BaseBodyVelocity() const;
        hako::robots::types::Position BaseScanPosition() const;
        hako::robots::types::Euler BaseScanEuler() const;

    private:
        std::shared_ptr<hako::robots::physics::IRigidBody> base_;
        std::shared_ptr<hako::robots::physics::IRigidBody> base_scan_;
        std::shared_ptr<hako::robots::actuator::IJointActuator> left_actuator_;
        std::shared_ptr<hako::robots::actuator::IJointActuator> right_actuator_;
    };
}
