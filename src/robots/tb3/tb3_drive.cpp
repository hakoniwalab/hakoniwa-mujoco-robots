#include "robots/tb3/tb3_drive.hpp"

#include <iostream>
#include <utility>

#include "actuator.hpp"

namespace hako::robots::tb3
{
Tb3Drive::Tb3Drive(std::shared_ptr<hako::robots::physics::IWorld> world)
    : base_(world->getRigidBody("base_link"))
    , base_scan_(world->getRigidBody("base_scan"))
    , left_actuator_(world->createJointActuator())
    , right_actuator_(world->createJointActuator())
{
}

bool Tb3Drive::LoadConfig(const std::string& left_config_path, const std::string& right_config_path)
{
    if (left_actuator_ == nullptr || right_actuator_ == nullptr) {
        std::cerr << "[ERROR] Joint actuator is not supported by this physics world." << std::endl;
        return false;
    }
    if (left_config_path.empty() || right_config_path.empty()) {
        std::cerr << "[ERROR] TB3 actuator config path is empty."
                  << " left='" << left_config_path << "'"
                  << " right='" << right_config_path << "'"
                  << std::endl;
        return false;
    }
    if (!left_actuator_->LoadConfig(left_config_path)) {
        std::cerr << "[ERROR] Failed to load left wheel actuator config: "
                  << left_config_path << std::endl;
        return false;
    }
    if (!right_actuator_->LoadConfig(right_config_path)) {
        std::cerr << "[ERROR] Failed to load right wheel actuator config: "
                  << right_config_path << std::endl;
        return false;
    }
    return true;
}

void Tb3Drive::SetWheelVelocityTarget(double left, double right)
{
    left_actuator_->SetTarget(left);
    right_actuator_->SetTarget(right);
}

hako::robots::types::Position Tb3Drive::BasePosition() const
{
    return base_->GetPosition();
}

hako::robots::types::Euler Tb3Drive::BaseEuler() const
{
    return base_->GetEuler();
}

hako::robots::types::BodyVelocity Tb3Drive::BaseBodyVelocity() const
{
    return base_->GetBodyVelocity();
}

hako::robots::types::Position Tb3Drive::BaseScanPosition() const
{
    return base_scan_->GetPosition();
}

hako::robots::types::Euler Tb3Drive::BaseScanEuler() const
{
    return base_scan_->GetEuler();
}
}
