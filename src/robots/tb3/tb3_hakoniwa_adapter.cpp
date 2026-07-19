#include "robots/tb3/tb3_hakoniwa_adapter.hpp"

#include <iostream>

#include "hakoniwa/pdu/adapter/geometry_msgs/twist.hpp"
#include "hakoniwa/pdu/adapter/hako_msgs/game_controller_operation.hpp"
#include "hakoniwa/pdu/adapter/nav_msgs/odometry.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/imu.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/joint_state.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/laser_scan.hpp"
#include "hakoniwa/pdu/adapter/tf2_msgs/tf_message.hpp"

namespace hako::robots::tb3
{
Tb3HakoniwaAdapter::Tb3HakoniwaAdapter(
    hakoniwa::pdu::Endpoint& endpoint,
    const hako::robots::config::AssetManifest& manifest,
    const Tb3RuntimeConfig& runtime)
    : endpoint_(endpoint)
    , manifest_(manifest)
    , runtime_(runtime)
{
}

Tb3HakoniwaAdapter::~Tb3HakoniwaAdapter() = default;

bool Tb3HakoniwaAdapter::ResolveManifestPduKey(
    const std::string& component_id,
    const std::string& config_path,
    hakoniwa::pdu::PduKey& out,
    std::string* error_message) const
{
    const std::string robot_name = manifest_.ComponentPduRobot(component_id);
    if (robot_name.empty()) {
        if (error_message != nullptr) {
            *error_message = "manifest component '" + component_id +
                "' must define pdu_robot for PDU connection";
        }
        return false;
    }

    std::string pdu_name;
    std::string error;
    if (!hako::robots::config::ReadPduNameFromConfig(config_path, pdu_name, &error)) {
        if (error_message != nullptr) {
            *error_message = "failed to read PDU name for component '" +
                component_id + "': " + error;
        }
        return false;
    }
    out = hakoniwa::pdu::PduKey {robot_name, pdu_name};
    return true;
}

bool Tb3HakoniwaAdapter::Initialize(std::string* error_message)
{
    const hakoniwa::pdu::PduKey gamepad_key {runtime_.pdu_robot_name, "hako_cmd_game"};
    const hakoniwa::pdu::PduKey base_pose_key {runtime_.pdu_robot_name, "base_link_pos"};
    const hakoniwa::pdu::PduKey base_scan_pose_key {runtime_.pdu_robot_name, "base_scan_pos"};

    hakoniwa::pdu::PduKey laser_scan_key;
    hakoniwa::pdu::PduKey imu_key;
    hakoniwa::pdu::PduKey joint_state_key;
    hakoniwa::pdu::PduKey odom_key;
    hakoniwa::pdu::PduKey tf_key;
    if (!ResolveManifestPduKey("lidar", runtime_.lidar_config, laser_scan_key, error_message) ||
        !ResolveManifestPduKey("imu", runtime_.imu_config, imu_key, error_message) ||
        !ResolveManifestPduKey("wheel_joint_states", runtime_.joint_state_config, joint_state_key, error_message) ||
        !ResolveManifestPduKey("odometry", runtime_.odom_config, odom_key, error_message) ||
        !ResolveManifestPduKey("tf", runtime_.tf_config, tf_key, error_message))
    {
        return false;
    }

    Tb3CommandConfig command_config {};
    command_config.max_linear_velocity = runtime_.max_linear_velocity;
    command_config.max_yaw_rate = runtime_.max_yaw_rate;
    command_config.command_deadzone = runtime_.command_deadzone;

    gamepad_adapter_ = std::make_unique<hako::robots::pdu::adapter::hako_msgs::GamepadCommandPduAdapter>(
        endpoint_,
        gamepad_key,
        command_config);
    base_pose_adapter_ = std::make_unique<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter>(
        endpoint_,
        base_pose_key);
    base_scan_pose_adapter_ = std::make_unique<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter>(
        endpoint_,
        base_scan_pose_key);
    laser_scan_adapter_ = std::make_unique<hako::robots::pdu::adapter::sensor_msgs::LaserScanPduAdapter>(
        endpoint_,
        laser_scan_key);
    imu_adapter_ = std::make_unique<hako::robots::pdu::adapter::sensor_msgs::ImuPduAdapter>(
        endpoint_,
        imu_key);
    joint_state_adapter_ = std::make_unique<hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter>(
        endpoint_,
        joint_state_key);
    odom_adapter_ = std::make_unique<hako::robots::pdu::adapter::nav_msgs::OdometryPduAdapter>(
        endpoint_,
        odom_key);
    tf_adapter_ = std::make_unique<hako::robots::pdu::adapter::tf2_msgs::TfPduAdapter>(
        endpoint_,
        tf_key);

    if (!SeedNeutralGamepadCommand()) {
        std::cerr << "[WARN] Failed to seed neutral TB3 gamepad command PDU." << std::endl;
    }
    return true;
}

bool Tb3HakoniwaAdapter::SeedNeutralGamepadCommand() const
{
    if (gamepad_adapter_ == nullptr) {
        return false;
    }
    HakoCpp_GameControllerOperation neutral {};
    return gamepad_adapter_->send(neutral);
}

bool Tb3HakoniwaAdapter::RecvCommand(Tb3Command& out)
{
    return gamepad_adapter_ != nullptr && gamepad_adapter_->recv(out);
}

bool Tb3HakoniwaAdapter::PublishBasePose(
    const hako::robots::types::Position& position,
    const hako::robots::types::Euler& euler)
{
    return base_pose_adapter_ != nullptr && base_pose_adapter_->send(position, euler);
}

bool Tb3HakoniwaAdapter::PublishBaseScanPose(
    const hako::robots::types::Position& position,
    const hako::robots::types::Euler& euler)
{
    return base_scan_pose_adapter_ != nullptr && base_scan_pose_adapter_->send(position, euler);
}

bool Tb3HakoniwaAdapter::PublishLaserScan(const hako::robots::sensor::lidar::LaserScanFrame& frame)
{
    return laser_scan_adapter_ != nullptr && laser_scan_adapter_->send(frame);
}

bool Tb3HakoniwaAdapter::PublishImu(const hako::robots::sensor::ImuFrame& frame)
{
    return imu_adapter_ != nullptr && imu_adapter_->send(frame);
}

bool Tb3HakoniwaAdapter::PublishJointState(const hako::robots::sensor::JointStateFrame& frame)
{
    return joint_state_adapter_ != nullptr && joint_state_adapter_->send(frame);
}

bool Tb3HakoniwaAdapter::PublishOdometry(const hako::robots::sensor::OdometryFrame& frame)
{
    return odom_adapter_ != nullptr && odom_adapter_->send(frame);
}

bool Tb3HakoniwaAdapter::PublishTf(const hako::robots::sensor::TfFrame& frame)
{
    return tf_adapter_ != nullptr && tf_adapter_->send(frame);
}
}
