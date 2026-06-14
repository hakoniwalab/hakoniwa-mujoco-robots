#pragma once

#include <memory>
#include <string>

#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "robots/tb3/tb3_robot.hpp"
#include "sensors/imu/imu_sensor.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"
#include "sensors/odometry/odometry_sensor.hpp"
#include "sensors/tf/tf_publisher.hpp"

namespace hako::robots::pdu::adapter::geometry_msgs { class TwistPosePduAdapter; }
namespace hako::robots::pdu::adapter::hako_msgs { class GamepadCommandPduAdapter; }
namespace hako::robots::pdu::adapter::nav_msgs { class OdometryPduAdapter; }
namespace hako::robots::pdu::adapter::sensor_msgs
{
    class ImuPduAdapter;
    class JointStatePduAdapter;
    class LaserScanPduAdapter;
}
namespace hako::robots::pdu::adapter::tf2_msgs { class TfPduAdapter; }

namespace hako::robots::tb3
{
    class Tb3HakoniwaAdapter
    {
    public:
        Tb3HakoniwaAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hako::robots::config::AssetManifest& manifest,
            const Tb3RuntimeConfig& runtime);
        ~Tb3HakoniwaAdapter();

        bool Initialize(std::string* error_message = nullptr);

        bool RecvCommand(Tb3Command& out);
        bool PublishBasePose(
            const hako::robots::types::Position& position,
            const hako::robots::types::Euler& euler);
        bool PublishBaseScanPose(
            const hako::robots::types::Position& position,
            const hako::robots::types::Euler& euler);
        bool PublishLaserScan(const hako::robots::sensor::lidar::LaserScanFrame& frame);
        bool PublishImu(const hako::robots::sensor::ImuFrame& frame);
        bool PublishJointState(const hako::robots::sensor::JointStateFrame& frame);
        bool PublishOdometry(const hako::robots::sensor::OdometryFrame& frame);
        bool PublishTf(const hako::robots::sensor::TfFrame& frame);

    private:
        bool ResolveManifestPduKey(
            const std::string& component_id,
            const std::string& config_path,
            hakoniwa::pdu::PduKey& out,
            std::string* error_message) const;

        hakoniwa::pdu::Endpoint& endpoint_;
        const hako::robots::config::AssetManifest& manifest_;
        const Tb3RuntimeConfig& runtime_;

        std::unique_ptr<hako::robots::pdu::adapter::hako_msgs::GamepadCommandPduAdapter> gamepad_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter> base_pose_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter> base_scan_pose_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::LaserScanPduAdapter> laser_scan_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::ImuPduAdapter> imu_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter> joint_state_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::nav_msgs::OdometryPduAdapter> odom_adapter_;
        std::unique_ptr<hako::robots::pdu::adapter::tf2_msgs::TfPduAdapter> tf_adapter_;
    };
}
