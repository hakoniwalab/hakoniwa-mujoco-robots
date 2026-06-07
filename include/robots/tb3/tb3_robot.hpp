#pragma once

#include <memory>
#include <string>

#include "physics.hpp"
#include "sensors/imu/imu_sensor.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"
#include "sensors/odometry/odometry_sensor.hpp"
#include "sensors/tf/tf_publisher.hpp"

namespace hako::robots::tb3
{
    struct Tb3RuntimeConfig
    {
        std::string endpoint_path {};
        std::string endpoint_name {};
        std::string lidar_config {};
        std::string imu_config {};
        std::string joint_state_config {};
        std::string odom_config {};
        std::string tf_config {};
        std::string asset_name {};
        std::string asset_config_path {};
        std::string left_wheel_actuator_config {};
        std::string right_wheel_actuator_config {};
        double max_linear_velocity {0.22};
        double max_yaw_rate {0.5};
        double max_linear_acceleration {0.1};
        double max_yaw_acceleration {0.5};
        double command_deadzone {0.1};
        double wheel_radius {0.033};
        double wheel_separation {0.16};
        double max_wheel_angular_velocity {12.0};
        double max_wheel_angular_acceleration {25.0};
        double lidar_yaw_bias_deg {0.0};
        double lidar_origin_offset {0.0};
    };

    struct Tb3Command
    {
        double linear_velocity {0.0};
        double yaw_rate {0.0};
    };

    struct Tb3CommandConfig
    {
        double max_linear_velocity {0.22};
        double max_yaw_rate {0.5};
        double command_deadzone {0.1};
    };

    class Tb3Robot
    {
    public:
        Tb3Robot(std::shared_ptr<hako::robots::physics::IWorld> world, Tb3RuntimeConfig config);
        ~Tb3Robot();

        bool Initialize(std::string* error_message = nullptr);
        void ApplyCommand(const Tb3Command& command);
        void Step();

        hako::robots::types::Position GetBasePosition() const;
        hako::robots::types::Euler GetBaseEuler() const;
        hako::robots::types::Position GetBaseScanPosition() const;
        hako::robots::types::Euler GetBaseScanEuler() const;

        bool MaybeBuildImu(
            double sim_timestep,
            double sim_time_sec,
            hako::robots::sensor::ImuFrame& out);
        bool MaybeBuildJointState(
            double sim_timestep,
            double sim_time_sec,
            hako::robots::sensor::JointStateFrame& out);
        bool MaybeBuildOdometry(
            double sim_timestep,
            double sim_time_sec,
            hako::robots::sensor::OdometryFrame& out);
        bool MaybeBuildTf(
            double sim_timestep,
            double sim_time_sec,
            hako::robots::sensor::TfFrame& out);
        bool MaybeBuildLaserScan(
            double sim_timestep,
            hako::robots::sensor::lidar::LaserScanFrame& out);

        void EmitDebugLog(int step) const;

    private:
        class Drive;

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        Tb3RuntimeConfig config_;
        std::unique_ptr<Drive> drive_;
        hako::robots::sensor::lidar::LiDAR2DSensor lidar_sensor_;
        hako::robots::sensor::ImuSensor imu_sensor_;
        hako::robots::sensor::JointStateSensor joint_state_sensor_;
        hako::robots::sensor::OdometryPublisher odom_sensor_;
        hako::robots::sensor::TfPublisher tf_sensor_;
        hako::robots::sensor::lidar::LaserScanFrame last_laser_scan_ {};
        hako::robots::sensor::JointStateFrame last_joint_state_frame_ {};
        double last_left_wheel_target_ {0.0};
        double last_right_wheel_target_ {0.0};
        double raw_linear_velocity_ {0.0};
        double raw_yaw_rate_ {0.0};
        double applied_linear_velocity_ {0.0};
        double applied_yaw_rate_ {0.0};
        double applied_left_wheel_target_ {0.0};
        double applied_right_wheel_target_ {0.0};
    };
}
