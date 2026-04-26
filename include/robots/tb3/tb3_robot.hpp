#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "hakoniwa/pdu/hako_msgs/pdu_cpptype_GameControllerOperation.hpp"
#include "hakoniwa/pdu/nav_msgs/pdu_cpptype_Odometry.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_Imu.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_JointState.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "hakoniwa/pdu/tf2_msgs/pdu_cpptype_TFMessage.hpp"
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
        double drive_gain {0.1};
        double turn_gain {0.15};
        double max_torque {1.0};
        double lidar_yaw_bias_deg {0.0};
        double lidar_origin_offset {0.0};
    };

    class Tb3Robot
    {
    public:
        Tb3Robot(std::shared_ptr<hako::robots::physics::IWorld> world, Tb3RuntimeConfig config);
        ~Tb3Robot();

        bool Initialize(std::string* error_message = nullptr);
        void ApplyCommand(const HakoCpp_GameControllerOperation& gamepad, bool has_input);
        void Step();

        void FillBasePose(HakoCpp_Twist& out) const;
        void FillBaseScanPose(HakoCpp_Twist& out) const;

        bool MaybeBuildImu(double sim_timestep, double sim_time_sec, HakoCpp_Imu& out);
        bool MaybeBuildJointState(double sim_timestep, double sim_time_sec, HakoCpp_JointState& out);
        bool MaybeBuildOdometry(double sim_timestep, double sim_time_sec, HakoCpp_Odometry& out);
        bool MaybeBuildTf(double sim_timestep, double sim_time_sec, HakoCpp_TFMessage& out);
        bool MaybeBuildLaserScan(double sim_timestep, HakoCpp_LaserScan& out);

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
        HakoCpp_LaserScan last_laser_scan_ {};
        hako::robots::sensor::JointStateFrame last_joint_state_frame_ {};
        double last_left_torque_ {0.0};
        double last_right_torque_ {0.0};
    };
}
