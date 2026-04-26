#include "robots/tb3/tb3_robot.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>

#include <mujoco/mujoco.h>

#include "actuator/actuator_impl.hpp"

namespace hako::robots::tb3
{
namespace
{
    HakoCpp_LaserScan to_hako_scan(const hako::robots::sensor::lidar::LaserScanFrame& frame)
    {
        HakoCpp_LaserScan out {};
        out.angle_min = frame.angle_min;
        out.angle_max = frame.angle_max;
        out.angle_increment = frame.angle_increment;
        out.time_increment = frame.time_increment;
        out.scan_time = frame.scan_time;
        out.range_min = frame.range_min;
        out.range_max = frame.range_max;
        out.ranges = frame.ranges;
        out.intensities = frame.intensities;
        return out;
    }

    HakoCpp_Time to_hako_time(double stamp_sec)
    {
        HakoCpp_Time out {};
        const double sec_floor = std::floor(stamp_sec);
        out.sec = static_cast<Hako_int32>(sec_floor);
        out.nanosec = static_cast<Hako_uint32>((stamp_sec - sec_floor) * 1.0e9);
        return out;
    }

    HakoCpp_Header to_hako_header(const hako::robots::sensor::MessageHeader& header)
    {
        HakoCpp_Header out {};
        out.stamp = to_hako_time(header.stamp_sec);
        out.frame_id = header.frame_id;
        return out;
    }

    HakoCpp_Quaternion to_hako_quaternion(const hako::robots::sensor::Quaternion& q)
    {
        HakoCpp_Quaternion out {};
        out.x = q.x;
        out.y = q.y;
        out.z = q.z;
        out.w = q.w;
        return out;
    }

    HakoCpp_Vector3 to_hako_vector3(const hako::robots::types::Vector3& v)
    {
        HakoCpp_Vector3 out {};
        out.x = v.x;
        out.y = v.y;
        out.z = v.z;
        return out;
    }

    HakoCpp_Imu to_hako_imu(const hako::robots::sensor::ImuFrame& frame)
    {
        HakoCpp_Imu out {};
        out.header = to_hako_header(frame.header);
        out.orientation = to_hako_quaternion(frame.orientation);
        out.angular_velocity = to_hako_vector3(frame.angular_velocity);
        out.linear_acceleration = to_hako_vector3(frame.linear_acceleration);
        out.orientation_covariance.fill(0.0);
        out.angular_velocity_covariance.fill(0.0);
        out.linear_acceleration_covariance.fill(0.0);
        return out;
    }

    HakoCpp_JointState to_hako_joint_state(const hako::robots::sensor::JointStateFrame& frame)
    {
        HakoCpp_JointState out {};
        out.header = to_hako_header(frame.header);
        out.name = frame.names;
        out.position = frame.position;
        out.velocity = frame.velocity;
        out.effort = frame.effort;
        return out;
    }

    HakoCpp_Odometry to_hako_odometry(const hako::robots::sensor::OdometryFrame& frame)
    {
        HakoCpp_Odometry out {};
        out.header = to_hako_header(frame.header);
        out.child_frame_id = frame.child_frame_id;
        out.pose.pose.position.x = frame.pose.position.x;
        out.pose.pose.position.y = frame.pose.position.y;
        out.pose.pose.position.z = frame.pose.position.z;
        out.pose.pose.orientation = to_hako_quaternion(frame.pose.orientation);
        out.pose.covariance.fill(0.0);
        out.twist.twist.linear = to_hako_vector3(frame.twist.linear);
        out.twist.twist.angular = to_hako_vector3(frame.twist.angular);
        out.twist.covariance.fill(0.0);
        return out;
    }

    HakoCpp_TFMessage to_hako_tf(const hako::robots::sensor::TfFrame& frame)
    {
        HakoCpp_TFMessage out {};
        out.transforms.reserve(frame.transforms.size());
        for (const auto& src : frame.transforms) {
            HakoCpp_TransformStamped dst {};
            dst.header = to_hako_header(src.header);
            dst.child_frame_id = src.child_frame_id;
            dst.transform.translation.x = src.transform.position.x;
            dst.transform.translation.y = src.transform.position.y;
            dst.transform.translation.z = src.transform.position.z;
            dst.transform.rotation = to_hako_quaternion(src.transform.orientation);
            out.transforms.push_back(std::move(dst));
        }
        return out;
    }

    void fill_twist_pose(
        HakoCpp_Twist& out,
        const hako::robots::types::Position& pos,
        const hako::robots::types::Euler& euler)
    {
        out.linear.x = pos.x;
        out.linear.y = pos.y;
        out.linear.z = pos.z;
        out.angular.x = euler.x;
        out.angular.y = euler.y;
        out.angular.z = euler.z;
    }
}

class Tb3Robot::Drive
{
public:
    explicit Drive(std::shared_ptr<hako::robots::physics::IWorld> world)
        : base_(world->getRigidBody("base_link"))
        , base_scan_(world->getRigidBody("base_scan"))
        , left_motor_(world->getTorqueActuator("left_motor"))
        , right_motor_(world->getTorqueActuator("right_motor"))
    {
    }

    void set_torque(double left, double right)
    {
        left_motor_->SetTorque(left);
        right_motor_->SetTorque(right);
    }

    hako::robots::types::Position position() const { return base_->GetPosition(); }
    hako::robots::types::Euler euler() const { return base_->GetEuler(); }
    hako::robots::types::BodyVelocity body_velocity() const { return base_->GetBodyVelocity(); }
    hako::robots::types::Position base_scan_position() const { return base_scan_->GetPosition(); }
    hako::robots::types::Euler base_scan_euler() const { return base_scan_->GetEuler(); }

private:
    std::shared_ptr<hako::robots::physics::IRigidBody> base_;
    std::shared_ptr<hako::robots::physics::IRigidBody> base_scan_;
    std::shared_ptr<hako::robots::actuator::ITorqueActuator> left_motor_;
    std::shared_ptr<hako::robots::actuator::ITorqueActuator> right_motor_;
};

Tb3Robot::Tb3Robot(std::shared_ptr<hako::robots::physics::IWorld> world, Tb3RuntimeConfig config)
    : world_(std::move(world))
    , config_(std::move(config))
    , drive_(std::make_unique<Drive>(world_))
    , lidar_sensor_(world_, "base_scan", "base_footprint")
    , imu_sensor_(world_)
    , joint_state_sensor_(world_)
    , odom_sensor_(world_)
    , tf_sensor_(world_)
{
}

Tb3Robot::~Tb3Robot() = default;

bool Tb3Robot::Initialize(std::string* error_message)
{
    if (!lidar_sensor_.LoadConfig(config_.lidar_config)) {
        if (error_message != nullptr) {
            *error_message = "failed to load LiDAR config: " + config_.lidar_config;
        }
        return false;
    }
    lidar_sensor_.SetRuntimeOptions(config_.lidar_yaw_bias_deg, config_.lidar_origin_offset);

    if (!imu_sensor_.LoadConfig(config_.imu_config) ||
        !joint_state_sensor_.LoadConfig(config_.joint_state_config) ||
        !odom_sensor_.LoadConfig(config_.odom_config) ||
        !tf_sensor_.LoadConfig(config_.tf_config))
    {
        if (error_message != nullptr) {
            *error_message = "failed to load TB3 sensor configs";
        }
        return false;
    }
    return true;
}

void Tb3Robot::ApplyCommand(const HakoCpp_GameControllerOperation& gamepad, bool has_input)
{
    last_left_torque_ = 0.0;
    last_right_torque_ = 0.0;
    if (has_input && gamepad.axis.size() >= 4) {
        const double turn = -1.5 * std::clamp(static_cast<double>(gamepad.axis[0]), -1.0, 1.0);
        const double forward = 0.5 * std::clamp(-static_cast<double>(gamepad.axis[3]), -1.0, 1.0);
        last_left_torque_ = std::clamp(
            (forward - turn * config_.turn_gain) * config_.drive_gain,
            -config_.max_torque,
            config_.max_torque);
        last_right_torque_ = std::clamp(
            (forward + turn * config_.turn_gain) * config_.drive_gain,
            -config_.max_torque,
            config_.max_torque);
    }
    drive_->set_torque(last_left_torque_, last_right_torque_);
}

void Tb3Robot::Step()
{
    world_->advanceTimeStep();
}

void Tb3Robot::FillBasePose(HakoCpp_Twist& out) const
{
    fill_twist_pose(out, drive_->position(), drive_->euler());
}

void Tb3Robot::FillBaseScanPose(HakoCpp_Twist& out) const
{
    fill_twist_pose(out, drive_->base_scan_position(), drive_->base_scan_euler());
}

bool Tb3Robot::MaybeBuildImu(double sim_timestep, double sim_time_sec, HakoCpp_Imu& out)
{
    if (!imu_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    hako::robots::sensor::ImuFrame frame {};
    imu_sensor_.Build(frame);
    frame.header.frame_id = imu_sensor_.GetConfig().frame_id;
    frame.header.stamp_sec = sim_time_sec;
    out = to_hako_imu(frame);
    return true;
}

bool Tb3Robot::MaybeBuildJointState(double sim_timestep, double sim_time_sec, HakoCpp_JointState& out)
{
    if (!joint_state_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    last_joint_state_frame_ = {};
    joint_state_sensor_.Build(last_joint_state_frame_);
    last_joint_state_frame_.header.frame_id = "";
    last_joint_state_frame_.header.stamp_sec = sim_time_sec;
    out = to_hako_joint_state(last_joint_state_frame_);
    return true;
}

bool Tb3Robot::MaybeBuildOdometry(double sim_timestep, double sim_time_sec, HakoCpp_Odometry& out)
{
    if (!odom_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    hako::robots::sensor::OdometryFrame frame {};
    odom_sensor_.Build(frame);
    frame.header.frame_id = odom_sensor_.GetConfig().frame_id;
    frame.header.stamp_sec = sim_time_sec;
    out = to_hako_odometry(frame);
    return true;
}

bool Tb3Robot::MaybeBuildTf(double sim_timestep, double sim_time_sec, HakoCpp_TFMessage& out)
{
    if (!tf_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    hako::robots::sensor::TfFrame frame {};
    tf_sensor_.Build(frame);
    for (auto& transform : frame.transforms) {
        transform.header.stamp_sec = sim_time_sec;
    }
    out = to_hako_tf(frame);
    return true;
}

bool Tb3Robot::MaybeBuildLaserScan(double sim_timestep, HakoCpp_LaserScan& out)
{
    if (!lidar_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    hako::robots::sensor::lidar::LaserScanFrame frame {};
    lidar_sensor_.Scan(frame);
    last_laser_scan_ = to_hako_scan(frame);
    out = last_laser_scan_;
    return true;
}

void Tb3Robot::EmitDebugLog(int step) const
{
    float lidar_min = std::numeric_limits<float>::infinity();
    float lidar_max = 0.0F;
    int lidar_hits = 0;
    for (float v : last_laser_scan_.ranges) {
        if (v >= last_laser_scan_.range_min && v < last_laser_scan_.range_max) {
            lidar_min = std::min(lidar_min, v);
            lidar_max = std::max(lidar_max, v);
            ++lidar_hits;
        }
    }
    const auto pos = drive_->position();
    const auto body_vel = drive_->body_velocity();
    const double left_joint_pos =
        (last_joint_state_frame_.position.size() > 0) ? last_joint_state_frame_.position[0] : 0.0;
    const double right_joint_pos =
        (last_joint_state_frame_.position.size() > 1) ? last_joint_state_frame_.position[1] : 0.0;
    const double left_joint_vel =
        (last_joint_state_frame_.velocity.size() > 0) ? last_joint_state_frame_.velocity[0] : 0.0;
    const double right_joint_vel =
        (last_joint_state_frame_.velocity.size() > 1) ? last_joint_state_frame_.velocity[1] : 0.0;
    std::cout << "[TB3] step=" << step
              << " pos=(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
              << " body_vx=" << body_vel.x
              << " torque=(" << last_left_torque_ << ", " << last_right_torque_ << ")"
              << " joint_pos=(" << left_joint_pos << ", " << right_joint_pos << ")"
              << " joint_vel=(" << left_joint_vel << ", " << right_joint_vel << ")"
              << " lidar_hits=" << lidar_hits
              << " lidar_min=" << (std::isfinite(lidar_min) ? lidar_min : -1.0F)
              << " lidar_max=" << lidar_max
              << std::endl;
}
}
