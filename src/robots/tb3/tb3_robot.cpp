#include "robots/tb3/tb3_robot.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>

#include <mujoco/mujoco.h>

namespace hako::robots::tb3
{
namespace
{
    double rate_limit(double current, double target, double max_delta)
    {
        return current + std::clamp(target - current, -max_delta, max_delta);
    }

}

Tb3Robot::Tb3Robot(std::shared_ptr<hako::robots::physics::IWorld> world, Tb3RuntimeConfig config)
    : world_(std::move(world))
    , config_(std::move(config))
    , drive_(std::make_unique<Tb3Drive>(world_))
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

    if (!drive_->LoadConfig(config_.left_wheel_actuator_config, config_.right_wheel_actuator_config)) {
        if (error_message != nullptr) {
            *error_message = "failed to load TB3 actuator configs";
        }
        return false;
    }

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

void Tb3Robot::ApplyCommand(const Tb3Command& command)
{
    raw_linear_velocity_ = std::clamp(
        command.linear_velocity,
        -config_.max_linear_velocity,
        config_.max_linear_velocity);
    raw_yaw_rate_ = std::clamp(
        command.yaw_rate,
        -config_.max_yaw_rate,
        config_.max_yaw_rate);

    const mjModel* model = world_->getModel();
    const double dt = (model != nullptr && model->opt.timestep > 0.0) ? model->opt.timestep : 0.001;
    applied_linear_velocity_ = rate_limit(
        applied_linear_velocity_,
        raw_linear_velocity_,
        std::max(0.0, config_.max_linear_acceleration) * dt);
    applied_yaw_rate_ = rate_limit(
        applied_yaw_rate_,
        raw_yaw_rate_,
        std::max(0.0, config_.max_yaw_acceleration) * dt);

    double target_left_wheel = 0.0;
    double target_right_wheel = 0.0;
    if (config_.wheel_radius > 0.0) {
        target_left_wheel = (
            applied_linear_velocity_ - applied_yaw_rate_ * config_.wheel_separation * 0.5) /
            config_.wheel_radius;
        target_right_wheel = (
            applied_linear_velocity_ + applied_yaw_rate_ * config_.wheel_separation * 0.5) /
            config_.wheel_radius;
        target_left_wheel = std::clamp(
            target_left_wheel,
            -config_.max_wheel_angular_velocity,
            config_.max_wheel_angular_velocity);
        target_right_wheel = std::clamp(
            target_right_wheel,
            -config_.max_wheel_angular_velocity,
            config_.max_wheel_angular_velocity);
    }

    const double max_wheel_delta = std::max(0.0, config_.max_wheel_angular_acceleration) * dt;
    applied_left_wheel_target_ = rate_limit(applied_left_wheel_target_, target_left_wheel, max_wheel_delta);
    applied_right_wheel_target_ = rate_limit(applied_right_wheel_target_, target_right_wheel, max_wheel_delta);
    last_left_wheel_target_ = target_left_wheel;
    last_right_wheel_target_ = target_right_wheel;

    drive_->SetWheelVelocityTarget(applied_left_wheel_target_, applied_right_wheel_target_);
}

void Tb3Robot::Step()
{
    world_->advanceTimeStep();
}

hako::robots::types::Position Tb3Robot::GetBasePosition() const
{
    return drive_->BasePosition();
}

hako::robots::types::Euler Tb3Robot::GetBaseEuler() const
{
    return drive_->BaseEuler();
}

hako::robots::types::Position Tb3Robot::GetBaseScanPosition() const
{
    return drive_->BaseScanPosition();
}

hako::robots::types::Euler Tb3Robot::GetBaseScanEuler() const
{
    return drive_->BaseScanEuler();
}

bool Tb3Robot::MaybeBuildImu(
    double sim_timestep,
    double sim_time_sec,
    hako::robots::sensor::ImuFrame& out)
{
    if (!imu_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    imu_sensor_.Build(out);
    out.header.frame_id = imu_sensor_.GetConfig().frame_id;
    out.header.stamp_sec = sim_time_sec;
    return true;
}

bool Tb3Robot::MaybeBuildJointState(
    double sim_timestep,
    double sim_time_sec,
    hako::robots::sensor::JointStateFrame& out)
{
    if (!joint_state_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    last_joint_state_frame_ = {};
    joint_state_sensor_.Build(last_joint_state_frame_);
    last_joint_state_frame_.header.frame_id = "";
    last_joint_state_frame_.header.stamp_sec = sim_time_sec;
    out = last_joint_state_frame_;
    return true;
}

bool Tb3Robot::MaybeBuildOdometry(
    double sim_timestep,
    double sim_time_sec,
    hako::robots::sensor::OdometryFrame& out)
{
    if (!odom_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    odom_sensor_.Build(out);
    out.header.frame_id = odom_sensor_.GetConfig().frame_id;
    out.header.stamp_sec = sim_time_sec;
    return true;
}

bool Tb3Robot::MaybeBuildTf(
    double sim_timestep,
    double sim_time_sec,
    hako::robots::sensor::TfFrame& out)
{
    if (!tf_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    tf_sensor_.Build(out);
    for (auto& transform : out.transforms) {
        transform.header.stamp_sec = sim_time_sec;
    }
    return true;
}

bool Tb3Robot::MaybeBuildLaserScan(
    double sim_timestep,
    hako::robots::sensor::lidar::LaserScanFrame& out)
{
    if (!lidar_sensor_.ShouldUpdate(sim_timestep)) {
        return false;
    }
    lidar_sensor_.Scan(out);
    last_laser_scan_ = out;
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
    const auto pos = drive_->BasePosition();
    const auto body_vel = drive_->BaseBodyVelocity();
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
              << " cmd=(v=" << applied_linear_velocity_ << ", yaw=" << applied_yaw_rate_ << ")"
              << " wheel_target=(" << last_left_wheel_target_ << ", " << last_right_wheel_target_ << ")"
              << " applied_wheel_target=(" << applied_left_wheel_target_ << ", "
              << applied_right_wheel_target_ << ")"
              << " joint_pos=(" << left_joint_pos << ", " << right_joint_pos << ")"
              << " joint_vel=(" << left_joint_vel << ", " << right_joint_vel << ")"
              << " lidar_hits=" << lidar_hits
              << " lidar_min=" << (std::isfinite(lidar_min) ? lidar_min : -1.0F)
              << " lidar_max=" << lidar_max
              << std::endl;
}
}
