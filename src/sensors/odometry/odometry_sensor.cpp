#include "sensors/odometry/odometry_sensor.hpp"

#include <utility>
#include "sensors/common/json_utils.hpp"

namespace hako::robots::sensor
{
namespace
{
Quaternion quat_from_mj(const mjtNum* q)
{
    Quaternion out {};
    out.w = q[0];
    out.x = q[1];
    out.y = q[2];
    out.z = q[3];
    return out;
}
}

OdometryPublisher::OdometryPublisher(std::shared_ptr<hako::robots::physics::IWorld> world)
    : world_(std::move(world))
{
}

bool OdometryPublisher::LoadConfig(const std::string& config_path)
{
    common::json root;
    if (!common::load_json_file(config_path, root)) {
        return false;
    }

    config_ = OdometryConfig {};
    config_.output.name = common::get_json_string(root, "name", "odom");
    config_.output.pdu_name = common::get_json_string(root, "pdu_name", "odom");
    config_.output.update_rate_hz = common::get_json_number(root, "update_rate_hz", 50.0);
    config_.frame_id = common::get_json_string(root, "frame_id", "odom");
    config_.child_frame_id = common::get_json_string(root, "child_frame_id", "base_footprint");
    config_.source_body = common::get_json_string(root, "source_body", "base_footprint");
    config_.mode = common::get_json_string(root, "mode", "ground_truth");

    source_body_ = world_->getRigidBody(config_.source_body);
    scheduler_.StartReady(GetUpdatePeriodSec());
    return true;
}

const OdometryConfig& OdometryPublisher::GetConfig() const
{
    return config_;
}

void OdometryPublisher::Build(OdometryFrame& out)
{
    auto* model = world_->getModel();
    auto* data = world_->getData();
    const int body_id = mj_name2id(model, mjOBJ_BODY, config_.source_body.c_str());
    if (body_id < 0) {
        return;
    }

    out.header.frame_id = config_.frame_id;
    out.child_frame_id = config_.child_frame_id;
    out.pose.position = source_body_->GetPosition();
    out.pose.orientation = quat_from_mj(&data->xquat[4 * body_id]);

    const auto body_vel = source_body_->GetBodyVelocity();
    const auto body_ang_vel = source_body_->GetBodyAngularVelocity();
    out.twist.linear.x = body_vel.x;
    out.twist.linear.y = body_vel.y;
    out.twist.linear.z = body_vel.z;
    out.twist.angular.x = body_ang_vel.x;
    out.twist.angular.y = body_ang_vel.y;
    out.twist.angular.z = body_ang_vel.z;
}

void OdometryPublisher::Reset()
{
    scheduler_.Reset();
}

double OdometryPublisher::GetUpdatePeriodSec() const
{
    return (config_.output.update_rate_hz > 0.0) ? (1.0 / config_.output.update_rate_hz) : 0.02;
}

bool OdometryPublisher::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}
}
