#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "physics.hpp"
#include "geometry_msgs/pdu_cpptype_Twist.hpp"

namespace hakoniwa
{
enum class PduBoundRigidBodyType {
    Mirrored,
    Controllable
};

enum class PduChannelKind {
    Publish,
    SubscribeLatest,
    SubscribeEvent
};

struct PduChannelConfig
{
    std::string logical_name {};
    std::string pdu_name {};
    PduChannelKind kind {PduChannelKind::Publish};
    bool notify_on_recv {false};
};

struct PduBoundRigidBodyConfig
{
    PduBoundRigidBodyType type {PduBoundRigidBodyType::Mirrored};
    std::string robot_name {};
    std::string body_name {};
    std::vector<PduChannelConfig> channels {};

    std::optional<PduChannelConfig> find_channel(const std::string& logical_name) const
    {
        for (const auto& channel : channels) {
            if (channel.logical_name == logical_name) {
                return channel;
            }
        }
        return std::nullopt;
    }
};

class PduBoundRigidBody
{
public:
    PduBoundRigidBody(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        PduBoundRigidBodyConfig config)
        : world_(std::move(world))
        , config_(std::move(config))
        , body_(world_->getRigidBody(config_.body_name))
    {
    }

    virtual ~PduBoundRigidBody() = default;

    const PduBoundRigidBodyConfig& config() const { return config_; }
    const std::string& robot_name() const { return config_.robot_name; }
    const std::string& body_name() const { return config_.body_name; }

    std::shared_ptr<hako::robots::physics::IRigidBody> rigid_body() const
    {
        return body_;
    }

    hako::robots::types::Position position() const
    {
        return body_->GetPosition();
    }

    hako::robots::types::Euler euler() const
    {
        return body_->GetEuler();
    }

    hako::robots::types::Velocity velocity() const
    {
        return body_->GetVelocity();
    }

    hako::robots::types::BodyVelocity body_velocity() const
    {
        return body_->GetBodyVelocity();
    }

    hako::robots::types::BodyAngularVelocity body_angular_velocity() const
    {
        return body_->GetBodyAngularVelocity();
    }

protected:
    const PduChannelConfig* find_channel_or_null(const std::string& logical_name) const
    {
        for (const auto& channel : config_.channels) {
            if (channel.logical_name == logical_name) {
                return &channel;
            }
        }
        return nullptr;
    }

    void apply_pose_twist(const HakoCpp_Twist& pose)
    {
        mjModel* model = world_->getModel();
        mjData* data = world_->getData();
        if (model == nullptr || data == nullptr) {
            throw std::runtime_error("MuJoCo world is not initialized");
        }
        const int body_id = mj_name2id(model, mjOBJ_BODY, body_name().c_str());
        if (body_id < 0) {
            throw std::runtime_error("MuJoCo body not found: " + body_name());
        }
        if (model->body_jntnum[body_id] <= 0) {
            throw std::runtime_error("MuJoCo body has no joint: " + body_name());
        }
        const int joint_id = model->body_jntadr[body_id];
        if (model->jnt_type[joint_id] != mjJNT_FREE) {
            throw std::runtime_error("MuJoCo body is not backed by a free joint: " + body_name());
        }

        const int qpos_adr = model->jnt_qposadr[joint_id];
        const int qvel_adr = model->jnt_dofadr[joint_id];
        data->qpos[qpos_adr + 0] = pose.linear.x;
        data->qpos[qpos_adr + 1] = pose.linear.y;
        data->qpos[qpos_adr + 2] = pose.linear.z;

        mjtNum euler[3] = {
            static_cast<mjtNum>(pose.angular.x),
            static_cast<mjtNum>(pose.angular.y),
            static_cast<mjtNum>(pose.angular.z)
        };
        mjtNum quat[4] = {};
        mju_euler2Quat(quat, euler, "XYZ");
        data->qpos[qpos_adr + 3] = quat[0];
        data->qpos[qpos_adr + 4] = quat[1];
        data->qpos[qpos_adr + 5] = quat[2];
        data->qpos[qpos_adr + 6] = quat[3];

        for (int i = 0; i < 6; ++i) {
            data->qvel[qvel_adr + i] = 0.0;
        }
        mj_forward(model, data);
    }

    HakoCpp_Twist build_pose_twist() const
    {
        HakoCpp_Twist out {};
        const auto pos = position();
        const auto ang = euler();
        out.linear.x = pos.x;
        out.linear.y = pos.y;
        out.linear.z = pos.z;
        out.angular.x = ang.x;
        out.angular.y = ang.y;
        out.angular.z = ang.z;
        return out;
    }

    HakoCpp_Twist build_velocity_twist() const
    {
        HakoCpp_Twist out {};
        const auto vel = velocity();
        const auto ang = body_angular_velocity();
        out.linear.x = vel.x;
        out.linear.y = vel.y;
        out.linear.z = vel.z;
        out.angular.x = ang.x;
        out.angular.y = ang.y;
        out.angular.z = ang.z;
        return out;
    }

    std::shared_ptr<hako::robots::physics::IWorld> world_ {};
    PduBoundRigidBodyConfig config_ {};
    std::shared_ptr<hako::robots::physics::IRigidBody> body_ {};
};
}  // namespace hakoniwa
