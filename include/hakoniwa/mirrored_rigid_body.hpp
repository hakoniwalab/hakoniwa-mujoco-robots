#pragma once

#include <stdexcept>

#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hako_msgs/pdu_cpptype_conv_ImpulseCollision.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hakoniwa/pdu_bound_rigid_body.hpp"

namespace hakoniwa
{
class MirroredRigidBody : public PduBoundRigidBody
{
public:
    MirroredRigidBody(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        hakoniwa::pdu::Endpoint& endpoint,
        PduBoundRigidBodyConfig config)
        : PduBoundRigidBody(std::move(world), std::move(config))
        , endpoint_(endpoint)
    {
        const auto* pos_channel = find_channel_or_null("pos");
        if (pos_channel == nullptr) {
            throw std::runtime_error("MirroredRigidBody requires 'pos' channel");
        }
        pos_channel_ = pos_channel;
        pos_key_ = hakoniwa::pdu::PduKey{robot_name(), pos_channel_->pdu_name};

        if (const auto* impulse_channel = find_channel_or_null("impulse"); impulse_channel != nullptr) {
            impulse_channel_ = impulse_channel;
            impulse_key_ = hakoniwa::pdu::PduKey{robot_name(), impulse_channel_->pdu_name};
        }
    }

    virtual ~MirroredRigidBody() = default;

    // Pull and reflect the latest external state on a best-effort basis.
    virtual bool mirror_from_pdu()
    {
        if (pos_channel_ == nullptr) {
            return false;
        }
        HakoCpp_Twist pose {};
        if (get_pos_endpoint_().recv(pose) != HAKO_PDU_ERR_OK) {
            return false;
        }
        apply_pose_twist(pose);
        return true;
    }

    virtual bool publish_impulse(const HakoCpp_ImpulseCollision& impulse)
    {
        if (impulse_channel_ == nullptr) {
            return false;
        }
        auto copy = impulse;
        return get_impulse_endpoint_().send(copy) == HAKO_PDU_ERR_OK;
    }

private:
    using TwistEndpoint = hakoniwa::pdu::TypedEndpoint<
        HakoCpp_Twist,
        hako::pdu::msgs::geometry_msgs::Twist>;
    using ImpulseEndpoint = hakoniwa::pdu::TypedEndpoint<
        HakoCpp_ImpulseCollision,
        hako::pdu::msgs::hako_msgs::ImpulseCollision>;

    TwistEndpoint& get_pos_endpoint_()
    {
        if (!pos_endpoint_cache_) {
            pos_endpoint_cache_.emplace(endpoint_, pos_key_);
        }
        return *pos_endpoint_cache_;
    }

    ImpulseEndpoint& get_impulse_endpoint_()
    {
        if (!impulse_endpoint_cache_) {
            impulse_endpoint_cache_.emplace(endpoint_, impulse_key_);
        }
        return *impulse_endpoint_cache_;
    }

    hakoniwa::pdu::Endpoint& endpoint_;
    const PduChannelConfig* pos_channel_ {nullptr};
    const PduChannelConfig* impulse_channel_ {nullptr};
    hakoniwa::pdu::PduKey pos_key_ {"", ""};
    hakoniwa::pdu::PduKey impulse_key_ {"", ""};
    std::optional<TwistEndpoint> pos_endpoint_cache_ {};
    std::optional<ImpulseEndpoint> impulse_endpoint_cache_ {};
};
}  // namespace hakoniwa
