#pragma once

#include <stdexcept>

#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
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

private:
    using TwistEndpoint = hakoniwa::pdu::TypedEndpoint<
        HakoCpp_Twist,
        hako::pdu::msgs::geometry_msgs::Twist>;

    TwistEndpoint& get_pos_endpoint_()
    {
        if (!pos_endpoint_cache_) {
            pos_endpoint_cache_.emplace(endpoint_, pos_key_);
        }
        return *pos_endpoint_cache_;
    }

    hakoniwa::pdu::Endpoint& endpoint_;
    const PduChannelConfig* pos_channel_ {nullptr};
    hakoniwa::pdu::PduKey pos_key_ {"", ""};
    std::optional<TwistEndpoint> pos_endpoint_cache_ {};
};
}  // namespace hakoniwa
