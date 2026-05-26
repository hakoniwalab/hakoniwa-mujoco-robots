#pragma once

#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hakoniwa/pdu_bound_rigid_body.hpp"

namespace hakoniwa
{
class ControllableRigidBody : public PduBoundRigidBody
{
public:
    static constexpr int kAddForceHoldSteps = 100;

    ControllableRigidBody(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        hakoniwa::pdu::Endpoint& endpoint,
        PduBoundRigidBodyConfig config)
        : PduBoundRigidBody(std::move(world), std::move(config))
        , endpoint_(endpoint)
    {
        if (const auto* channel = find_channel_or_null("pos"); channel != nullptr) {
            pos_channel_ = channel;
            pos_key_ = hakoniwa::pdu::PduKey{robot_name(), channel->pdu_name};
        }
        if (const auto* channel = find_channel_or_null("velocity"); channel != nullptr) {
            velocity_channel_ = channel;
            velocity_key_ = hakoniwa::pdu::PduKey{robot_name(), channel->pdu_name};
        }
        if (const auto* channel = find_channel_or_null("set_pos"); channel != nullptr) {
            set_pos_channel_ = channel;
            set_pos_key_ = hakoniwa::pdu::PduKey{robot_name(), channel->pdu_name};
            if (set_pos_channel_->kind == PduChannelKind::SubscribeEvent) {
                register_set_pos_callback_();
            }
        }
        if (const auto* channel = find_channel_or_null("add_force"); channel != nullptr) {
            add_force_channel_ = channel;
            add_force_key_ = hakoniwa::pdu::PduKey{robot_name(), channel->pdu_name};
            if (add_force_channel_->kind == PduChannelKind::SubscribeEvent) {
                register_add_force_callback_();
            }
        }
    }

    virtual ~ControllableRigidBody() = default;

    virtual bool process_input_events()
    {
        const bool set_pos_applied =
            handle_set_pos_latest() || handle_set_pos_event();
        const bool force_applied = handle_add_force_event();
        return set_pos_applied || force_applied;
    }

    virtual bool publish_state()
    {
        const bool pos_published = publish_pose();
        const bool velocity_published = publish_velocity();
        return pos_published || velocity_published;
    }

protected:
    using TwistEndpoint = hakoniwa::pdu::TypedEndpoint<
        HakoCpp_Twist,
        hako::pdu::msgs::geometry_msgs::Twist>;

    bool handle_set_pos_latest()
    {
        if (set_pos_channel_ == nullptr || set_pos_channel_->kind != PduChannelKind::SubscribeLatest) {
            return false;
        }
        HakoCpp_Twist pose {};
        if (get_set_pos_endpoint_().recv(pose) != HAKO_PDU_ERR_OK) {
            return false;
        }
        apply_pose_twist(pose);
        return true;
    }

    bool handle_set_pos_event()
    {
        std::optional<HakoCpp_Twist> pending;
        {
            std::lock_guard<std::mutex> lock(set_pos_event_mutex_);
            pending = pending_set_pos_event_;
            pending_set_pos_event_.reset();
        }
        if (!pending.has_value()) {
            return false;
        }
        apply_pose_twist(*pending);
        return true;
    }

    bool handle_add_force_event()
    {
        std::optional<HakoCpp_Twist> pending;
        {
            std::lock_guard<std::mutex> lock(force_event_mutex_);
            pending = pending_force_event_;
            pending_force_event_.reset();
        }
        if (pending.has_value()) {
            active_force_ = hako::robots::types::Vector3{
                pending->linear.x,
                pending->linear.y,
                pending->linear.z
            };
            active_force_steps_remaining_ = kAddForceHoldSteps;
        }
        if (active_force_steps_remaining_ <= 0) {
            rigid_body()->SetForce(hako::robots::types::Vector3{0.0, 0.0, 0.0});
            return false;
        }
        rigid_body()->SetForce(active_force_);
        active_force_steps_remaining_--;
        return true;
    }

    bool publish_pose()
    {
        if (pos_channel_ == nullptr) {
            return false;
        }
        return get_pos_endpoint_().send(build_pose_twist()) == HAKO_PDU_ERR_OK;
    }

    bool publish_velocity()
    {
        if (velocity_channel_ == nullptr) {
            return false;
        }
        return get_velocity_endpoint_().send(build_velocity_twist()) == HAKO_PDU_ERR_OK;
    }

private:
    void register_set_pos_callback_()
    {
        const auto resolved_key = hakoniwa::pdu::PduResolvedKey{
            robot_name(),
            endpoint_.get_pdu_channel_id(set_pos_key_)
        };
        if (resolved_key.channel_id < 0) {
            throw std::runtime_error("failed to resolve set_pos PDU channel for " + robot_name());
        }
        endpoint_.subscribe_on_recv_callback(
            resolved_key,
            [this](const hakoniwa::pdu::PduResolvedKey&, std::span<const std::byte> payload) {
                HakoCpp_Twist pose {};
                hako::pdu::msgs::geometry_msgs::Twist convertor;
                std::vector<std::byte> copy(payload.begin(), payload.end());
                if (!convertor.pdu2cpp(reinterpret_cast<char*>(copy.data()), pose)) {
                    return;
                }
                std::lock_guard<std::mutex> lock(set_pos_event_mutex_);
                pending_set_pos_event_ = pose;
            });
    }

    void register_add_force_callback_()
    {
        const auto resolved_key = hakoniwa::pdu::PduResolvedKey{
            robot_name(),
            endpoint_.get_pdu_channel_id(add_force_key_)
        };
        if (resolved_key.channel_id < 0) {
            throw std::runtime_error("failed to resolve add_force PDU channel for " + robot_name());
        }
        endpoint_.subscribe_on_recv_callback(
            resolved_key,
            [this](const hakoniwa::pdu::PduResolvedKey&, std::span<const std::byte> payload) {
                HakoCpp_Twist force {};
                hako::pdu::msgs::geometry_msgs::Twist convertor;
                std::vector<std::byte> copy(payload.begin(), payload.end());
                if (!convertor.pdu2cpp(reinterpret_cast<char*>(copy.data()), force)) {
                    return;
                }
                std::lock_guard<std::mutex> lock(force_event_mutex_);
                pending_force_event_ = force;
            });
    }

    TwistEndpoint& get_pos_endpoint_()
    {
        if (!pos_endpoint_cache_) {
            pos_endpoint_cache_.emplace(endpoint_, pos_key_);
        }
        return *pos_endpoint_cache_;
    }

    TwistEndpoint& get_velocity_endpoint_()
    {
        if (!velocity_endpoint_cache_) {
            velocity_endpoint_cache_.emplace(endpoint_, velocity_key_);
        }
        return *velocity_endpoint_cache_;
    }

    TwistEndpoint& get_set_pos_endpoint_()
    {
        if (!set_pos_endpoint_cache_) {
            set_pos_endpoint_cache_.emplace(endpoint_, set_pos_key_);
        }
        return *set_pos_endpoint_cache_;
    }

    hakoniwa::pdu::Endpoint& endpoint_;
    const PduChannelConfig* pos_channel_ {nullptr};
    const PduChannelConfig* velocity_channel_ {nullptr};
    const PduChannelConfig* set_pos_channel_ {nullptr};
    const PduChannelConfig* add_force_channel_ {nullptr};
    hakoniwa::pdu::PduKey pos_key_ {"", ""};
    hakoniwa::pdu::PduKey velocity_key_ {"", ""};
    hakoniwa::pdu::PduKey set_pos_key_ {"", ""};
    hakoniwa::pdu::PduKey add_force_key_ {"", ""};
    std::optional<TwistEndpoint> pos_endpoint_cache_ {};
    std::optional<TwistEndpoint> velocity_endpoint_cache_ {};
    std::optional<TwistEndpoint> set_pos_endpoint_cache_ {};
    std::mutex set_pos_event_mutex_ {};
    std::optional<HakoCpp_Twist> pending_set_pos_event_ {};
    std::mutex force_event_mutex_ {};
    std::optional<HakoCpp_Twist> pending_force_event_ {};
    hako::robots::types::Vector3 active_force_ {0.0, 0.0, 0.0};
    int active_force_steps_remaining_ {0};
};
}  // namespace hakoniwa
