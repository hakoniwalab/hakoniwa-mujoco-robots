#pragma once

#include <optional>
#include <stdexcept>

#include "hakoniwa/pdu/adapter/geometry_msgs/twist.hpp"
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
                get_set_pos_event_reader_().subscribe();
            }
        }
        if (const auto* channel = find_channel_or_null("add_force"); channel != nullptr) {
            add_force_channel_ = channel;
            add_force_key_ = hakoniwa::pdu::PduKey{robot_name(), channel->pdu_name};
            if (add_force_channel_->kind == PduChannelKind::SubscribeEvent) {
                get_add_force_event_reader_().subscribe();
            }
        }
    }

    virtual ~ControllableRigidBody() = default;

    virtual bool process_input_events()
    {
        const bool set_pos_applied =
            handle_set_pos_latest() || handle_set_pos_event();
        const bool force_applied =
            handle_add_force_latest() || handle_add_force_event();
        return set_pos_applied || force_applied;
    }

    virtual bool publish_state()
    {
        const bool pos_published = publish_pose();
        const bool velocity_published = publish_velocity();
        return pos_published || velocity_published;
    }

protected:
    bool handle_set_pos_latest()
    {
        if (set_pos_channel_ == nullptr || set_pos_channel_->kind != PduChannelKind::SubscribeLatest) {
            return false;
        }
        PduRigidBodyPose pose {};
        if (!get_set_pos_reader_().recv_pose(pose)) {
            return false;
        }
        apply_pose(pose);
        return true;
    }

    bool handle_set_pos_event()
    {
        if (set_pos_channel_ == nullptr || set_pos_channel_->kind != PduChannelKind::SubscribeEvent) {
            return false;
        }
        PduRigidBodyPose pose {};
        if (!get_set_pos_event_reader_().take_pose(pose)) {
            return false;
        }
        apply_pose(pose);
        return true;
    }

    bool handle_add_force_event()
    {
        if (add_force_channel_ != nullptr && add_force_channel_->kind == PduChannelKind::SubscribeEvent) {
            PduRigidBodyForce pending {};
            if (get_add_force_event_reader_().take_force(pending)) {
                active_force_ = pending.force;
                active_force_steps_remaining_ = kAddForceHoldSteps;
            }
        }
        if (active_force_steps_remaining_ <= 0) {
            rigid_body()->SetForce(hako::robots::types::Vector3{0.0, 0.0, 0.0});
            return false;
        }
        rigid_body()->SetForce(active_force_);
        active_force_steps_remaining_--;
        return true;
    }

    bool handle_add_force_latest()
    {
        if (add_force_channel_ == nullptr || add_force_channel_->kind != PduChannelKind::SubscribeLatest) {
            return false;
        }
        PduRigidBodyForce force {};
        if (!get_add_force_reader_().recv_force(force)) {
            return false;
        }
        active_force_ = force.force;
        active_force_steps_remaining_ = kAddForceHoldSteps;
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
        return get_pos_writer_().send_pose(build_pose());
    }

    bool publish_velocity()
    {
        if (velocity_channel_ == nullptr) {
            return false;
        }
        return get_velocity_writer_().send_velocity(build_velocity());
    }

private:
    hako::robots::pdu::adapter::geometry_msgs::TwistWriter& get_pos_writer_()
    {
        if (!pos_writer_cache_) {
            pos_writer_cache_.emplace(endpoint_, pos_key_);
        }
        return *pos_writer_cache_;
    }

    hako::robots::pdu::adapter::geometry_msgs::TwistWriter& get_velocity_writer_()
    {
        if (!velocity_writer_cache_) {
            velocity_writer_cache_.emplace(endpoint_, velocity_key_);
        }
        return *velocity_writer_cache_;
    }

    hako::robots::pdu::adapter::geometry_msgs::TwistReader& get_set_pos_reader_()
    {
        if (!set_pos_reader_cache_) {
            set_pos_reader_cache_.emplace(endpoint_, set_pos_key_);
        }
        return *set_pos_reader_cache_;
    }

    hako::robots::pdu::adapter::geometry_msgs::TwistReader& get_add_force_reader_()
    {
        if (!add_force_reader_cache_) {
            add_force_reader_cache_.emplace(endpoint_, add_force_key_);
        }
        return *add_force_reader_cache_;
    }

    hako::robots::pdu::adapter::geometry_msgs::TwistEventReader& get_set_pos_event_reader_()
    {
        if (!set_pos_event_reader_cache_) {
            set_pos_event_reader_cache_.emplace(endpoint_, set_pos_key_);
        }
        return *set_pos_event_reader_cache_;
    }

    hako::robots::pdu::adapter::geometry_msgs::TwistEventReader& get_add_force_event_reader_()
    {
        if (!add_force_event_reader_cache_) {
            add_force_event_reader_cache_.emplace(endpoint_, add_force_key_);
        }
        return *add_force_event_reader_cache_;
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
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistWriter> pos_writer_cache_ {};
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistWriter> velocity_writer_cache_ {};
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistReader> set_pos_reader_cache_ {};
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistReader> add_force_reader_cache_ {};
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistEventReader> set_pos_event_reader_cache_ {};
    std::optional<hako::robots::pdu::adapter::geometry_msgs::TwistEventReader> add_force_event_reader_cache_ {};
    hako::robots::types::Vector3 active_force_ {0.0, 0.0, 0.0};
    int active_force_steps_remaining_ {0};
};
}  // namespace hakoniwa
