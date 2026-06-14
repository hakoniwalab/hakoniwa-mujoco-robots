#pragma once

#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hakoniwa/pdu/converter/common.hpp"
#include "hakoniwa/pdu/converter/geometry_msgs/twist.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hakoniwa/pdu_bound_rigid_body.hpp"

namespace hako::robots::pdu::adapter::geometry_msgs
{
    class TwistPosePduAdapter
    {
    public:
        TwistPosePduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(
            const hako::robots::types::Position& position,
            const hako::robots::types::Euler& euler)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::ToHakoTwistPose(position, euler);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send(const HakoCpp_Twist& twist)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            return endpoint_.send(twist) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_Twist& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Twist,
            hako::pdu::msgs::geometry_msgs::Twist> endpoint_;
    };

    class TwistReader
    {
    public:
        TwistReader(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool recv_pose(hakoniwa::PduRigidBodyPose& out)
        {
            HakoCpp_Twist pdu {};
            if (endpoint_.recv(pdu) != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::geometry_msgs::ToRigidBodyPose(pdu);
            return true;
        }

        bool recv_force(hakoniwa::PduRigidBodyForce& out)
        {
            HakoCpp_Twist pdu {};
            if (endpoint_.recv(pdu) != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::geometry_msgs::ToRigidBodyForce(pdu);
            return true;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Twist,
            hako::pdu::msgs::geometry_msgs::Twist> endpoint_;
    };

    class TwistWriter
    {
    public:
        TwistWriter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send_pose(const hakoniwa::PduRigidBodyPose& pose)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::geometry_msgs::ToHakoPdu(pose);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send_velocity(const hakoniwa::PduRigidBodyVelocity& velocity)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::geometry_msgs::ToHakoPdu(velocity);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send(const HakoCpp_Twist& twist)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            return endpoint_.send(twist) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Twist,
            hako::pdu::msgs::geometry_msgs::Twist> endpoint_;
    };

    class TwistEventReader
    {
    public:
        TwistEventReader(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint)
            , key_(key)
        {
        }

        void subscribe()
        {
            const auto resolved_key = hakoniwa::pdu::PduResolvedKey{
                key_.robot,
                endpoint_.get_pdu_channel_id(key_)
            };
            if (resolved_key.channel_id < 0) {
                throw std::runtime_error("failed to resolve Twist event PDU channel");
            }
            endpoint_.subscribe_on_recv_callback(
                resolved_key,
                [this](const hakoniwa::pdu::PduResolvedKey&, std::span<const std::byte> payload) {
                    HakoCpp_Twist pdu {};
                    hako::pdu::msgs::geometry_msgs::Twist convertor;
                    std::vector<std::byte> copy(payload.begin(), payload.end());
                    if (!convertor.pdu2cpp(reinterpret_cast<char*>(copy.data()), pdu)) {
                        return;
                    }
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_ = pdu;
                });
        }

        bool take_pose(hakoniwa::PduRigidBodyPose& out)
        {
            std::optional<HakoCpp_Twist> pending = take_pending_();
            if (!pending.has_value()) {
                return false;
            }
            out = hako::robots::pdu::converter::geometry_msgs::ToRigidBodyPose(*pending);
            return true;
        }

        bool take_force(hakoniwa::PduRigidBodyForce& out)
        {
            std::optional<HakoCpp_Twist> pending = take_pending_();
            if (!pending.has_value()) {
                return false;
            }
            out = hako::robots::pdu::converter::geometry_msgs::ToRigidBodyForce(*pending);
            return true;
        }

    private:
        hakoniwa::pdu::Endpoint& endpoint_;
        hakoniwa::pdu::PduKey key_ {"", ""};
        std::mutex mutex_ {};
        std::optional<HakoCpp_Twist> pending_ {};

        std::optional<HakoCpp_Twist> take_pending_()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto out = pending_;
            pending_.reset();
            return out;
        }
    };
}
