#pragma once

#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "trajectory_msgs/pdu_cpptype_conv_JointTrajectory.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include "hakoniwa/pdu/converter/trajectory_msgs/joint_trajectory.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "trajectory_msgs/pdu_cpptype_JointTrajectory.hpp"

#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace hako::robots::pdu::adapter::trajectory_msgs
{
    class JointTrajectoryPduAdapter
    {
    public:
        JointTrajectoryPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool recv(hako::robots::actuator::JointTrajectoryTarget& out)
        {
            HakoCpp_JointTrajectory pdu {};
            if (endpoint_.recv(pdu) != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::trajectory_msgs::
                ToJointTrajectoryTarget(pdu);
            return true;
        }

        bool recv(HakoCpp_JointTrajectory& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_JointTrajectory,
            hako::pdu::msgs::trajectory_msgs::JointTrajectory> endpoint_;
    };

    class JointTrajectoryEventReader
    {
    public:
        JointTrajectoryEventReader(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint)
            , key_(key)
        {
        }

        void subscribe()
        {
            const auto resolved_key = hakoniwa::pdu::PduResolvedKey {
                key_.robot,
                endpoint_.get_pdu_channel_id(key_)
            };
            if (resolved_key.channel_id < 0) {
                throw std::runtime_error(
                    "failed to resolve JointTrajectory event PDU channel");
            }
            if (endpoint_.set_recv_event(resolved_key) != HAKO_PDU_ERR_OK) {
                throw std::runtime_error(
                    "failed to register JointTrajectory receive event");
            }
            endpoint_.subscribe_on_recv_callback(
                resolved_key,
                [this](const hakoniwa::pdu::PduResolvedKey&,
                       std::span<const std::byte> payload) {
                    HakoCpp_JointTrajectory pdu {};
                    hako::pdu::msgs::trajectory_msgs::JointTrajectory converter;
                    std::vector<std::byte> copy(payload.begin(), payload.end());
                    if (!converter.pdu2cpp(
                            reinterpret_cast<char*>(copy.data()), pdu))
                    {
                        return;
                    }
                    auto target = hako::robots::pdu::converter::trajectory_msgs::
                        ToJointTrajectoryTarget(pdu);
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_ = std::move(target);
                });
        }

        bool take(hako::robots::actuator::JointTrajectoryTarget& out)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!pending_.has_value()) {
                return false;
            }
            out = std::move(*pending_);
            pending_.reset();
            return true;
        }

    private:
        hakoniwa::pdu::Endpoint& endpoint_;
        hakoniwa::pdu::PduKey key_ {"", ""};
        std::mutex mutex_ {};
        std::optional<hako::robots::actuator::JointTrajectoryTarget> pending_ {};
    };
}
