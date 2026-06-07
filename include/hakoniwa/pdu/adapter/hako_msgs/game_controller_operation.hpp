#pragma once

#include <algorithm>
#include <cmath>

#include "hako_msgs/pdu_cpptype_GameControllerOperation.hpp"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "robots/tb3/tb3_robot.hpp"

namespace hako::robots::pdu::adapter::hako_msgs
{
    class GamepadCommandPduAdapter
    {
    public:
        GamepadCommandPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key,
            hako::robots::tb3::Tb3CommandConfig config)
            : endpoint_(endpoint, key)
            , config_(config)
        {
        }

        bool recv(hako::robots::tb3::Tb3Command& out)
        {
            HakoCpp_GameControllerOperation gamepad {};
            const auto rc = endpoint_.recv(gamepad);
            if (rc != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = to_command(gamepad);
            return true;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_GameControllerOperation,
            hako::pdu::msgs::hako_msgs::GameControllerOperation> endpoint_;
        hako::robots::tb3::Tb3CommandConfig config_ {};

        static double apply_deadzone(double value, double deadzone)
        {
            const double threshold = std::clamp(deadzone, 0.0, 0.95);
            if (std::abs(value) <= threshold) {
                return 0.0;
            }
            const double sign = (value < 0.0) ? -1.0 : 1.0;
            return sign * ((std::abs(value) - threshold) / (1.0 - threshold));
        }

        hako::robots::tb3::Tb3Command to_command(const HakoCpp_GameControllerOperation& gamepad) const
        {
            hako::robots::tb3::Tb3Command out {};
            if (gamepad.axis.size() >= 4) {
                const double turn_axis = apply_deadzone(
                    std::clamp(static_cast<double>(gamepad.axis[0]), -1.0, 1.0),
                    config_.command_deadzone);
                const double forward_axis = apply_deadzone(
                    std::clamp(-static_cast<double>(gamepad.axis[3]), -1.0, 1.0),
                    config_.command_deadzone);
                out.yaw_rate = -turn_axis * config_.max_yaw_rate;
                out.linear_velocity = forward_axis * config_.max_linear_velocity;
            }
            return out;
        }
    };
}
