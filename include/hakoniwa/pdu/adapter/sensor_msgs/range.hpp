#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/range.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_Range.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Range.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class RangePduAdapter
    {
    public:
        RangePduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(
            const hako::robots::sensor::ultrasonic::UltrasonicConfig& config,
            const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame)
        {
            auto pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(config, frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Range,
            hako::pdu::msgs::sensor_msgs::Range> endpoint_;
    };
}
