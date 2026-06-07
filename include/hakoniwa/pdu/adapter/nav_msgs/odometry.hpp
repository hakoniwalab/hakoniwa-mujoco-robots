#pragma once

#include "hakoniwa/pdu/converter/nav_msgs/odometry.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "nav_msgs/pdu_cpptype_Odometry.hpp"
#include "nav_msgs/pdu_cpptype_conv_Odometry.hpp"
#include "sensors/odometry/odometry_sensor.hpp"

namespace hako::robots::pdu::adapter::nav_msgs
{
    class OdometryPduAdapter
    {
    public:
        OdometryPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::OdometryFrame& frame)
        {
            auto pdu = hako::robots::pdu::converter::nav_msgs::ToHakoPdu(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Odometry,
            hako::pdu::msgs::nav_msgs::Odometry> endpoint_;
    };
}
