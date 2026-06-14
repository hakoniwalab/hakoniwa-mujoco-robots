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
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::nav_msgs::ToHakoPdu(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_Odometry& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Odometry,
            hako::pdu::msgs::nav_msgs::Odometry> endpoint_;
    };
}
