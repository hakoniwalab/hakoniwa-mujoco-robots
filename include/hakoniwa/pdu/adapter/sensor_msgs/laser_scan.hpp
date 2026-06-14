#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/laser_scan.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class LaserScanPduAdapter
    {
    public:
        LaserScanPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::lidar::LaserScanFrame& frame)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_LaserScan& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_LaserScan,
            hako::pdu::msgs::sensor_msgs::LaserScan> endpoint_;
    };
}
