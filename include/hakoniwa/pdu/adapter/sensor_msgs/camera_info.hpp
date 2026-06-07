#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/camera_info.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_conv_CameraInfo.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class CameraInfoPduAdapter
    {
    public:
        CameraInfoPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(
            const hako::robots::sensor::camera::CameraConfig& config,
            double timestamp)
        {
            HakoCpp_CameraInfo pdu {};
            if (!hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(config, timestamp, pdu)) {
                return false;
            }
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send(
            const hako::robots::sensor::camera::DepthCameraConfig& config,
            double timestamp)
        {
            HakoCpp_CameraInfo pdu {};
            if (!hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(config, timestamp, pdu)) {
                return false;
            }
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_CameraInfo,
            hako::pdu::msgs::sensor_msgs::CameraInfo> endpoint_;
    };
}
