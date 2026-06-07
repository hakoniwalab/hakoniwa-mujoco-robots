#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/image.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_Image.hpp"
#include "sensor_msgs/pdu_cpptype_conv_Image.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class ImagePduAdapter
    {
    public:
        ImagePduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::camera::ImageFrame& frame)
        {
            HakoCpp_Image pdu {};
            if (!hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(frame, pdu)) {
                return false;
            }
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send(const hako::robots::sensor::camera::DepthFrame& frame)
        {
            HakoCpp_Image pdu {};
            if (!hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(frame, pdu)) {
                return false;
            }
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Image,
            hako::pdu::msgs::sensor_msgs::Image> endpoint_;
    };
}
