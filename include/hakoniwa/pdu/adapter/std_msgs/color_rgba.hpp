#pragma once

#include "hakoniwa/pdu/converter/std_msgs/color_rgba.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_ColorRGBA.hpp"
#include "std_msgs/pdu_cpptype_conv_ColorRGBA.hpp"

namespace hako::robots::pdu::adapter::std_msgs
{
    class ColorRGBAPduAdapter
    {
    public:
        ColorRGBAPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::camera::RGBAColor& color)
        {
            auto pdu = hako::robots::pdu::converter::std_msgs::ToHakoPdu(color);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool capture_and_send(
            hako::robots::sensor::camera::ICameraSensor& sensor,
            int x = -1,
            int y = -1)
        {
            return send(sensor.CaptureAsRGBA(x, y));
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_ColorRGBA,
            hako::pdu::msgs::std_msgs::ColorRGBA> endpoint_;
    };
}
