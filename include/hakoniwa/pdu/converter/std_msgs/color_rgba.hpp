#pragma once

#include "sensors/camera/camera_sensor.hpp"
#include "std_msgs/pdu_cpptype_ColorRGBA.hpp"

namespace hako::robots::pdu::converter::std_msgs
{
    inline HakoCpp_ColorRGBA ToHakoPdu(
        const hako::robots::sensor::camera::RGBAColor& color)
    {
        HakoCpp_ColorRGBA out {};
        out.r = color.r;
        out.g = color.g;
        out.b = color.b;
        out.a = color.a;
        return out;
    }
}
