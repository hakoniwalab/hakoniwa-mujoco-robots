#pragma once

#include "sensor_msgs/pdu_cpptype_Range.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_Range ToHakoPdu(
        const hako::robots::sensor::ultrasonic::UltrasonicConfig& config,
        const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame)
    {
        HakoCpp_Range out {};
        out.header.frame_id = config.frame_id;
        out.radiation_type = static_cast<Hako_uint8>(config.radiation_type);
        out.field_of_view = static_cast<float>(config.cone.horizontal);
        out.min_range = static_cast<float>(config.detection_distance.min);
        out.max_range = static_cast<float>(config.detection_distance.max);
        out.range = static_cast<float>(frame.range);
        return out;
    }
}
