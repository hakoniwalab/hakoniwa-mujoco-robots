#pragma once

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_Imu.hpp"
#include "sensors/imu/imu_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_Imu ToHakoPdu(const hako::robots::sensor::ImuFrame& frame)
    {
        HakoCpp_Imu out {};
        out.header = hako::robots::pdu::converter::ToHakoHeader(frame.header);
        out.orientation = hako::robots::pdu::converter::ToHakoQuaternion(frame.orientation);
        out.angular_velocity = hako::robots::pdu::converter::ToHakoVector3(frame.angular_velocity);
        out.linear_acceleration = hako::robots::pdu::converter::ToHakoVector3(frame.linear_acceleration);
        out.orientation_covariance.fill(0.0);
        out.angular_velocity_covariance.fill(0.0);
        out.linear_acceleration_covariance.fill(0.0);
        return out;
    }
}
