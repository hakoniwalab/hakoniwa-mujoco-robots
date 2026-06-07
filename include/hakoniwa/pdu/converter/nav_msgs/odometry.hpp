#pragma once

#include "hakoniwa/pdu/converter/common.hpp"
#include "nav_msgs/pdu_cpptype_Odometry.hpp"
#include "sensors/odometry/odometry_sensor.hpp"

namespace hako::robots::pdu::converter::nav_msgs
{
    inline HakoCpp_Odometry ToHakoPdu(const hako::robots::sensor::OdometryFrame& frame)
    {
        HakoCpp_Odometry out {};
        out.header = hako::robots::pdu::converter::ToHakoHeader(frame.header);
        out.child_frame_id = frame.child_frame_id;
        out.pose.pose.position.x = frame.pose.position.x;
        out.pose.pose.position.y = frame.pose.position.y;
        out.pose.pose.position.z = frame.pose.position.z;
        out.pose.pose.orientation = hako::robots::pdu::converter::ToHakoQuaternion(frame.pose.orientation);
        out.pose.covariance.fill(0.0);
        out.twist.twist.linear = hako::robots::pdu::converter::ToHakoVector3(frame.twist.linear);
        out.twist.twist.angular = hako::robots::pdu::converter::ToHakoVector3(frame.twist.angular);
        out.twist.covariance.fill(0.0);
        return out;
    }
}
