#pragma once

#include <algorithm>
#include <cmath>

#include "builtin_interfaces/pdu_cpptype_Time.hpp"
#include "geometry_msgs/pdu_cpptype_Quaternion.hpp"
#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "geometry_msgs/pdu_cpptype_Vector3.hpp"
#include "std_msgs/pdu_cpptype_Header.hpp"
#include "primitive_types.hpp"
#include "sensor.hpp"

namespace hako::robots::pdu::converter
{
    inline HakoCpp_Time ToHakoTime(double stamp_sec)
    {
        HakoCpp_Time out {};
        const double sec_floor = std::floor(stamp_sec);
        out.sec = static_cast<Hako_int32>(sec_floor);
        long long nanosec = std::llround((stamp_sec - sec_floor) * 1.0e9);
        if (nanosec >= 1000000000LL) {
            out.sec += 1;
            nanosec -= 1000000000LL;
        }
        out.nanosec = static_cast<Hako_uint32>(std::max<long long>(0, nanosec));
        return out;
    }

    inline HakoCpp_Header ToHakoHeader(const hako::robots::sensor::MessageHeader& header)
    {
        HakoCpp_Header out {};
        out.stamp = ToHakoTime(header.stamp_sec);
        out.frame_id = header.frame_id;
        return out;
    }

    inline HakoCpp_Quaternion ToHakoQuaternion(const hako::robots::sensor::Quaternion& q)
    {
        HakoCpp_Quaternion out {};
        out.x = q.x;
        out.y = q.y;
        out.z = q.z;
        out.w = q.w;
        return out;
    }

    inline HakoCpp_Vector3 ToHakoVector3(const hako::robots::types::Vector3& v)
    {
        HakoCpp_Vector3 out {};
        out.x = v.x;
        out.y = v.y;
        out.z = v.z;
        return out;
    }

    inline HakoCpp_Twist ToHakoTwistPose(
        const hako::robots::types::Position& position,
        const hako::robots::types::Euler& euler)
    {
        HakoCpp_Twist out {};
        out.linear.x = position.x;
        out.linear.y = position.y;
        out.linear.z = position.z;
        out.angular.x = euler.x;
        out.angular.y = euler.y;
        out.angular.z = euler.z;
        return out;
    }
}
