#pragma once

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_JointState.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_JointState ToHakoPdu(const hako::robots::sensor::JointStateFrame& frame)
    {
        HakoCpp_JointState out {};
        out.header = hako::robots::pdu::converter::ToHakoHeader(frame.header);
        out.name = frame.names;
        out.position = frame.position;
        out.velocity = frame.velocity;
        out.effort = frame.effort;
        return out;
    }
}
