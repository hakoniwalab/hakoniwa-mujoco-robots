#pragma once

#include "actuator.hpp"
#include "std_msgs/pdu_cpptype_Float64.hpp"

namespace hako::robots::pdu::converter::std_msgs
{
    inline hako::robots::actuator::JointActuatorTarget ToJointActuatorTarget(
        const HakoCpp_Float64& pdu)
    {
        hako::robots::actuator::JointActuatorTarget out {};
        out.value = pdu.data;
        return out;
    }
}
