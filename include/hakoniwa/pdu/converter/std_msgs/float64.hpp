#pragma once

#include "std_msgs/pdu_cpptype_Float64.hpp"

namespace hako::robots::pdu::converter::std_msgs
{
    inline double ToDouble(const HakoCpp_Float64& pdu)
    {
        return pdu.data;
    }

    inline HakoCpp_Float64 FromDouble(double value)
    {
        HakoCpp_Float64 pdu {};
        pdu.data = value;
        return pdu;
    }
}
