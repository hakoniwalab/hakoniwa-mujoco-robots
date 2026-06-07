#pragma once

#include "actuator.hpp"
#include "hakoniwa/pdu/converter/std_msgs/float64.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_Float64.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"

namespace hako::robots::pdu::adapter::std_msgs
{
    class JointActuatorPduAdapter
    {
    public:
        JointActuatorPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key,
            hako::robots::actuator::IJointActuator& actuator)
            : endpoint_(endpoint, key)
            , actuator_(actuator)
        {
        }

        bool recv(hako::robots::actuator::JointActuatorTarget& out)
        {
            HakoCpp_Float64 pdu {};
            const auto rc = endpoint_.recv(pdu);
            if (rc != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::std_msgs::ToJointActuatorTarget(pdu);
            return true;
        }

        bool recv_and_apply()
        {
            hako::robots::actuator::JointActuatorTarget target {};
            if (!recv(target)) {
                return false;
            }
            actuator_.SetTarget(target);
            return true;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Float64,
            hako::pdu::msgs::std_msgs::Float64> endpoint_;
        hako::robots::actuator::IJointActuator& actuator_;
    };
}
