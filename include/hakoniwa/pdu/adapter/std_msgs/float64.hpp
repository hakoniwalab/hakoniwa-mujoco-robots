#pragma once

#include "hakoniwa/pdu/converter/std_msgs/float64.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_Float64.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"

namespace hako::robots::pdu::adapter::std_msgs
{
    class Float64PduAdapter
    {
    public:
        Float64PduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool recv(double& out)
        {
            HakoCpp_Float64 pdu {};
            const auto rc = endpoint_.recv(pdu);
            if (rc != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::std_msgs::ToDouble(pdu);
            return true;
        }

        bool recv(HakoCpp_Float64& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

        bool send(double value)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            const auto pdu =
                hako::robots::pdu::converter::std_msgs::FromDouble(value);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool send(const HakoCpp_Float64& value)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            return endpoint_.send(value) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Float64,
            hako::pdu::msgs::std_msgs::Float64> endpoint_;
    };
}
