#pragma once

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_Float64MultiArray.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64MultiArray.hpp"

#include <vector>

namespace hako::robots::pdu::adapter::std_msgs
{
    class Float64MultiArrayPduAdapter
    {
    public:
        Float64MultiArrayPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool recv(std::vector<double>& out)
        {
            HakoCpp_Float64MultiArray pdu {};
            const auto rc = endpoint_.recv(pdu);
            if (rc != HAKO_PDU_ERR_OK) {
                return false;
            }
            out.assign(pdu.data.begin(), pdu.data.end());
            return true;
        }

        bool recv(HakoCpp_Float64MultiArray& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

        bool send(const std::vector<double>& values)
        {
            HakoCpp_Float64MultiArray pdu {};
            pdu.data.assign(values.begin(), values.end());
            return send(pdu);
        }

        bool send(const HakoCpp_Float64MultiArray& value)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            return endpoint_.send(value) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_Float64MultiArray,
            hako::pdu::msgs::std_msgs::Float64MultiArray> endpoint_;
    };
}
