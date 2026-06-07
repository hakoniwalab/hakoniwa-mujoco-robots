#pragma once

#include "hakoniwa/pdu/converter/tf2_msgs/tf_message.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensors/tf/tf_publisher.hpp"
#include "tf2_msgs/pdu_cpptype_TFMessage.hpp"
#include "tf2_msgs/pdu_cpptype_conv_TFMessage.hpp"

namespace hako::robots::pdu::adapter::tf2_msgs
{
    class TfPduAdapter
    {
    public:
        TfPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::TfFrame& frame)
        {
            auto pdu = hako::robots::pdu::converter::tf2_msgs::ToHakoPdu(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_TFMessage,
            hako::pdu::msgs::tf2_msgs::TFMessage> endpoint_;
    };
}
