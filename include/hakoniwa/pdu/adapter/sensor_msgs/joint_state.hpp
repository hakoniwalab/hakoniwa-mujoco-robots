#pragma once

#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include "hakoniwa/pdu/converter/sensor_msgs/joint_state.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_JointState.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    class JointStatePduAdapter
    {
    public:
        JointStatePduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::JointStateFrame& frame)
        {
            // A Hakoniwa PDU channel is single-writer by convention.
            // Multiple readers may call recv(), but only one component should call send() for this PduKey.
            auto pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_JointState& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_JointState,
            hako::pdu::msgs::sensor_msgs::JointState> endpoint_;
    };
}
