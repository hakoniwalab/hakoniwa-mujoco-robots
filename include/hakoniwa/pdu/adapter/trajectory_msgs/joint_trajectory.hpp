#pragma once

#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "trajectory_msgs/pdu_cpptype_conv_JointTrajectory.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

#include "hakoniwa/pdu/converter/trajectory_msgs/joint_trajectory.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "trajectory_msgs/pdu_cpptype_JointTrajectory.hpp"

namespace hako::robots::pdu::adapter::trajectory_msgs
{
    class JointTrajectoryPduAdapter
    {
    public:
        JointTrajectoryPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool recv(hako::robots::actuator::JointTrajectoryTarget& out)
        {
            HakoCpp_JointTrajectory pdu {};
            if (endpoint_.recv(pdu) != HAKO_PDU_ERR_OK) {
                return false;
            }
            out = hako::robots::pdu::converter::trajectory_msgs::
                ToJointTrajectoryTarget(pdu);
            return true;
        }

        bool recv(HakoCpp_JointTrajectory& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_JointTrajectory,
            hako::pdu::msgs::trajectory_msgs::JointTrajectory> endpoint_;
    };
}
