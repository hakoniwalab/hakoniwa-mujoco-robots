#pragma once

#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "hakoniwa/pdu_bound_rigid_body.hpp"

namespace hako::robots::pdu::converter::geometry_msgs
{
    inline HakoCpp_Twist ToHakoPdu(const hakoniwa::PduRigidBodyPose& pose)
    {
        HakoCpp_Twist out {};
        out.linear.x = pose.position.x;
        out.linear.y = pose.position.y;
        out.linear.z = pose.position.z;
        out.angular.x = pose.euler.x;
        out.angular.y = pose.euler.y;
        out.angular.z = pose.euler.z;
        return out;
    }

    inline HakoCpp_Twist ToHakoPdu(const hakoniwa::PduRigidBodyVelocity& velocity)
    {
        HakoCpp_Twist out {};
        out.linear.x = velocity.linear.x;
        out.linear.y = velocity.linear.y;
        out.linear.z = velocity.linear.z;
        out.angular.x = velocity.angular.x;
        out.angular.y = velocity.angular.y;
        out.angular.z = velocity.angular.z;
        return out;
    }

    inline hakoniwa::PduRigidBodyPose ToRigidBodyPose(const HakoCpp_Twist& pdu)
    {
        hakoniwa::PduRigidBodyPose out {};
        out.position.x = pdu.linear.x;
        out.position.y = pdu.linear.y;
        out.position.z = pdu.linear.z;
        out.euler.x = pdu.angular.x;
        out.euler.y = pdu.angular.y;
        out.euler.z = pdu.angular.z;
        return out;
    }

    inline hakoniwa::PduRigidBodyForce ToRigidBodyForce(const HakoCpp_Twist& pdu)
    {
        hakoniwa::PduRigidBodyForce out {};
        out.force.x = pdu.linear.x;
        out.force.y = pdu.linear.y;
        out.force.z = pdu.linear.z;
        return out;
    }
}
