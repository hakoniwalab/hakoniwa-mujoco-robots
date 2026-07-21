#pragma once

#include "actuator.hpp"
#include "trajectory_msgs/pdu_cpptype_JointTrajectory.hpp"

#include <utility>

namespace hako::robots::pdu::converter::trajectory_msgs
{
    inline double ToSeconds(const HakoCpp_Duration& duration)
    {
        return static_cast<double>(duration.sec) +
               static_cast<double>(duration.nanosec) * 1.0e-9;
    }

    inline hako::robots::actuator::JointTrajectoryTarget ToJointTrajectoryTarget(
        const HakoCpp_JointTrajectory& src)
    {
        hako::robots::actuator::JointTrajectoryTarget out {};
        out.joint_names = src.joint_names;
        out.points.reserve(src.points.size());

        for (const auto& src_point : src.points) {
            hako::robots::actuator::JointTrajectoryPointTarget point {};
            point.positions = src_point.positions;
            point.velocities = src_point.velocities;
            point.accelerations = src_point.accelerations;
            point.effort = src_point.effort;
            point.time_from_start_sec = ToSeconds(src_point.time_from_start);
            out.points.push_back(std::move(point));
        }

        return out;
    }
}
