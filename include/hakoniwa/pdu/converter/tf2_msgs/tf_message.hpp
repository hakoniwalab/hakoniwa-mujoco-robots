#pragma once

#include <utility>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensors/tf/tf_publisher.hpp"
#include "tf2_msgs/pdu_cpptype_TFMessage.hpp"

namespace hako::robots::pdu::converter::tf2_msgs
{
    inline HakoCpp_TFMessage ToHakoPdu(const hako::robots::sensor::TfFrame& frame)
    {
        HakoCpp_TFMessage out {};
        out.transforms.reserve(frame.transforms.size());
        for (const auto& src : frame.transforms) {
            HakoCpp_TransformStamped dst {};
            dst.header = hako::robots::pdu::converter::ToHakoHeader(src.header);
            dst.child_frame_id = src.child_frame_id;
            dst.transform.translation.x = src.transform.position.x;
            dst.transform.translation.y = src.transform.position.y;
            dst.transform.translation.z = src.transform.position.z;
            dst.transform.rotation = hako::robots::pdu::converter::ToHakoQuaternion(src.transform.orientation);
            out.transforms.push_back(std::move(dst));
        }
        return out;
    }
}
