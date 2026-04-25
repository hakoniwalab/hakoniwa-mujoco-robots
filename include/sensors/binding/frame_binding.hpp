#pragma once

#include <string>

namespace hako::robots::sensor::binding
{
    struct LinkBinding
    {
        std::string frame_id {};
        std::string parent_body {};
        std::string source_body {};
    };

    struct TransformBinding
    {
        std::string parent_frame_id {};
        std::string child_frame_id {};
        std::string source_body {};
    };
}
