// include/sensors/debug/raycast_debug.hpp
#pragma once

#include <array>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace hako::robots::sensor::debug {

struct RaycastDebugLine {
    std::array<double, 3> from {};
    std::array<double, 3> to {};
    bool hit {false};
    int geom_id {-1};
    std::string label {};
};

using RaycastDebugLines = std::vector<RaycastDebugLine>;

bool AddRaycastDebugLine(
    mjvScene& scene,
    const RaycastDebugLine& line,
    double width = 0.003);

int AddRaycastDebugLines(
    mjvScene& scene,
    const RaycastDebugLines& lines,
    double width = 0.003);

} // namespace hako::robots::sensor::debug