// src/sensors/debug/raycast_debug.cpp

#include "sensors/debug/raycast_debug.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>

namespace hako::robots::sensor::debug {
namespace {

void copy_rgba(float dst[4], const float src[4])
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

bool is_valid_width(double width)
{
    return width > 0.0;
}

} // namespace

bool AddRaycastDebugLine(
    mjvScene& scene,
    const RaycastDebugLine& line,
    double width)
{
    if (!is_valid_width(width)) {
        return false;
    }

    if (scene.ngeom >= scene.maxgeom) {
        return false;
    }

    mjvGeom& geom = scene.geoms[scene.ngeom];

    const mjtNum from[3] = {
        static_cast<mjtNum>(line.from[0]),
        static_cast<mjtNum>(line.from[1]),
        static_cast<mjtNum>(line.from[2])
    };

    const mjtNum to[3] = {
        static_cast<mjtNum>(line.to[0]),
        static_cast<mjtNum>(line.to[1]),
        static_cast<mjtNum>(line.to[2])
    };

    /*
     * mjv_connector initializes a visual connector geom between two points.
     * mjGEOM_LINE is lightweight and works well for raycast debugging.
     */
    mjv_connector(
        &geom,
        mjGEOM_LINE,
        static_cast<mjtNum>(width),
        from,
        to);

    /*
     * Color convention:
     *   hit    : green
     *   no-hit : red
     *
     * Keep alpha high enough to be visible over normal model geoms.
     */
    static constexpr float kHitRgba[4] = {
        0.0F, 1.0F, 0.0F, 1.0F
    };

    static constexpr float kNoHitRgba[4] = {
        1.0F, 0.0F, 0.0F, 1.0F
    };

    copy_rgba(geom.rgba, line.hit ? kHitRgba : kNoHitRgba);

    ++scene.ngeom;
    return true;
}

int AddRaycastDebugLines(
    mjvScene& scene,
    const RaycastDebugLines& lines,
    double width)
{
    int added = 0;

    for (const auto& line : lines) {
        if (AddRaycastDebugLine(scene, line, width)) {
            ++added;
        } else {
            break;
        }
    }

    return added;
}

} // namespace hako::robots::sensor::debug