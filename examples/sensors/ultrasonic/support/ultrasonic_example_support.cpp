#include "examples/sensors/ultrasonic/support/ultrasonic_example_support.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace hako::examples::sensors::ultrasonic
{
std::string StatusToString(
    hako::robots::sensor::ultrasonic::UltrasonicStatus status)
{
    using hako::robots::sensor::ultrasonic::UltrasonicStatus;

    switch (status) {
    case UltrasonicStatus::OK:
        return "OK";
    case UltrasonicStatus::NO_HIT:
        return "NO_HIT";
    case UltrasonicStatus::BELOW_MIN_RANGE:
        return "BELOW_MIN_RANGE";
    case UltrasonicStatus::INVALID:
        return "INVALID";
    default:
        return "UNKNOWN";
    }
}

int FindSiteId(const mjModel* model, const std::string& site_name)
{
    const int site_id = mj_name2id(model, mjOBJ_SITE, site_name.c_str());
    if (site_id < 0) {
        throw std::runtime_error("site was not found: " + site_name);
    }

    return site_id;
}

hako::robots::sensor::debug::RaycastDebugLine MakeUltrasonicCenterRayDebugLine(
    const mjData* data,
    int site_id,
    const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame)
{
    const mjtNum* site_pos = &data->site_xpos[3 * site_id];
    const mjtNum* site_mat = &data->site_xmat[9 * site_id];

    mjtNum local_forward[3] = {1.0, 0.0, 0.0};
    mjtNum world_forward[3] = {0.0, 0.0, 0.0};

    mju_mulMatVec3(world_forward, site_mat, local_forward);
    mju_normalize3(world_forward);

    hako::robots::sensor::debug::RaycastDebugLine line {};

    line.from = {
        static_cast<double>(site_pos[0]),
        static_cast<double>(site_pos[1]),
        static_cast<double>(site_pos[2])
    };

    line.to = {
        static_cast<double>(site_pos[0] + world_forward[0] * frame.range),
        static_cast<double>(site_pos[1] + world_forward[1] * frame.range),
        static_cast<double>(site_pos[2] + world_forward[2] * frame.range)
    };

    using hako::robots::sensor::ultrasonic::UltrasonicStatus;

    line.hit =
        frame.status == UltrasonicStatus::OK ||
        frame.status == UltrasonicStatus::BELOW_MIN_RANGE;

    line.geom_id = -1;
    line.label = "ultrasonic_center_ray";

    return line;
}

void PrintHelp()
{
    std::cout << R"(

Controls:
  i : move forward  (+X)
  k : move backward (-X)
  j : move left     (+Y)
  l : move right    (-Y)
  s : sense and print ultrasonic range
  h : help
  q : quit

Viewer:
  - After pressing 's', the measured center ray is drawn in the viewer.
  - Green line: hit
  - Red line: no-hit

Note:
  On macOS, the MuJoCo viewer runs on the main thread.
  Console input runs on a worker thread.

)" << std::endl;
}

void TerminalCommandLoop(AppState& state)
{
    while (state.running.load()) {
        std::cout << "> " << std::flush;

        char key = '\0';
        std::cin >> key;

        if (!std::cin) {
            return;
        }

        if (key == 'q') {
            state.running.store(false);
            return;
        }
        if (key == 's') {
            state.pending_measure.store(true);
            continue;
        }
        if (key == 'h') {
            state.print_help.store(true);
            continue;
        }
        if (key == 'i') {
            state.move_forward.fetch_add(1);
            continue;
        }
        if (key == 'k') {
            state.move_forward.fetch_sub(1);
            continue;
        }
        if (key == 'j') {
            state.move_left.fetch_add(1);
            continue;
        }
        if (key == 'l') {
            state.move_left.fetch_sub(1);
            continue;
        }

        std::cout << "unknown command: " << key << std::endl;
        state.print_help.store(true);
    }
}

void PrintFrame(const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame)
{
    std::cout
        << "range="
        << std::fixed << std::setprecision(3)
        << frame.range
        << " m"
        << ", status="
        << StatusToString(frame.status)
        << ", variance="
        << std::scientific << frame.variance
        << std::defaultfloat
        << std::endl;
}
}
