#pragma once

#include "sensors/debug/raycast_debug.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"

#include <mujoco/mujoco.h>

#include <atomic>
#include <string>

namespace hako::examples::sensors::ultrasonic
{
    struct AppState
    {
        std::atomic_bool running {true};
        std::atomic_bool pending_measure {true};
        std::atomic_bool print_help {false};
        std::atomic_int move_forward {0};
        std::atomic_int move_left {0};
    };

    std::string StatusToString(
        hako::robots::sensor::ultrasonic::UltrasonicStatus status);
    int FindSiteId(const mjModel* model, const std::string& site_name);
    void PrintHelp();
    void TerminalCommandLoop(AppState& state);
    void PrintFrame(const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame);
    hako::robots::sensor::debug::RaycastDebugLine MakeUltrasonicCenterRayDebugLine(
        const mjData* data,
        int site_id,
        const hako::robots::sensor::ultrasonic::UltrasonicFrame& frame);
}
