#pragma once

#include "sensors/camera/camera_sensor.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>

namespace hako::examples::sensors::color_camera
{
    struct AppState
    {
        std::atomic_bool running {true};
        std::atomic_bool pending_shot {false};
        std::atomic_bool print_help {false};
        std::atomic_int move_forward {0};
        std::atomic_int move_left {0};
    };

    void PrintHelp();
    void PrintUsage(
        const char* program,
        const char* default_model_path,
        const char* default_config_path,
        const char* default_output_path);
    void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void TerminalCommandLoop(AppState& state);
    void MoveCamera(
        mjModel* model,
        mjData* data,
        int qpos_addr,
        int forward_steps,
        int left_steps,
        double move_step);
    void PrintImageSamples(
        const hako::robots::sensor::camera::ImageFrame& frame,
        const char* camera_name);
}
