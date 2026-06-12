#pragma once

#include "sensors/camera/camera_sensor.hpp"

#include <mujoco/mujoco.h>

#include <atomic>

namespace hako::examples::sensors::color_camera
{
    struct AppState
    {
        std::atomic_bool running {true};
        std::atomic_bool pending_shot {false};
        std::atomic_bool print_help {false};
    };

    class CameraMotionController
    {
    public:
        CameraMotionController(
            mjModel* model,
            mjData* data,
            const char* sensor_joint_name,
            double move_step);

        void MoveForward(int steps);
        void MoveLeft(int steps);
        void Update();
        void PrintPosition(const char* label) const;

    private:
        mjModel* model_;
        mjData* data_;
        int qpos_addr_;
        double move_step_;
        std::atomic_int pending_forward_steps_ {0};
        std::atomic_int pending_left_steps_ {0};
    };

    void PrintHelp();
    void PrintUsage(
        const char* program,
        const char* default_model_path,
        const char* default_config_path,
        const char* default_output_path);
    void HandleViewerKey(AppState& state, CameraMotionController& motion, int key, int action);
    void TerminalCommandLoop(AppState& state, CameraMotionController& motion);
    void PrintImageSamples(
        const hako::robots::sensor::camera::ImageFrame& frame,
        const char* camera_name);
    void PrintCenterRgbaSample(
        const hako::robots::sensor::camera::RGBAColor& color,
        int x,
        int y);
    void PrintRegionAverageRgbaSample(
        const hako::robots::sensor::camera::RGBAColor& color,
        int x,
        int y,
        int width,
        int height);
}
