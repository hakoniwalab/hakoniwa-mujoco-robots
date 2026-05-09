#pragma once

#include <mujoco/mujoco.h>

#include <functional>
#include <mutex>

/**
 * @brief Callback invoked after mjv_updateScene() and before mjr_render().
 *
 * This can be used to add debug geoms such as raycast lines to the MuJoCo scene.
 *
 * Notes:
 * - The callback is called while the MuJoCo viewer mutex is locked.
 * - The callback should be lightweight.
 * - The callback may append geoms to scene.geoms as long as scene.ngeom < scene.maxgeom.
 */
using ViewerOverlayCallback = std::function<void(mjvScene& scene)>;

/**
 * @brief MuJoCo 3D viewer thread.
 *
 * This is the compatibility API used by existing samples.
 *
 * @param model MuJoCo model.
 * @param data MuJoCo data.
 * @param running_flag Simulation/viewer running flag.
 * @param mutex Mutex for synchronizing access to MuJoCo data.
 */
void viewer_thread(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex);

/**
 * @brief MuJoCo 3D viewer thread with overlay callback.
 *
 * The overlay callback is invoked after the normal MuJoCo scene is updated
 * and before rendering. This is intended for debug visualization such as:
 *
 * - ultrasonic sensor rays
 * - 2D LiDAR rays
 * - contact/debug markers
 * - temporary sensor diagnostics
 *
 * @param model MuJoCo model.
 * @param data MuJoCo data.
 * @param running_flag Simulation/viewer running flag.
 * @param mutex Mutex for synchronizing access to MuJoCo data.
 * @param overlay Callback used to append debug geoms to the scene.
 */
void viewer_thread_with_overlay(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex,
    ViewerOverlayCallback overlay);