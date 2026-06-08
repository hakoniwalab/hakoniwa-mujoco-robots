#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

struct GLFWwindow;

namespace hako::robots::physics
{
class IWorld;
}

namespace hako::robots::sensor::camera
{
class MujocoCameraRenderer;
}

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
using ViewerKeyCallback = std::function<void(int key, int action, int mods)>;

enum class MujocoRenderWindowMode
{
    Visible,
    Hidden
};

class MujocoRenderRuntime
{
public:
    MujocoRenderRuntime(
        mjModel* model,
        mjData* data,
        bool& running_flag,
        std::mutex& mutex);
    MujocoRenderRuntime(
        mjModel* model,
        mjData* data,
        bool& running_flag,
        std::mutex& mutex,
        MujocoRenderWindowMode window_mode);
    MujocoRenderRuntime(
        mjModel* model,
        mjData* data,
        std::atomic_bool& running_flag,
        std::mutex& mutex);
    MujocoRenderRuntime(
        mjModel* model,
        mjData* data,
        std::atomic_bool& running_flag,
        std::mutex& mutex,
        MujocoRenderWindowMode window_mode);
    ~MujocoRenderRuntime();

    MujocoRenderRuntime(const MujocoRenderRuntime&) = delete;
    MujocoRenderRuntime& operator=(const MujocoRenderRuntime&) = delete;

    void SetOverlayCallback(ViewerOverlayCallback overlay);
    void SetKeyCallback(ViewerKeyCallback key_callback);
    void Run();
    void MakeContextCurrent();
    bool HasVisibleWindow() const;

    std::shared_ptr<hako::robots::sensor::camera::MujocoCameraRenderer>
    CreateCameraRenderer(std::shared_ptr<hako::robots::physics::IWorld> world);

private:
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    void Initialize();
    void HandleMouseButton(int button, int action, int mods);
    void HandleMouseMove(double xpos, double ypos);
    void HandleScroll(double yoffset);
    void HandleKeyboard(int key, int action, int mods);
    bool IsRunning() const;

    mjModel* model_;
    mjData* data_;
    bool* running_flag_ {nullptr};
    std::atomic_bool* atomic_running_flag_ {nullptr};
    std::mutex& mutex_;
    MujocoRenderWindowMode window_mode_ {MujocoRenderWindowMode::Visible};
    ViewerOverlayCallback overlay_;
    ViewerKeyCallback key_callback_;

    mjvCamera camera_ {};
    mjvOption option_ {};
    mjvScene scene_ {};
    mjrContext context_ {};
    GLFWwindow* window_ {nullptr};
    bool mouse_button_left_ {false};
    bool mouse_button_right_ {false};
    bool mouse_shift_down_ {false};
    double last_x_ {0.0};
    double last_y_ {0.0};
};

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
