#include "viewer/mujoco_viewer.hpp"

#define GL_SILENCE_DEPRECATION

#include <GLFW/glfw3.h>

#include <iostream>
#include <mutex>
#include <stdexcept>

MujocoRenderRuntime::MujocoRenderRuntime(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex)
    : MujocoRenderRuntime(
          model,
          data,
          running_flag,
          mutex,
          MujocoRenderWindowMode::Visible)
{
}

MujocoRenderRuntime::MujocoRenderRuntime(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex,
    MujocoRenderWindowMode window_mode)
    : model_(model),
      data_(data),
      running_flag_(&running_flag),
      mutex_(mutex),
      window_mode_(window_mode)
{
    Initialize();
}

MujocoRenderRuntime::MujocoRenderRuntime(
    mjModel* model,
    mjData* data,
    std::atomic_bool& running_flag,
    std::mutex& mutex)
    : MujocoRenderRuntime(
          model,
          data,
          running_flag,
          mutex,
          MujocoRenderWindowMode::Visible)
{
}

MujocoRenderRuntime::MujocoRenderRuntime(
    mjModel* model,
    mjData* data,
    std::atomic_bool& running_flag,
    std::mutex& mutex,
    MujocoRenderWindowMode window_mode)
    : model_(model),
      data_(data),
      atomic_running_flag_(&running_flag),
      mutex_(mutex),
      window_mode_(window_mode)
{
    Initialize();
}

void MujocoRenderRuntime::Initialize()
{
    if (model_ == nullptr || data_ == nullptr) {
        throw std::invalid_argument("MujocoRenderRuntime requires non-null mjModel and mjData");
    }

    if (!glfwInit()) {
        const char* description = nullptr;
        const int error_code = glfwGetError(&description);
        std::string message = "[ERROR] GLFW initialization failed";
        if (error_code != GLFW_NO_ERROR) {
            message += " code=" + std::to_string(error_code);
            message += " msg=" + std::string(description != nullptr ? description : "<null>");
        }
        throw std::runtime_error(message);
    }

    glfwWindowHint(GLFW_VISIBLE, HasVisibleWindow() ? GLFW_TRUE : GLFW_FALSE);
    const int window_width = HasVisibleWindow() ? 800 : 1;
    const int window_height = HasVisibleWindow() ? 600 : 1;
    window_ = glfwCreateWindow(
        window_width,
        window_height,
        HasVisibleWindow() ? "MuJoCo Simulation Viewer" : "MuJoCo Render Runtime",
        nullptr,
        nullptr);
    if (window_ == nullptr) {
        glfwTerminate();
        throw std::runtime_error("[ERROR] GLFW window creation failed");
    }

    glfwSetWindowUserPointer(window_, this);
    MakeContextCurrent();
    glfwSwapInterval(1);
    if (HasVisibleWindow()) {
        glfwSetKeyCallback(window_, KeyboardCallback);
        glfwSetMouseButtonCallback(window_, MouseButtonCallback);
        glfwSetCursorPosCallback(window_, MouseMoveCallback);
        glfwSetScrollCallback(window_, ScrollCallback);
    }

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
}

MujocoRenderRuntime::~MujocoRenderRuntime()
{
    if (window_ != nullptr) {
        MakeContextCurrent();
        mjr_freeContext(&context_);
        mjv_freeScene(&scene_);
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
    }
}

void MujocoRenderRuntime::SetOverlayCallback(ViewerOverlayCallback overlay)
{
    overlay_ = std::move(overlay);
}

void MujocoRenderRuntime::SetKeyCallback(ViewerKeyCallback key_callback)
{
    key_callback_ = std::move(key_callback);
}

void MujocoRenderRuntime::Run()
{
    if (!HasVisibleWindow()) {
        return;
    }

    while (IsRunning() && !glfwWindowShouldClose(window_)) {
        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(window_, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        {
            std::lock_guard<std::mutex> lock(mutex_);

            mjv_updateScene(
                model_,
                data_,
                &option_,
                nullptr,
                &camera_,
                mjCAT_ALL,
                &scene_);

            if (overlay_) {
                overlay_(scene_);
            }

            mjr_setBuffer(mjFB_WINDOW, &context_);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            mjr_render(viewport, &scene_, &context_);
        }

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}

bool MujocoRenderRuntime::IsRunning() const
{
    if (atomic_running_flag_ != nullptr) {
        return atomic_running_flag_->load();
    }
    return running_flag_ != nullptr && *running_flag_;
}

void MujocoRenderRuntime::MakeContextCurrent()
{
    glfwMakeContextCurrent(window_);
}

bool MujocoRenderRuntime::HasVisibleWindow() const
{
    return window_mode_ == MujocoRenderWindowMode::Visible;
}

void MujocoRenderRuntime::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    auto* runtime = static_cast<MujocoRenderRuntime*>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr) {
        runtime->HandleMouseButton(button, action, mods);
    }
}

void MujocoRenderRuntime::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos)
{
    auto* runtime = static_cast<MujocoRenderRuntime*>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr) {
        runtime->HandleMouseMove(xpos, ypos);
    }
}

void MujocoRenderRuntime::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)xoffset;
    auto* runtime = static_cast<MujocoRenderRuntime*>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr) {
        runtime->HandleScroll(yoffset);
    }
}

void MujocoRenderRuntime::KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    auto* runtime = static_cast<MujocoRenderRuntime*>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr) {
        runtime->HandleKeyboard(key, action, mods);
    }
}

void MujocoRenderRuntime::HandleMouseButton(int button, int action, int mods)
{
    mouse_button_left_ = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
    mouse_button_right_ = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
    mouse_shift_down_ = (mods & GLFW_MOD_SHIFT) != 0;
    glfwGetCursorPos(window_, &last_x_, &last_y_);
}

void MujocoRenderRuntime::HandleMouseMove(double xpos, double ypos)
{
    if (!mouse_button_left_ && !mouse_button_right_) {
        return;
    }

    const double dx = xpos - last_x_;
    const double dy = ypos - last_y_;
    last_x_ = xpos;
    last_y_ = ypos;

    int mode = mjMOUSE_MOVE_V;
    if (mouse_button_left_) {
        mode = mjMOUSE_ROTATE_H;
    } else if (mouse_shift_down_ && mouse_button_right_) {
        mode = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(model_, mode, dx / 200.0, dy / 200.0, &scene_, &camera_);
}

void MujocoRenderRuntime::HandleScroll(double yoffset)
{
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0.0, 0.05 * yoffset, &scene_, &camera_);
}

void MujocoRenderRuntime::HandleKeyboard(int key, int action, int mods)
{
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
    if (key_callback_) {
        key_callback_(key, action, mods);
    }
}

void viewer_thread(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex)
{
    viewer_thread_with_overlay(
        model,
        data,
        running_flag,
        mutex,
        nullptr);
}

void viewer_thread_with_overlay(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex,
    ViewerOverlayCallback overlay)
{
    try {
        MujocoRenderRuntime runtime(model, data, running_flag, mutex);
        runtime.SetOverlayCallback(std::move(overlay));
        runtime.Run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
