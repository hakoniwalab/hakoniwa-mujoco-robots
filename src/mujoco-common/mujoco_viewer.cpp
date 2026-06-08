#include "viewer/mujoco_viewer.hpp"

#define GL_SILENCE_DEPRECATION

#include <GLFW/glfw3.h>

#include <iostream>
#include <mutex>
#include <stdexcept>

WorldViewer::WorldViewer(
    mjModel* model,
    mjData* data,
    bool& running_flag,
    std::mutex& mutex)
    : model_(model),
      data_(data),
      running_flag_(&running_flag),
      mutex_(mutex)
{
    Initialize();
}

WorldViewer::WorldViewer(
    mjModel* model,
    mjData* data,
    std::atomic_bool& running_flag,
    std::mutex& mutex)
    : model_(model),
      data_(data),
      atomic_running_flag_(&running_flag),
      mutex_(mutex)
{
    Initialize();
}

void WorldViewer::Initialize()
{
    if (model_ == nullptr || data_ == nullptr) {
        throw std::invalid_argument("WorldViewer requires non-null mjModel and mjData");
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

    window_ = glfwCreateWindow(800, 600, "MuJoCo Simulation Viewer", nullptr, nullptr);
    if (window_ == nullptr) {
        glfwTerminate();
        throw std::runtime_error("[ERROR] GLFW window creation failed");
    }

    glfwSetWindowUserPointer(window_, this);
    MakeContextCurrent();
    glfwSwapInterval(1);
    glfwSetKeyCallback(window_, KeyboardCallback);
    glfwSetMouseButtonCallback(window_, MouseButtonCallback);
    glfwSetCursorPosCallback(window_, MouseMoveCallback);
    glfwSetScrollCallback(window_, ScrollCallback);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
}

WorldViewer::~WorldViewer()
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

void WorldViewer::SetOverlayCallback(ViewerOverlayCallback overlay)
{
    overlay_ = std::move(overlay);
}

void WorldViewer::SetKeyCallback(ViewerKeyCallback key_callback)
{
    key_callback_ = std::move(key_callback);
}

void WorldViewer::Run()
{
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

bool WorldViewer::IsRunning() const
{
    if (atomic_running_flag_ != nullptr) {
        return atomic_running_flag_->load();
    }
    return running_flag_ != nullptr && *running_flag_;
}

void WorldViewer::MakeContextCurrent()
{
    glfwMakeContextCurrent(window_);
}

void WorldViewer::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    auto* viewer = static_cast<WorldViewer*>(glfwGetWindowUserPointer(window));
    if (viewer != nullptr) {
        viewer->HandleMouseButton(button, action, mods);
    }
}

void WorldViewer::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos)
{
    auto* viewer = static_cast<WorldViewer*>(glfwGetWindowUserPointer(window));
    if (viewer != nullptr) {
        viewer->HandleMouseMove(xpos, ypos);
    }
}

void WorldViewer::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)xoffset;
    auto* viewer = static_cast<WorldViewer*>(glfwGetWindowUserPointer(window));
    if (viewer != nullptr) {
        viewer->HandleScroll(yoffset);
    }
}

void WorldViewer::KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    auto* viewer = static_cast<WorldViewer*>(glfwGetWindowUserPointer(window));
    if (viewer != nullptr) {
        viewer->HandleKeyboard(key, action, mods);
    }
}

void WorldViewer::HandleMouseButton(int button, int action, int mods)
{
    mouse_button_left_ = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
    mouse_button_right_ = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
    mouse_shift_down_ = (mods & GLFW_MOD_SHIFT) != 0;
    glfwGetCursorPos(window_, &last_x_, &last_y_);
}

void WorldViewer::HandleMouseMove(double xpos, double ypos)
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

void WorldViewer::HandleScroll(double yoffset)
{
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0.0, 0.05 * yoffset, &scene_, &camera_);
}

void WorldViewer::HandleKeyboard(int key, int action, int mods)
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
        WorldViewer viewer(model, data, running_flag, mutex);
        viewer.SetOverlayCallback(std::move(overlay));
        viewer.Run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
