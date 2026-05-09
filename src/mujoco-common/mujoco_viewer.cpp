#include "mujoco_viewer.hpp"

#define GL_SILENCE_DEPRECATION

#include <GLFW/glfw3.h>
#include <iostream>
#include <mutex>

namespace {

mjvCamera g_camera;
mjvOption g_option;
mjvScene g_scene;
mjrContext g_context;
GLFWwindow* g_window = nullptr;
bool g_mouse_button_left = false;
bool g_mouse_button_right = false;
bool g_mouse_shift_down = false;
double g_last_x = 0.0;
double g_last_y = 0.0;
mjModel* g_model = nullptr;

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    g_mouse_button_left = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
    g_mouse_button_right = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
    g_mouse_shift_down = (mods & GLFW_MOD_SHIFT) != 0;
    glfwGetCursorPos(window, &g_last_x, &g_last_y);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    (void)window;
    if (!g_mouse_button_left && !g_mouse_button_right) {
        return;
    }

    const double dx = xpos - g_last_x;
    const double dy = ypos - g_last_y;
    g_last_x = xpos;
    g_last_y = ypos;

    int mode = mjMOUSE_MOVE_V;
    if (g_mouse_button_left) {
        mode = mjMOUSE_ROTATE_H;
    } else if (g_mouse_shift_down && g_mouse_button_right) {
        mode = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(g_model, mode, dx / 200.0, dy / 200.0, &g_scene, &g_camera);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    (void)window;
    (void)xoffset;
    mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, 0.05 * yoffset, &g_scene, &g_camera);
}

void keyboard_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

}  // namespace

void viewer_thread(mjModel* model, mjData* data, bool& running_flag, std::mutex& mutex) {

    if (!glfwInit()) {
        const char* description = nullptr;
        const int error_code = glfwGetError(&description);
        std::cerr << "[ERROR] GLFW initialization failed";
        if (error_code != GLFW_NO_ERROR) {
            std::cerr << " code=" << error_code
                      << " msg=" << (description != nullptr ? description : "<null>");
        }
        std::cerr << std::endl;
        return;
    }

    g_model = model;
    g_window = glfwCreateWindow(800, 600, "MuJoCo Simulation Viewer", nullptr, nullptr);
    if (g_window == nullptr) {
        std::cerr << "[ERROR] GLFW window creation failed" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(g_window, keyboard_callback);
    glfwSetMouseButtonCallback(g_window, mouse_button_callback);
    glfwSetCursorPosCallback(g_window, mouse_move_callback);
    glfwSetScrollCallback(g_window, scroll_callback);

    mjv_defaultCamera(&g_camera);
    mjv_defaultOption(&g_option);
    mjv_makeScene(model, &g_scene, 2000);
    mjr_makeContext(model, &g_context, mjFONTSCALE_150);

    while (running_flag && !glfwWindowShouldClose(g_window)) {
        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(g_window, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        {
            std::lock_guard<std::mutex> lock(mutex);
            mjv_updateScene(model, data, &g_option, nullptr, &g_camera, mjCAT_ALL, &g_scene);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            mjr_render(viewport, &g_scene, &g_context);
        }

        glfwSwapBuffers(g_window);
        glfwPollEvents();
    }

    mjr_freeContext(&g_context);
    mjv_freeScene(&g_scene);
    glfwDestroyWindow(g_window);
    g_window = nullptr;
    glfwTerminate();
}
