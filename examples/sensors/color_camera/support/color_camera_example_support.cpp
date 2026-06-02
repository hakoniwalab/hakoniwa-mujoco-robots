#include "examples/sensors/color_camera/support/color_camera_example_support.hpp"

#include "examples/sensors/common/freejoint_motion.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace hako::examples::sensors::color_camera
{
namespace
{
struct Rgb
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

Rgb sample_pixel(const std::vector<uint8_t>& rgb, int width, int x, int y)
{
    const size_t index = static_cast<size_t>((y * width + x) * 3);
    return Rgb {rgb.at(index + 0), rgb.at(index + 1), rgb.at(index + 2)};
}

void print_sample(
    const std::string& label,
    const std::vector<uint8_t>& rgb,
    int width,
    int x,
    int y)
{
    const Rgb sample = sample_pixel(rgb, width, x, y);
    std::cout
        << std::left << std::setw(8) << label
        << " pixel=(" << std::right << std::setw(3) << x << ", "
        << std::setw(3) << y << ")"
        << " rgb=("
        << std::setw(3) << static_cast<int>(sample.r) << ", "
        << std::setw(3) << static_cast<int>(sample.g) << ", "
        << std::setw(3) << static_cast<int>(sample.b) << ")"
        << std::endl;
}
}

void PrintHelp()
{
    std::cout << R"(
Controls:
  i      : move camera forward  (+X)
  k      : move camera backward (-X)
  j      : move camera left     (+Y)
  l      : move camera right    (-Y)
  s      : capture color_camera and write PNG
  h      : show help
  q / Esc: quit

Viewer:
  Use the mouse to rotate / zoom the MuJoCo viewer.
  Press 's' in either the viewer window or this terminal to save a sensor shot.
)" << std::endl;
}

void PrintUsage(
    const char* program,
    const char* default_model_path,
    const char* default_config_path,
    const char* default_output_path)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [model.xml] [camera-config.json] [output.png]\n\n"
        << "Default:\n"
        << "  model : " << default_model_path << "\n"
        << "  config: " << default_config_path << "\n"
        << "  output: " << default_output_path << "\n";
}

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;
    if (action != GLFW_PRESS) {
        return;
    }

    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state == nullptr) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state->running.store(false);
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_S) {
        state->pending_shot.store(true);
    } else if (key == GLFW_KEY_H) {
        state->print_help.store(true);
    } else if (key == GLFW_KEY_I) {
        state->move_forward.fetch_add(1);
    } else if (key == GLFW_KEY_K) {
        state->move_forward.fetch_sub(1);
    } else if (key == GLFW_KEY_J) {
        state->move_left.fetch_add(1);
    } else if (key == GLFW_KEY_L) {
        state->move_left.fetch_sub(1);
    }
}

void TerminalCommandLoop(AppState& state)
{
    while (state.running.load()) {
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
            state.pending_shot.store(true);
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

void MoveCamera(
    mjModel* model,
    mjData* data,
    int qpos_addr,
    int forward_steps,
    int left_steps,
    double move_step)
{
    if (forward_steps == 0 && left_steps == 0) {
        return;
    }

    hako::examples::sensors::MoveFreejointPlanarSteps(
        model,
        data,
        qpos_addr,
        forward_steps,
        left_steps,
        move_step);
    hako::examples::sensors::PrintPlanarStepMove(
        "camera",
        forward_steps,
        left_steps,
        move_step);
    hako::examples::sensors::PrintFreejointPosition(data, qpos_addr, "camera_pos");
}

void PrintImageSamples(
    const hako::robots::sensor::camera::ImageFrame& frame,
    const char* camera_name)
{
    const int y = frame.height / 2;
    std::cout << "\nCaptured " << camera_name << " "
              << frame.width << "x" << frame.height
              << " format=" << frame.format << std::endl;
    print_sample("left", frame.data, frame.width, frame.width / 6, y);
    print_sample("center", frame.data, frame.width, frame.width / 2, y);
    print_sample("right", frame.data, frame.width, (frame.width * 5) / 6, y);
}
}
