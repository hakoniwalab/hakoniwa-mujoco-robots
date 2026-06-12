#include "examples/sensors/color_camera/support/color_camera_example_support.hpp"

#include "examples/sensors/common/freejoint_motion.hpp"

#include <GLFW/glfw3.h>

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

CameraMotionController::CameraMotionController(
    mjModel* model,
    mjData* data,
    const char* sensor_joint_name,
    double move_step)
    : model_(model),
      data_(data),
      qpos_addr_(hako::examples::sensors::FindFreejointQposAddr(model, sensor_joint_name)),
      move_step_(move_step)
{
}

void CameraMotionController::MoveForward(int steps)
{
    pending_forward_steps_.fetch_add(steps);
}

void CameraMotionController::MoveLeft(int steps)
{
    pending_left_steps_.fetch_add(steps);
}

void CameraMotionController::Update()
{
    const int forward_steps = pending_forward_steps_.exchange(0);
    const int left_steps = pending_left_steps_.exchange(0);
    if (forward_steps == 0 && left_steps == 0) {
        return;
    }

    hako::examples::sensors::MoveFreejointPlanarSteps(
        model_,
        data_,
        qpos_addr_,
        forward_steps,
        left_steps,
        move_step_);
    hako::examples::sensors::PrintPlanarStepMove(
        "camera",
        forward_steps,
        left_steps,
        move_step_);
    PrintPosition("camera_pos");
}

void CameraMotionController::PrintPosition(const char* label) const
{
    hako::examples::sensors::PrintFreejointPosition(data_, qpos_addr_, label);
}

void HandleViewerKey(AppState& state, CameraMotionController& motion, int key, int action)
{
    if (action != GLFW_PRESS) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state.running.store(false);
    } else if (key == GLFW_KEY_S) {
        state.pending_shot.store(true);
    } else if (key == GLFW_KEY_H) {
        state.print_help.store(true);
    } else if (key == GLFW_KEY_I) {
        motion.MoveForward(1);
    } else if (key == GLFW_KEY_K) {
        motion.MoveForward(-1);
    } else if (key == GLFW_KEY_J) {
        motion.MoveLeft(1);
    } else if (key == GLFW_KEY_L) {
        motion.MoveLeft(-1);
    }
}

void TerminalCommandLoop(AppState& state, CameraMotionController& motion)
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
            motion.MoveForward(1);
            continue;
        }
        if (key == 'k') {
            motion.MoveForward(-1);
            continue;
        }
        if (key == 'j') {
            motion.MoveLeft(1);
            continue;
        }
        if (key == 'l') {
            motion.MoveLeft(-1);
            continue;
        }

        std::cout << "unknown command: " << key << std::endl;
        state.print_help.store(true);
    }
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

void PrintCenterRgbaSample(
    const hako::robots::sensor::camera::RGBAColor& color,
    int x,
    int y)
{
    std::cout << std::fixed << std::setprecision(3)
              << "center_rgba pixel=(" << x << ", " << y << ")"
              << " rgba=("
              << color.r << ", "
              << color.g << ", "
              << color.b << ", "
              << color.a << ")"
              << std::defaultfloat << std::endl;
}

void PrintRegionAverageRgbaSample(
    const hako::robots::sensor::camera::RGBAColor& color,
    int x,
    int y,
    int width,
    int height)
{
    std::cout << std::fixed << std::setprecision(3)
              << "region_average_rgba rect=("
              << x << ", " << y << ", "
              << width << ", " << height << ")"
              << " rgba=("
              << color.r << ", "
              << color.g << ", "
              << color.b << ", "
              << color.a << ")"
              << std::defaultfloat << std::endl;
}
}
