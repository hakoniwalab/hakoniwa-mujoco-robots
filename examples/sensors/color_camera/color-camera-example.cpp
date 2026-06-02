#include "examples/sensors/common/example_world.hpp"
#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/color_camera/support/color_camera_example_support.hpp"
#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/image_frame_writer.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultModelPath =
    "models/sensors/color_camera/color-camera-sample.xml";
constexpr const char* kDefaultConfigPath =
    "config/sensors/color_camera/simple-color-camera.json";
constexpr const char* kDefaultOutputPath =
    "./camera_color_sample.png";
constexpr const char* kCameraName = "color_camera";
constexpr const char* kSensorJointName = "color_sensor_freejoint";
constexpr double kMoveStep = 0.05;

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        hako::examples::sensors::color_camera::PrintUsage(
            argv[0],
            kDefaultModelPath,
            kDefaultConfigPath,
            kDefaultOutputPath);
        return 0;
    }

    const std::string model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    const std::string config_path = argc > 2 ? argv[2] : kDefaultConfigPath;
    const std::filesystem::path output_path = argc > 3 ? argv[3] : kDefaultOutputPath;

    hako::robots::sensor::camera::CameraConfig config {};
    if (!hako::robots::sensor::camera::LoadCameraConfigFromJson(config_path, config)) {
        std::cerr << "Failed to load camera config: " << config_path << std::endl;
        return 1;
    }
    if (config.image.format != "R8G8B8") {
        std::cerr << "This example expects R8G8B8 format, got: "
                  << config.image.format << std::endl;
        return 1;
    }

    auto world = std::make_shared<hako::examples::sensors::ExampleWorld>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    mjModel* model = world->getModel();
    mjData* data = world->getData();
    int camera_qpos_addr = -1;
    try {
        camera_qpos_addr = hako::examples::sensors::FindFreejointQposAddr(model, kSensorJointName);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed." << std::endl;
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(900, 650, "Hakoniwa Color Camera Example", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        std::cerr << "GLFW window creation failed." << std::endl;
        return 1;
    }

    hako::examples::sensors::color_camera::AppState state {};
    glfwSetWindowUserPointer(window, &state);
    glfwSetKeyCallback(window, hako::examples::sensors::color_camera::KeyCallback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera viewer_camera;
    mjvOption viewer_option;
    mjvScene viewer_scene;
    mjrContext context;
    mjv_defaultCamera(&viewer_camera);
    mjv_defaultOption(&viewer_option);
    mjv_defaultScene(&viewer_scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &viewer_scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    auto sensor_renderer = std::make_shared<hako::robots::sensor::camera::MujocoCameraRenderer>(
        world,
        false);
    auto camera_sensor = std::make_unique<hako::robots::sensor::camera::CameraSensor>(
        sensor_renderer,
        kCameraName);
    if (!camera_sensor->LoadConfig(config)) {
        mjr_freeContext(&context);
        mjv_freeScene(&viewer_scene);
        glfwDestroyWindow(window);
        glfwTerminate();
        std::cerr << "Failed to validate camera config: " << config_path << std::endl;
        return 1;
    }

    std::cout << "Hakoniwa Color Camera Example" << std::endl;
    std::cout << "model : " << model_path << std::endl;
    std::cout << "config: " << config_path << std::endl;
    std::cout << "output: " << output_path << std::endl;
    hako::examples::sensors::PrintFreejointPosition(data, camera_qpos_addr, "camera_pos");
    hako::examples::sensors::color_camera::PrintHelp();

    std::thread terminal_thread(
        hako::examples::sensors::color_camera::TerminalCommandLoop,
        std::ref(state));

    while (state.running.load() && !glfwWindowShouldClose(window)) {
        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        mjr_setBuffer(mjFB_WINDOW, &context);
        mjv_updateScene(
            model,
            data,
            &viewer_option,
            nullptr,
            &viewer_camera,
            mjCAT_ALL,
            &viewer_scene);
        mjr_render(viewport, &viewer_scene, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();

        const int forward_steps = state.move_forward.exchange(0);
        const int left_steps = state.move_left.exchange(0);
        hako::examples::sensors::color_camera::MoveCamera(
            model,
            data,
            camera_qpos_addr,
            forward_steps,
            left_steps,
            kMoveStep);

        if (state.pending_shot.exchange(false)) {
            hako::robots::sensor::camera::ImageFrame frame {};
            camera_sensor->Capture(frame);
            if (frame.data.empty()) {
                std::cerr << "CameraSensor::Capture produced an empty image." << std::endl;
            } else {
                hako::examples::sensors::color_camera::PrintImageSamples(frame, kCameraName);
                if (hako::robots::sensor::camera::WriteImageFrameToPng(frame, output_path)) {
                    std::cout << "Wrote PNG: " << output_path << "\n" << std::endl;
                } else {
                    std::cerr << "Failed to write PNG: " << output_path << std::endl;
                }
            }
        }
        if (state.print_help.exchange(false)) {
            hako::examples::sensors::color_camera::PrintHelp();
        }
    }

    state.running.store(false);
    if (terminal_thread.joinable()) {
        terminal_thread.detach();
    }

    camera_sensor.reset();
    sensor_renderer.reset();

    mjr_freeContext(&context);
    mjv_freeScene(&viewer_scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    std::cout << "bye" << std::endl;
    return 0;
}
