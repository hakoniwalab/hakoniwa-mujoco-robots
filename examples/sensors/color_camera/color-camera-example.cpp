#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/color_camera/support/color_camera_example_support.hpp"
#include "physics/physics_impl.hpp"
#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/image_frame_writer.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <mujoco/mujoco.h>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
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

    hako::robots::sensor::camera::CameraProfileConfig profile {};
    if (!hako::robots::sensor::camera::LoadCameraProfileConfigFromJson(config_path, profile)) {
        std::cerr << "Failed to load camera config: " << config_path << std::endl;
        return 1;
    }
    const auto& config = profile.spec;
    const std::string camera_name = profile.mjcf_binding.camera_name.empty()
        ? kCameraName
        : profile.mjcf_binding.camera_name;
    const std::string sensor_joint_name = profile.mjcf_binding.freejoint_name.empty()
        ? kSensorJointName
        : profile.mjcf_binding.freejoint_name;
    if (config.image.format != "R8G8B8") {
        std::cerr << "This example expects R8G8B8 format, got: "
                  << config.image.format << std::endl;
        return 1;
    }

    auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    mjModel* model = world->getModel();
    mjData* data = world->getData();

    hako::examples::sensors::color_camera::AppState state {};
    std::unique_ptr<hako::examples::sensors::color_camera::CameraMotionController> camera_motion;
    try {
        camera_motion = std::make_unique<hako::examples::sensors::color_camera::CameraMotionController>(
            model,
            data,
            sensor_joint_name.c_str(),
            kMoveStep);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    std::atomic_bool viewer_running {true};
    std::mutex mujoco_mutex;

    MujocoRenderRuntime render_runtime(
        model,
        data,
        viewer_running,
        mujoco_mutex,
        MujocoRenderWindowMode::Visible);
    render_runtime.SetKeyCallback(
        [&](int key, int action, int mods) {
            (void)mods;
            hako::examples::sensors::color_camera::HandleViewerKey(
                state,
                *camera_motion,
                key,
                action);
            if (!state.running.load()) {
                viewer_running.store(false);
            }
        });

    auto sensor_renderer = render_runtime.CreateCameraRenderer(world);
    auto camera_sensor = std::make_unique<hako::robots::sensor::camera::CameraSensor>(
        sensor_renderer,
        camera_name);
    if (!camera_sensor->LoadConfig(config)) {
        std::cerr << "Failed to validate camera config: " << config_path << std::endl;
        return 1;
    }

    std::cout << "Hakoniwa Color Camera Example" << std::endl;
    std::cout << "model : " << model_path << std::endl;
    std::cout << "config: " << config_path << std::endl;
    std::cout << "camera: " << camera_name << std::endl;
    std::cout << "output: " << output_path << std::endl;
    camera_motion->PrintPosition("camera_pos");
    hako::examples::sensors::color_camera::PrintHelp();

    std::thread terminal_thread(
        hako::examples::sensors::color_camera::TerminalCommandLoop,
        std::ref(state),
        std::ref(*camera_motion));

    render_runtime.SetOverlayCallback([&](mjvScene& scene) {
        (void)scene;
        if (!state.running.load()) {
            viewer_running.store(false);
            return;
        }

        camera_motion->Update();

        if (state.pending_shot.exchange(false)) {
            hako::robots::sensor::camera::ImageFrame frame {};
            camera_sensor->Capture(frame);
            if (frame.data.empty()) {
                std::cerr << "CameraSensor::Capture produced an empty image." << std::endl;
            } else {
                hako::examples::sensors::color_camera::PrintImageSamples(frame, camera_name.c_str());
                const int center_x = frame.width / 2;
                const int center_y = frame.height / 2;
                const auto center_rgba = camera_sensor->CaptureAsRGBA(center_x, center_y);
                hako::examples::sensors::color_camera::PrintCenterRgbaSample(
                    center_rgba,
                    center_x,
                    center_y);
                const int region_size = std::max(1, std::min(frame.width, frame.height) / 8);
                const int region_x = center_x - region_size / 2;
                const int region_y = center_y - region_size / 2;
                const auto region_rgba = camera_sensor->CaptureRegionAverageRGBA(
                    region_x,
                    region_y,
                    region_size,
                    region_size);
                hako::examples::sensors::color_camera::PrintRegionAverageRgbaSample(
                    region_rgba,
                    region_x,
                    region_y,
                    region_size,
                    region_size);
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
    });

    render_runtime.Run();

    state.running.store(false);
    viewer_running.store(false);
    if (terminal_thread.joinable()) {
        terminal_thread.detach();
    }

    camera_sensor.reset();
    sensor_renderer.reset();

    std::cout << "bye" << std::endl;
    return EXIT_SUCCESS;
}
