#include "hakoniwa/pdu/adapter/sensor_msgs/image.hpp"
#include "examples/sensors/color_camera/support/color_camera_example_support.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "physics/physics_impl.hpp"
#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "viewer/mujoco_viewer.hpp"

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
constexpr const char* kDefaultCameraConfigPath =
    "config/sensors/color_camera/simple-color-camera.json";
constexpr const char* kDefaultPduDefPath =
    "config/camera-pdudef-compact.json";
constexpr const char* kDefaultEndpointConfigPath =
    "config/endpoint/camera_endpoint.json";
constexpr const char* kDefaultEndpointName = "camera_endpoint";
constexpr const char* kDefaultAssetName = "CameraAsset";
constexpr const char* kDefaultCameraName = "color_camera";
constexpr const char* kDefaultSensorJointName = "color_sensor_freejoint";
constexpr const char* kDefaultImagePduName = "camera_image";
constexpr double kMoveStep = 0.05;

std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::string model_path;
std::string camera_config_path;
std::string pdu_def_path;
std::string endpoint_config_path;
std::string asset_name;
std::string endpoint_name;
std::atomic_bool running {true};
std::atomic_bool viewer_running {true};
std::atomic_bool endpoint_ready {false};
std::mutex mujoco_mutex;
std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter> image_adapter;
std::unique_ptr<hako::robots::sensor::camera::CameraSensor> camera_sensor;
std::unique_ptr<hako::examples::sensors::color_camera::CameraMotionController> camera_motion;
hako::examples::sensors::color_camera::AppState app_state {};

std::string EnvOrDefault(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0') {
        return value;
    }
    return fallback;
}

void PrintUsage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [model.xml] [camera-config.json] [pdu-def.json] [endpoint.json]\n\n"
        << "Defaults:\n"
        << "  model.xml          " << kDefaultModelPath << "\n"
        << "  camera-config.json " << kDefaultCameraConfigPath << "\n"
        << "  pdu-def.json       " << kDefaultPduDefPath << "\n"
        << "  endpoint.json      " << kDefaultEndpointConfigPath << "\n\n"
        << "Environment:\n"
        << "  HAKO_CAMERA_ASSET_NAME       asset name, default " << kDefaultAssetName << "\n"
        << "  HAKO_CAMERA_ENDPOINT_NAME    endpoint name, default " << kDefaultEndpointName << "\n";
}

void PrintPublisherHelp()
{
    std::cout << R"(
Controls:
  i      : move camera forward  (+X)
  k      : move camera backward (-X)
  j      : move camera left     (+Y)
  l      : move camera right    (-Y)
  h      : show help
  q / Esc: quit publisher

Viewer:
  Use the mouse to rotate / zoom the MuJoCo viewer.
  The Python OpenCV window shows the camera image published over Hakoniwa PDU.
)" << std::endl;
}

static int OnInitialize(hako_asset_context_t* context)
{
    (void)context;
    if (endpoint == nullptr) {
        std::cerr << "[ERROR] Endpoint is not initialized." << std::endl;
        return -1;
    }
    if (endpoint->post_start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to complete endpoint post_start." << std::endl;
        return -1;
    }
    endpoint_ready.store(true);
    return 0;
}

static int OnReset(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

int IsForceStop()
{
    return running.load() && app_state.running.load() ? 0 : 1;
}

static int OnManualTimingControl(hako_asset_context_t* context)
{
    (void)context;
    const double sim_timestep = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(sim_timestep * 1.0e6);

    std::cout << "[INFO] Camera Hakoniwa asset started." << std::endl;
    PrintPublisherHelp();

    while (running.load() && app_state.running.load()) {
        hako_asset_usleep(delta_time_usec);
    }

    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage(argv[0]);
        return 0;
    }

    model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    camera_config_path = argc > 2 ? argv[2] : kDefaultCameraConfigPath;
    pdu_def_path = argc > 3 ? argv[3] : kDefaultPduDefPath;
    endpoint_config_path = argc > 4 ? argv[4] : kDefaultEndpointConfigPath;
    asset_name = EnvOrDefault("HAKO_CAMERA_ASSET_NAME", kDefaultAssetName);
    endpoint_name = EnvOrDefault("HAKO_CAMERA_ENDPOINT_NAME", kDefaultEndpointName);

    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
        return 1;
    }
    hako::robots::sensor::camera::CameraProfileConfig profile {};
    if (!hako::robots::sensor::camera::LoadCameraProfileConfigFromJson(camera_config_path, profile)) {
        std::cerr << "[ERROR] Failed to load camera config: "
                  << camera_config_path << std::endl;
        return 1;
    }
    const std::string sensor_joint_name = profile.mjcf_binding.freejoint_name.empty()
        ? kDefaultSensorJointName
        : profile.mjcf_binding.freejoint_name;
    const std::string camera_name = profile.mjcf_binding.camera_name.empty()
        ? kDefaultCameraName
        : profile.mjcf_binding.camera_name;
    const std::string image_pdu_name = profile.pdu_config.pdu_name.empty()
        ? kDefaultImagePduName
        : profile.pdu_config.pdu_name;
    if (profile.spec.image.format != "R8G8B8") {
        std::cerr << "[ERROR] This example expects R8G8B8 format, got: "
                  << profile.spec.image.format << std::endl;
        return 1;
    }
    try {
        camera_motion = std::make_unique<hako::examples::sensors::color_camera::CameraMotionController>(
            world->getModel(),
            world->getData(),
            sensor_joint_name.c_str(),
            kMoveStep);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    hako_asset_callbacks_t callbacks {};
    callbacks.on_initialize = OnInitialize;
    callbacks.on_simulation_step = nullptr;
    callbacks.on_manual_timing_control = OnManualTimingControl;
    callbacks.on_reset = OnReset;

    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(world->getModel()->opt.timestep * 1.0e6);

    hako_conductor_start(delta_time_usec, 100000);

    const int register_result = hako_asset_register(
        asset_name.c_str(),
        pdu_def_path.c_str(),
        &callbacks,
        delta_time_usec,
        HAKO_ASSET_MODEL_PLANT);
    if (register_result != 0) {
        std::cerr << "[ERROR] hako_asset_register() returns "
                  << register_result << std::endl;
        hako_conductor_stop();
        return 1;
    }

    endpoint = std::make_unique<hakoniwa::pdu::Endpoint>(
        endpoint_name,
        HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    if (endpoint->open(endpoint_config_path) != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to open endpoint config: "
                  << endpoint_config_path << std::endl;
        hako_conductor_stop();
        return 1;
    }
    if (endpoint->start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to start endpoint." << std::endl;
        endpoint->close();
        endpoint.reset();
        hako_conductor_stop();
        return 1;
    }
    const hakoniwa::pdu::PduKey image_key {asset_name, image_pdu_name};
    image_adapter = std::make_unique<hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter>(
        *endpoint,
        image_key);
    std::cout << "[INFO] Starting camera asset with:" << std::endl;
    std::cout << "  model    : " << model_path << std::endl;
    std::cout << "  camera   : " << camera_config_path << std::endl;
    std::cout << "  pdu_def  : " << pdu_def_path << std::endl;
    std::cout << "  endpoint : " << endpoint_config_path << std::endl;

    MujocoRenderRuntime render_runtime(
        world->getModel(),
        world->getData(),
        viewer_running,
        mujoco_mutex,
        MujocoRenderWindowMode::Visible);
    auto sensor_renderer = render_runtime.CreateCameraRenderer(world);
    camera_sensor = std::make_unique<hako::robots::sensor::camera::CameraSensor>(
        sensor_renderer,
        camera_name);
    if (!camera_sensor->LoadConfig(profile.spec)) {
        std::cerr << "[ERROR] Failed to validate camera config: "
                  << camera_config_path << std::endl;
        return 1;
    }
    const double sim_timestep = world->getModel()->opt.timestep;
    hako::robots::sensor::camera::ImageFrame frame {};
    std::cout << "[INFO] asset=" << asset_name
              << " camera=" << camera_name
              << " freejoint=" << sensor_joint_name
              << " pdu=" << image_pdu_name
              << " sensor_rate_hz=" << profile.spec.update_rate
              << " pdu_config_rate_hz=" << profile.pdu_config.update_rate_hz
              << std::endl;
    render_runtime.SetKeyCallback(
        [&](int key, int action, int mods) {
            (void)mods;
            hako::examples::sensors::color_camera::HandleViewerKey(
                app_state,
                *camera_motion,
                key,
                action);
            if (!app_state.running.load()) {
                running.store(false);
                viewer_running.store(false);
            }
        });
    render_runtime.SetPreRenderCallback([&]() {
        if (!app_state.running.load()) {
            running.store(false);
            viewer_running.store(false);
            return;
        }
        camera_motion->Update();
        if (endpoint_ready.load()) {
            world->advanceTimeStep();
            if (camera_sensor->ShouldUpdate(sim_timestep)) {
                camera_sensor->Capture(frame);
                if (!frame.data.empty() && image_adapter != nullptr &&
                    !image_adapter->send(frame))
                {
                    std::cerr << "[WARN] Failed to send camera image PDU." << std::endl;
                }
            }
        }
        if (app_state.pending_shot.exchange(false)) {
            std::cout << "[INFO] This publisher streams camera images continuously; "
                      << "PNG capture is not used here." << std::endl;
        }
        if (app_state.print_help.exchange(false)) {
            PrintPublisherHelp();
        }
    });

    std::thread terminal_thread(
        hako::examples::sensors::color_camera::TerminalCommandLoop,
        std::ref(app_state),
        std::ref(*camera_motion));
    int start_result = 0;
    std::atomic_bool asset_thread_finished {false};
    std::thread asset_thread([&]() {
        start_result = hako_asset_start_no_wait(IsForceStop);
        asset_thread_finished.store(true);
        running.store(false);
        viewer_running.store(false);
    });

    render_runtime.Run();
    const bool asset_finished_before_viewer_return = asset_thread_finished.load();
    running.store(false);
    viewer_running.store(false);
    app_state.running.store(false);
    if (terminal_thread.joinable()) {
        terminal_thread.detach();
    }
    if (asset_thread.joinable()) {
        asset_thread.join();
    }
    if (endpoint != nullptr) {
        (void)endpoint->stop();
        endpoint->close();
        endpoint.reset();
    }
    camera_sensor.reset();
    image_adapter.reset();
    hako_conductor_stop();
    if (start_result != 0 && asset_finished_before_viewer_return) {
        std::cerr << "[ERROR] hako_asset_start() returns "
                  << start_result << std::endl;
        return 1;
    }
    return 0;
}
