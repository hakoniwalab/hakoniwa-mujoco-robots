#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/ultrasonic/support/ultrasonic_example_support.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "physics/physics_impl.hpp"
#include "sensors/debug/raycast_debug.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultModelPath =
    "models/sensors/ultrasonic/ultrasonic-sensor-test.xml";
constexpr const char* kDefaultSensorConfigPath =
    "config/sensors/ultrasonic/lego-spike-distance-sensor.json";
constexpr const char* kDefaultPduDefPath =
    "config/ultrasonic-pdudef-compact.json";
constexpr const char* kDefaultEndpointConfigPath =
    "config/endpoint/ultrasonic_endpoint.json";
constexpr const char* kDefaultEndpointName = "ultrasonic_endpoint";
constexpr const char* kDefaultAssetName = "UltrasonicAsset";
constexpr const char* kDefaultSensorSiteName = "front_ultrasonic_site";
constexpr const char* kDefaultExcludeBodyName = "base_footprint";
constexpr const char* kDefaultBaseJointName = "base_freejoint";
constexpr const char* kDefaultRangePduName = "range";

constexpr double kMoveStep = 0.05;
constexpr double kDebugRayWidth = 0.006;

std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::string model_path;
std::string sensor_config_path;
std::string pdu_def_path;
std::string endpoint_config_path;
std::string asset_name;
std::string endpoint_name;
std::atomic_bool running {true};
std::atomic_bool viewer_running {true};
std::atomic_bool endpoint_ready {false};
std::mutex mujoco_mutex;
std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter> range_adapter;
std::unique_ptr<hako::robots::sensor::ultrasonic::UltrasonicSensor> ultrasonic_sensor;
hako::examples::sensors::ultrasonic::AppState app_state {};
hako::robots::sensor::ultrasonic::UltrasonicFrame last_frame {};
bool has_last_frame = false;
int qpos_addr = -1;
int sensor_site_id = -1;

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
        << "  " << program << " [model.xml] [sensor-config.json] [pdu-def.json] [endpoint.json]\n\n"
        << "Defaults:\n"
        << "  model.xml          " << kDefaultModelPath << "\n"
        << "  sensor-config.json " << kDefaultSensorConfigPath << "\n"
        << "  pdu-def.json       " << kDefaultPduDefPath << "\n"
        << "  endpoint.json      " << kDefaultEndpointConfigPath << "\n\n"
        << "Environment:\n"
        << "  HAKO_ULTRASONIC_ASSET_NAME       asset name, default " << kDefaultAssetName << "\n"
        << "  HAKO_ULTRASONIC_ENDPOINT_NAME    endpoint name, default " << kDefaultEndpointName << "\n";
}

void PrintPublisherHelp()
{
    std::cout << R"(
Controls:
  i : move forward  (+X)
  k : move backward (-X)
  j : move left     (+Y)
  l : move right    (-Y)
  s : print latest ultrasonic range
  h : help
  q : quit publisher

Viewer:
  The latest measured ray is drawn in the MuJoCo viewer.
  The Python reader prints the range PDU published over Hakoniwa.
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

void ApplyPendingMotion(mjModel* model, mjData* data)
{
    const int forward_steps = app_state.move_forward.exchange(0);
    const int left_steps = app_state.move_left.exchange(0);
    if (forward_steps == 0 && left_steps == 0) {
        return;
    }

    hako::examples::sensors::MoveFreejointPlanarSteps(
        model,
        data,
        qpos_addr,
        forward_steps,
        left_steps,
        kMoveStep);
    hako::examples::sensors::PrintPlanarStepMove(
        "base",
        forward_steps,
        left_steps,
        kMoveStep);
    hako::examples::sensors::PrintFreejointPosition(data, qpos_addr, "base_pos");
}

static int OnManualTimingControl(hako_asset_context_t* context)
{
    (void)context;
    auto* model = world->getModel();
    auto* data = world->getData();
    const double sim_timestep = model->opt.timestep;
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(sim_timestep * 1.0e6);
    const auto& config = ultrasonic_sensor->GetConfig();

    std::cout << "[INFO] Ultrasonic Hakoniwa asset started." << std::endl;
    PrintPublisherHelp();

    while (running.load() && app_state.running.load()) {
        if (endpoint_ready.load()) {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            ApplyPendingMotion(model, data);
            world->advanceTimeStep();

            if (ultrasonic_sensor->ShouldUpdate(sim_timestep)) {
                hako::robots::sensor::ultrasonic::UltrasonicFrame frame {};
                ultrasonic_sensor->Measure(frame);
                last_frame = frame;
                has_last_frame = true;

                if (range_adapter != nullptr &&
                    !range_adapter->send(config, frame))
                {
                    std::cerr << "[WARN] Failed to send ultrasonic range PDU." << std::endl;
                }

                if (app_state.pending_measure.exchange(false)) {
                    hako::examples::sensors::ultrasonic::PrintFrame(frame);
                }
            }
        }

        if (app_state.print_help.exchange(false)) {
            PrintPublisherHelp();
        }
        hako_asset_usleep(delta_time_usec);
    }

    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage(argv[0]);
        return 0;
    }

    model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    sensor_config_path = argc > 2 ? argv[2] : kDefaultSensorConfigPath;
    pdu_def_path = argc > 3 ? argv[3] : kDefaultPduDefPath;
    endpoint_config_path = argc > 4 ? argv[4] : kDefaultEndpointConfigPath;
    asset_name = EnvOrDefault("HAKO_ULTRASONIC_ASSET_NAME", kDefaultAssetName);
    endpoint_name = EnvOrDefault("HAKO_ULTRASONIC_ENDPOINT_NAME", kDefaultEndpointName);

    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    auto* model = world->getModel();
    auto* data = world->getData();
    if (model == nullptr || data == nullptr) {
        std::cerr << "[ERROR] model/data is null" << std::endl;
        return 1;
    }

    qpos_addr = hako::examples::sensors::FindFreejointQposAddr(model, kDefaultBaseJointName);
    sensor_site_id =
        hako::examples::sensors::ultrasonic::FindSiteId(model, kDefaultSensorSiteName);

    ultrasonic_sensor =
        std::make_unique<hako::robots::sensor::ultrasonic::UltrasonicSensor>(
            world,
            kDefaultSensorSiteName,
            kDefaultExcludeBodyName);
    if (!ultrasonic_sensor->LoadConfig(sensor_config_path)) {
        std::cerr << "[ERROR] Failed to load ultrasonic config: "
                  << sensor_config_path << std::endl;
        return 1;
    }

    const auto& config = ultrasonic_sensor->GetConfig();
    const std::string range_pdu_name = config.pdu_config.pdu_name.empty()
        ? kDefaultRangePduName
        : config.pdu_config.pdu_name;

    hako_asset_callbacks_t callbacks {};
    callbacks.on_initialize = OnInitialize;
    callbacks.on_simulation_step = nullptr;
    callbacks.on_manual_timing_control = OnManualTimingControl;
    callbacks.on_reset = OnReset;

    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(model->opt.timestep * 1.0e6);

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

    const hakoniwa::pdu::PduKey range_key {asset_name, range_pdu_name};
    range_adapter =
        std::make_unique<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter>(
            *endpoint,
            range_key);

    std::cout << "[INFO] Starting ultrasonic asset with:" << std::endl;
    std::cout << "  model    : " << model_path << std::endl;
    std::cout << "  sensor   : " << sensor_config_path << std::endl;
    std::cout << "  pdu_def  : " << pdu_def_path << std::endl;
    std::cout << "  endpoint : " << endpoint_config_path << std::endl;
    std::cout << "[INFO] asset=" << asset_name
              << " site=" << kDefaultSensorSiteName
              << " pdu=" << range_pdu_name
              << " sensor_rate_hz=" << config.update_rate
              << " pdu_config_rate_hz=" << config.pdu_config.update_rate_hz
              << std::endl;

    std::thread terminal_thread(
        hako::examples::sensors::ultrasonic::TerminalCommandLoop,
        std::ref(app_state));
    int start_result = 0;
    std::atomic_bool asset_thread_finished {false};
    std::thread asset_thread([&]() {
        start_result = hako_asset_start_no_wait(IsForceStop);
        asset_thread_finished.store(true);
        running.store(false);
        viewer_running.store(false);
    });

    MujocoRenderRuntime render_runtime(
        model,
        data,
        viewer_running,
        mujoco_mutex,
        MujocoRenderWindowMode::Visible);
    render_runtime.SetOverlayCallback([&](mjvScene& scene) {
        if (!app_state.running.load()) {
            running.store(false);
            viewer_running.store(false);
            return;
        }
        if (!has_last_frame) {
            return;
        }
        const auto line =
            hako::examples::sensors::ultrasonic::MakeUltrasonicCenterRayDebugLine(
                data,
                sensor_site_id,
                last_frame);
        hako::robots::sensor::debug::AddRaycastDebugLine(
            scene,
            line,
            kDebugRayWidth);
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
    range_adapter.reset();
    ultrasonic_sensor.reset();
    hako_conductor_stop();
    if (start_result != 0 && asset_finished_before_viewer_return) {
        std::cerr << "[ERROR] hako_asset_start() returns "
                  << start_result << std::endl;
        return 1;
    }
    return 0;
}
