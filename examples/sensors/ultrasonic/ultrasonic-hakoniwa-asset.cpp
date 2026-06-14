#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/ultrasonic/support/ultrasonic_example_support.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
#include "physics/physics_impl.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"
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
std::mutex mujoco_mutex;
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

int RunManualTimingControl(hakoniwa::pdu::Endpoint& endpoint)
{
    (void)endpoint;
    auto* model = world->getModel();
    auto* data = world->getData();
    const double sim_timestep = model->opt.timestep;
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(sim_timestep * 1.0e6);
    const auto& config = ultrasonic_sensor->GetConfig();

    std::cout << "[INFO] Ultrasonic Hakoniwa asset started." << std::endl;
    PrintPublisherHelp();

    while (running.load() && app_state.running.load()) {
        {
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

    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(model->opt.timestep * 1.0e6);

    hako::robots::runtime::HakoniwaAssetLifecycle asset_lifecycle({
        endpoint_name,
        endpoint_config_path,
        asset_name,
        pdu_def_path,
        delta_time_usec,
        HAKO_ASSET_MODEL_PLANT
    });

    std::string lifecycle_error;
    if (!asset_lifecycle.OpenEndpoint(&lifecycle_error)) {
        std::cerr << "[ERROR] " << lifecycle_error << std::endl;
        return 1;
    }

    const hakoniwa::pdu::PduKey range_key {asset_name, range_pdu_name};
    range_adapter =
        std::make_unique<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter>(
            asset_lifecycle.Endpoint(),
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
    bool start_result = true;
    std::atomic_bool asset_thread_finished {false};
    std::thread asset_thread([&]() {
        start_result = asset_lifecycle.RegisterAndRunAssetNoWait(
            [](hakoniwa::pdu::Endpoint& endpoint) {
                return RunManualTimingControl(endpoint);
            },
            []() {
                return running.load() && app_state.running.load() ? 0 : 1;
            },
            {},
            &lifecycle_error);
        if (!start_result) {
            std::cerr << "[ERROR] " << lifecycle_error << std::endl;
        }
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
    range_adapter.reset();
    ultrasonic_sensor.reset();
    asset_lifecycle.StopAndClose();
    if (!start_result && asset_finished_before_viewer_return) {
        return 1;
    }
    return 0;
}
