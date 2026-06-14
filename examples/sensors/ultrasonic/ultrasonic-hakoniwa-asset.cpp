#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/ultrasonic/support/ultrasonic_example_support.hpp"
#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/range.hpp"
#include "physics/physics_impl.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"
#include "sensors/debug/raycast_debug.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <atomic>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

namespace {

class UltrasonicHakoniwaAssetApp
{
public:
    int Run(int argc, char** argv)
    {
        if (argc > 1 && std::string(argv[1]) == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }

        if (!LoadManifest(argc, argv) ||
            !InitializeWorld() ||
            !InitializeSensor() ||
            !InitializeLifecycle())
        {
            return 1;
        }

        return RunViewerAndAssetThreads();
    }

private:
    static constexpr const char* kDefaultManifestPath =
        "config/assets/ultrasonic-hakoniwa-asset.json";
    static constexpr const char* kUltrasonicComponentId = "front_ultrasonic";
    static constexpr const char* kDefaultSensorSiteName = "front_ultrasonic_site";
    static constexpr const char* kDefaultExcludeBodyName = "base_footprint";
    static constexpr const char* kDefaultBaseJointName = "base_freejoint";

    static constexpr double kMoveStep = 0.05;
    static constexpr double kDebugRayWidth = 0.006;

    static std::string EnvOrDefault(const char* name, const char* fallback)
    {
        const char* value = std::getenv(name);
        if (value != nullptr && value[0] != '\0') {
            return value;
        }
        return fallback;
    }

    static bool ReadEndpointNameFromConfig(
        const std::string& endpoint_config_path,
        std::string& out,
        std::string* error_message)
    {
        std::ifstream ifs(endpoint_config_path);
        if (!ifs) {
            if (error_message != nullptr) {
                *error_message = "failed to open endpoint config: " + endpoint_config_path;
            }
            return false;
        }

        try {
            nlohmann::json root;
            ifs >> root;
            if (!root.contains("name") || !root.at("name").is_string() ||
                root.at("name").get<std::string>().empty())
            {
                if (error_message != nullptr) {
                    *error_message = "endpoint field is missing or invalid: name";
                }
                return false;
            }
            out = root.at("name").get<std::string>();
        } catch (const std::exception& e) {
            if (error_message != nullptr) {
                *error_message = "failed to parse endpoint config: "
                    + endpoint_config_path + ": " + e.what();
            }
            return false;
        }

        return true;
    }

    static void PrintUsage(const char* program)
    {
        std::cout
            << "Usage:\n"
            << "  " << program << " [manifest.json]\n\n"
            << "Defaults:\n"
            << "  manifest.json " << kDefaultManifestPath << "\n\n"
            << "Environment:\n"
            << "  HAKO_ULTRASONIC_MANIFEST_PATH    manifest path, default " << kDefaultManifestPath << "\n"
            << "  HAKO_ULTRASONIC_ASSET_NAME       asset registration name, default manifest pdu_robot\n"
            << "  HAKO_ULTRASONIC_ENDPOINT_NAME    endpoint name, default endpoint JSON name\n";
    }

    static void PrintPublisherHelp()
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

    bool LoadManifest(int argc, char** argv)
    {
        const std::string manifest_path = argc > 1
            ? argv[1]
            : EnvOrDefault("HAKO_ULTRASONIC_MANIFEST_PATH", kDefaultManifestPath);

        std::string manifest_error;
        if (!hako::robots::config::LoadAssetManifestFromJson(
                manifest_path,
                manifest_,
                &manifest_error))
        {
            std::cerr << "[ERROR] Failed to load ultrasonic manifest: "
                      << manifest_error << std::endl;
            return false;
        }

        ultrasonic_component_ = manifest_.FindComponent(kUltrasonicComponentId);
        if (ultrasonic_component_ == nullptr) {
            std::cerr << "[ERROR] Manifest component is missing: "
                      << kUltrasonicComponentId << std::endl;
            return false;
        }
        if (ultrasonic_component_->pdu_robot.empty()) {
            std::cerr << "[ERROR] Manifest component pdu_robot is missing: "
                      << kUltrasonicComponentId << std::endl;
            return false;
        }

        asset_name_ = EnvOrDefault(
            "HAKO_ULTRASONIC_ASSET_NAME",
            ultrasonic_component_->pdu_robot.c_str());

        std::string endpoint_name_from_config;
        std::string endpoint_error;
        if (!ReadEndpointNameFromConfig(
                manifest_.endpoint,
                endpoint_name_from_config,
                &endpoint_error))
        {
            std::cerr << "[ERROR] " << endpoint_error << std::endl;
            return false;
        }
        endpoint_name_ = EnvOrDefault(
            "HAKO_ULTRASONIC_ENDPOINT_NAME",
            endpoint_name_from_config.c_str());

        return true;
    }

    bool InitializeWorld()
    {
        world_ = std::make_shared<hako::robots::physics::impl::WorldImpl>();
        try {
            world_->loadModel(manifest_.model);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
            return false;
        }

        if (world_->getModel() == nullptr || world_->getData() == nullptr) {
            std::cerr << "[ERROR] model/data is null" << std::endl;
            return false;
        }
        return true;
    }

    bool InitializeSensor()
    {
        auto* model = world_->getModel();
        qpos_addr_ = hako::examples::sensors::FindFreejointQposAddr(
            model,
            kDefaultBaseJointName);

        ultrasonic_sensor_ =
            std::make_unique<hako::robots::sensor::ultrasonic::UltrasonicSensor>(
                world_,
                kDefaultSensorSiteName,
                kDefaultExcludeBodyName);
        if (!ultrasonic_sensor_->LoadConfig(ultrasonic_component_->config)) {
            std::cerr << "[ERROR] Failed to load ultrasonic config: "
                      << ultrasonic_component_->config << std::endl;
            return false;
        }

        const auto& config = ultrasonic_sensor_->GetConfig();
        if (config.pdu_config.pdu_name.empty()) {
            std::cerr << "[ERROR] pdu_config.pdu_name is missing: "
                      << ultrasonic_component_->config << std::endl;
            return false;
        }
        range_pdu_name_ = config.pdu_config.pdu_name;
        sensor_site_name_ = config.runtime_binding.source_site.empty()
            ? kDefaultSensorSiteName
            : config.runtime_binding.source_site;
        sensor_site_id_ =
            hako::examples::sensors::ultrasonic::FindSiteId(model, sensor_site_name_);
        return true;
    }

    bool InitializeLifecycle()
    {
        const hako_time_t delta_time_usec =
            static_cast<hako_time_t>(world_->getModel()->opt.timestep * 1.0e6);

        asset_lifecycle_ =
            std::make_unique<hako::robots::runtime::HakoniwaAssetLifecycle>(
                hako::robots::runtime::HakoniwaAssetLifecycleConfig {
                    endpoint_name_,
                    manifest_.endpoint,
                    asset_name_,
                    manifest_.pdu_def,
                    delta_time_usec,
                    HAKO_ASSET_MODEL_PLANT
                });

        std::string lifecycle_error;
        if (!asset_lifecycle_->OpenEndpoint(&lifecycle_error)) {
            std::cerr << "[ERROR] " << lifecycle_error << std::endl;
            return false;
        }

        const hakoniwa::pdu::PduKey range_key {
            ultrasonic_component_->pdu_robot,
            range_pdu_name_};
        range_adapter_ =
            std::make_unique<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter>(
                asset_lifecycle_->Endpoint(),
                range_key);

        PrintStartupInfo();
        return true;
    }

    void PrintStartupInfo() const
    {
        const auto& config = ultrasonic_sensor_->GetConfig();
        std::cout << "[INFO] Starting ultrasonic asset with:" << std::endl;
        std::cout << "  manifest : " << manifest_.path << std::endl;
        std::cout << "  model    : " << manifest_.model << std::endl;
        std::cout << "  sensor   : " << ultrasonic_component_->config << std::endl;
        std::cout << "  pdu_def  : " << manifest_.pdu_def << std::endl;
        std::cout << "  endpoint : " << manifest_.endpoint << std::endl;
        std::cout << "[INFO] asset=" << asset_name_
                  << " pdu_robot=" << ultrasonic_component_->pdu_robot
                  << " site=" << sensor_site_name_
                  << " pdu=" << range_pdu_name_
                  << " sensor_rate_hz=" << config.update_rate
                  << " pdu_config_rate_hz=" << config.pdu_config.update_rate_hz
                  << std::endl;
    }

    void ApplyPendingMotion(mjModel* model, mjData* data)
    {
        const int forward_steps = app_state_.move_forward.exchange(0);
        const int left_steps = app_state_.move_left.exchange(0);
        if (forward_steps == 0 && left_steps == 0) {
            return;
        }

        hako::examples::sensors::MoveFreejointPlanarSteps(
            model,
            data,
            qpos_addr_,
            forward_steps,
            left_steps,
            kMoveStep);
        hako::examples::sensors::PrintPlanarStepMove(
            "base",
            forward_steps,
            left_steps,
            kMoveStep);
        hako::examples::sensors::PrintFreejointPosition(data, qpos_addr_, "base_pos");
    }

    int RunManualTimingControl(hakoniwa::pdu::Endpoint& endpoint)
    {
        (void)endpoint;
        auto* model = world_->getModel();
        auto* data = world_->getData();
        const double sim_timestep = model->opt.timestep;
        const hako_time_t delta_time_usec =
            static_cast<hako_time_t>(sim_timestep * 1.0e6);
        const auto& config = ultrasonic_sensor_->GetConfig();

        std::cout << "[INFO] Ultrasonic Hakoniwa asset started." << std::endl;
        PrintPublisherHelp();

        while (running_.load() && app_state_.running.load()) {
            {
                std::lock_guard<std::mutex> lock(mujoco_mutex_);
                ApplyPendingMotion(model, data);
                world_->advanceTimeStep();

                if (ultrasonic_sensor_->ShouldUpdate(sim_timestep)) {
                    hako::robots::sensor::ultrasonic::UltrasonicFrame frame {};
                    ultrasonic_sensor_->Measure(frame);
                    last_frame_ = frame;
                    has_last_frame_ = true;

                    if (range_adapter_ != nullptr &&
                        !range_adapter_->send(config, frame))
                    {
                        std::cerr << "[WARN] Failed to send ultrasonic range PDU." << std::endl;
                    }

                    if (app_state_.pending_measure.exchange(false)) {
                        hako::examples::sensors::ultrasonic::PrintFrame(frame);
                    }
                }
            }

            if (app_state_.print_help.exchange(false)) {
                PrintPublisherHelp();
            }
            hako_asset_usleep(delta_time_usec);
        }

        return 0;
    }

    void OnViewerOverlay(mjvScene& scene)
    {
        if (!app_state_.running.load()) {
            running_.store(false);
            viewer_running_.store(false);
            return;
        }
        if (!has_last_frame_) {
            return;
        }
        const auto line =
            hako::examples::sensors::ultrasonic::MakeUltrasonicCenterRayDebugLine(
                world_->getData(),
                sensor_site_id_,
                last_frame_);
        hako::robots::sensor::debug::AddRaycastDebugLine(
            scene,
            line,
            kDebugRayWidth);
    }

    int RunViewerAndAssetThreads()
    {
        std::thread terminal_thread(
            hako::examples::sensors::ultrasonic::TerminalCommandLoop,
            std::ref(app_state_));

        bool start_result = true;
        std::string lifecycle_error;
        std::atomic_bool asset_thread_finished {false};
        std::thread asset_thread([&]() {
            start_result = asset_lifecycle_->RegisterAndRunAssetNoWait(
                [this](hakoniwa::pdu::Endpoint& endpoint) {
                    return RunManualTimingControl(endpoint);
                },
                [this]() {
                    return running_.load() && app_state_.running.load() ? 0 : 1;
                },
                {},
                &lifecycle_error);
            if (!start_result) {
                std::cerr << "[ERROR] " << lifecycle_error << std::endl;
            }
            asset_thread_finished.store(true);
            running_.store(false);
            viewer_running_.store(false);
        });

        MujocoRenderRuntime render_runtime(
            world_->getModel(),
            world_->getData(),
            viewer_running_,
            mujoco_mutex_,
            MujocoRenderWindowMode::Visible);
        render_runtime.SetOverlayCallback([this](mjvScene& scene) {
            OnViewerOverlay(scene);
        });
        render_runtime.Run();

        const bool asset_finished_before_viewer_return = asset_thread_finished.load();
        running_.store(false);
        viewer_running_.store(false);
        app_state_.running.store(false);
        if (terminal_thread.joinable()) {
            terminal_thread.detach();
        }
        if (asset_thread.joinable()) {
            asset_thread.join();
        }
        range_adapter_.reset();
        ultrasonic_sensor_.reset();
        asset_lifecycle_->StopAndClose();
        if (!start_result && asset_finished_before_viewer_return) {
            return 1;
        }
        return 0;
    }

    hako::robots::config::AssetManifest manifest_ {};
    const hako::robots::config::AssetManifestComponent* ultrasonic_component_ {nullptr};
    std::string asset_name_ {};
    std::string endpoint_name_ {};
    std::string range_pdu_name_ {};
    std::string sensor_site_name_ {};

    std::shared_ptr<hako::robots::physics::impl::WorldImpl> world_ {};
    std::unique_ptr<hako::robots::runtime::HakoniwaAssetLifecycle> asset_lifecycle_ {};
    std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter> range_adapter_ {};
    std::unique_ptr<hako::robots::sensor::ultrasonic::UltrasonicSensor> ultrasonic_sensor_ {};

    std::atomic_bool running_ {true};
    std::atomic_bool viewer_running_ {true};
    std::mutex mujoco_mutex_ {};

    hako::examples::sensors::ultrasonic::AppState app_state_ {};
    hako::robots::sensor::ultrasonic::UltrasonicFrame last_frame_ {};
    bool has_last_frame_ {false};
    int qpos_addr_ {-1};
    int sensor_site_id_ {-1};
};

} // namespace

int main(int argc, char** argv)
{
    UltrasonicHakoniwaAssetApp app;
    return app.Run(argc, argv);
}
