#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <thread>

#include <mujoco/mujoco.h>

#include "mujoco_debug.hpp"
#include "viewer/mujoco_viewer.hpp"

#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "robots/tb3/tb3_hakoniwa_adapter.hpp"
#include "robots/tb3/tb3_robot.hpp"
#include "robots/tb3/tb3_runtime_config_loader.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"

#include "hakoniwa/pdu/adapter/sensor_msgs/image.hpp"
#include "sensors/camera/camera_config_loader.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

namespace {
std::shared_ptr<hako::robots::physics::IWorld> world;
std::mutex data_mutex;
bool running_flag = true;
std::string lidar_config_override_path;
using hako::robots::tb3::Tb3RuntimeConfig;

std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter> image_adapter;
std::unique_ptr<hako::robots::sensor::camera::CameraSensor> camera_sensor;
std::optional<hako::robots::sensor::camera::ImageFrame> latest_camera_frame;
std::atomic_bool render_running {true};
hako::robots::config::AssetManifest asset_manifest;
Tb3RuntimeConfig runtime;
hako::robots::runtime::HakoniwaAssetLifecycle* lifecycle {nullptr};

std::optional<hakoniwa::pdu::PduKey> make_manifest_pdu_key(
    const hako::robots::config::AssetManifest& manifest,
    const std::string& component_id,
    const std::string& config_path)
{
    std::string robot_name = manifest.ComponentPduRobot(component_id);
    if (robot_name.empty()) {
        std::cerr << "[ERROR] manifest component '" << component_id
                  << "' must define pdu_robot for PDU connection." << std::endl;
        return std::nullopt;
    }

    std::string pdu_name;
    std::string error;
    if (!config_path.empty() &&
        !hako::robots::config::ReadPduNameFromConfig(config_path, pdu_name, &error))
    {
        std::cerr << "[WARN] Failed to read PDU name for component '"
                  << component_id << "': " << error << std::endl;
    }
    if (pdu_name.empty()) {
        std::cerr << "[ERROR] Failed to resolve PDU name for component '"
                  << component_id << "'." << std::endl;
        return std::nullopt;
    }
    return hakoniwa::pdu::PduKey {robot_name, pdu_name};
}

static bool initialize_camera(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    hakoniwa::pdu::Endpoint& endpoint,
    MujocoRenderRuntime& render_runtime,
    const std::string& config_path)
{
    hako::robots::sensor::camera::CameraProfileConfig profile {};
    if (!hako::robots::sensor::camera::LoadCameraProfileConfigFromJson(config_path, profile)) {
        std::cerr << "[ERROR] Failed to load camera config: "
                  << config_path << std::endl;
        return false;
    }
    const std::string camera_name =
        profile.mjcf_binding.camera_name.empty()
            ? "color_camera"
            : profile.mjcf_binding.camera_name;

    const auto image_key =
        make_manifest_pdu_key(asset_manifest, "color_camera", config_path);
    if (!image_key.has_value()) {
        return false;
    }
    image_adapter = std::make_unique<hako::robots::pdu::adapter::sensor_msgs::ImagePduAdapter>(
        endpoint,
        *image_key);

    auto sensor_renderer = render_runtime.CreateCameraRenderer(world);
    camera_sensor = std::make_unique<hako::robots::sensor::camera::CameraSensor>(
        sensor_renderer,
        camera_name);
    if (!camera_sensor->LoadConfig(profile.spec)) {
        std::cerr << "[ERROR] Failed to validate camera config: "
                  << config_path << std::endl;
        return false;
    }
    std::cout << "[INFO] TB3 camera initialized:"
              << " config=" << config_path
              << " camera=" << camera_name
              << " pdu=" << image_key->robot << "/" << image_key->pdu
              << " sensor_rate_hz=" << profile.spec.update_rate
              << " pdu_config_rate_hz=" << profile.pdu_config.update_rate_hz
              << std::endl;

    render_runtime.SetPreRenderCallback([]() {
        if (camera_sensor == nullptr) {
            return;
        }
        hako::robots::sensor::camera::ImageFrame frame {};
        camera_sensor->Capture(frame);
        if (!frame.data.empty()) {
            latest_camera_frame = std::move(frame);
        }
    });
    return true;
}

static int run_manual_timing_control(hakoniwa::pdu::Endpoint& endpoint)
{
    const double sim_timestep  = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);
    hako::robots::tb3::Tb3Robot tb3(world, runtime);

    std::string tb3_error;
    if (!tb3.Initialize(&tb3_error)) {
        std::cerr << "ERROR: " << tb3_error << std::endl;
        return -1;
    }

    hako::robots::tb3::Tb3HakoniwaAdapter tb3_io(endpoint, asset_manifest, runtime);
    std::string io_error;
    if (!tb3_io.Initialize(&io_error)) {
        std::cerr << "ERROR: " << io_error << std::endl;
        return -1;
    }

    hako::robots::sensor::ImuFrame imu_frame {};
    hako::robots::sensor::lidar::LaserScanFrame laser_scan_frame {};
    hako::robots::sensor::JointStateFrame joint_state_frame {};
    hako::robots::sensor::OdometryFrame odom_frame {};
    hako::robots::sensor::TfFrame tf_frame {};

    int step = 0;
    hako::robots::tb3::Tb3Command command {};

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            (void)tb3_io.RecvCommand(command);
            // --- 制御 ---
            tb3.ApplyCommand(command);
            tb3.Step();
            // --- base_link_pos 送信（1ms周期） ---
            (void)tb3_io.PublishBasePose(tb3.GetBasePosition(), tb3.GetBaseEuler());

            const double sim_time_sec = static_cast<double>(hako_asset_simulation_time()) / 1.0e6;
            if (tb3.MaybeBuildImu(sim_timestep, sim_time_sec, imu_frame)) {
                (void)tb3_io.PublishImu(imu_frame);
            }
            if (tb3.MaybeBuildJointState(sim_timestep, sim_time_sec, joint_state_frame)) {
                (void)tb3_io.PublishJointState(joint_state_frame);
            }

            if (tb3.MaybeBuildOdometry(sim_timestep, sim_time_sec, odom_frame)) {
                (void)tb3_io.PublishOdometry(odom_frame);
            }
            if (tb3.MaybeBuildTf(sim_timestep, sim_time_sec, tf_frame)) {
                (void)tb3_io.PublishTf(tf_frame);
            }

            // --- LiDAR スキャン（lidar_period_sec 周期） ---
            // Unity: EventTick() — update_cycle ごとに Scan() → FlushNamedPdu()
            if (tb3.MaybeBuildLaserScan(sim_timestep, laser_scan_frame)) {
                (void)tb3_io.PublishLaserScan(laser_scan_frame);

                // base_scan_pos も同じタイミングでだけ送る
                (void)tb3_io.PublishBaseScanPose(tb3.GetBaseScanPosition(), tb3.GetBaseScanEuler());
            }
            if (lifecycle != nullptr &&
                lifecycle->IsReady() &&
                camera_sensor != nullptr &&
                image_adapter != nullptr &&
                latest_camera_frame.has_value() &&
                camera_sensor->ShouldUpdate(sim_timestep) &&
                !image_adapter->send(*latest_camera_frame))
            {
                std::cerr << "[WARN] Failed to send camera image PDU." << std::endl;
            }

            // --- デバッグログ（500ステップごと） ---
            if ((step % 500) == 0) {
                tb3.EmitDebugLog(step);
            }
            ++step;
        }

        hako_asset_usleep(delta_time_usec);
        { // keep real-time pacing at 20ms for each 20ms of simulated time
            static uint64_t previous_time = 0;
            const uint64_t current_time = static_cast<uint64_t>(hako_asset_simulation_time());
            const uint64_t time_diff = current_time - previous_time;
            if (time_diff >= 20 * 1000) {
                auto end = std::chrono::steady_clock::now();
                const auto elapsed = end - start;
                const auto target = std::chrono::milliseconds(20);
                if (elapsed < target) {
                    std::this_thread::sleep_for(target - elapsed);
                }
                previous_time = current_time;
            }
        }
    }

    return 0;
}

void simulation_thread(hako::robots::runtime::HakoniwaAssetLifecycle& asset_lifecycle)
{
    std::string lifecycle_error;
    if (!asset_lifecycle.RegisterAndRunAsset(
        [](hakoniwa::pdu::Endpoint& endpoint) {
            return run_manual_timing_control(endpoint);
        },
        {},
        &lifecycle_error))
    {
        std::cerr << "ERROR: " << lifecycle_error << std::endl;
    }
}

} // namespace

int main(int argc, const char* argv[])
{
    if (argc >= 2) {
        lidar_config_override_path = argv[1];
    }
    const std::string manifest_path = hako::robots::tb3::GetTb3ManifestPathFromEnvironment();
    std::string manifest_error;
    if (!hako::robots::config::LoadAssetManifestFromJson(manifest_path, asset_manifest, &manifest_error)) {
        std::cerr << "[ERROR] Failed to load TB3 manifest: "
                  << manifest_error << std::endl;
        return 1;
    }
    std::cout << "[INFO] Loaded TB3 manifest: " << manifest_path << std::endl;

    hako::robots::tb3::Tb3RuntimeConfigOverrides overrides {};
    overrides.lidar_config = lidar_config_override_path;
    runtime = hako::robots::tb3::LoadTb3RuntimeConfig(asset_manifest, overrides);
    std::cout << "[INFO] Using LiDAR config: " << runtime.lidar_config << std::endl;

    std::cout << "[INFO] Creating TB3 world and loading model..." << std::endl;
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(asset_manifest.model);
        std::cout << "[INFO] TB3 model loaded successfully from: " << asset_manifest.model << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load TB3 model: " << e.what() << std::endl;
        std::cerr << "Please check if the model file exists at: " << asset_manifest.model << std::endl;
        return 1;
    }

    hako::robots::runtime::HakoniwaAssetLifecycle asset_lifecycle({
        runtime.endpoint_name,
        runtime.endpoint_path,
        runtime.asset_name,
        runtime.asset_config_path,
        static_cast<hako_time_t>(world->getModel()->opt.timestep * 1e6),
        HAKO_ASSET_MODEL_PLANT
    });
    lifecycle = &asset_lifecycle;

    std::string lifecycle_error;
    if (!asset_lifecycle.OpenEndpoint(&lifecycle_error)) {
        std::cerr << "[ERROR] " << lifecycle_error << std::endl;
        lifecycle = nullptr;
        return 1;
    }
    std::cout << "[INFO] TB3 endpoint started successfully." << std::endl;

    render_running.store(true);
    MujocoRenderRuntime render_runtime(
        world->getModel(),
        world->getData(),
        render_running,
        data_mutex,
#if USE_VIEWER
        MujocoRenderWindowMode::Visible
#else
        MujocoRenderWindowMode::Hidden
#endif
    );

    if (!initialize_camera(world, asset_lifecycle.Endpoint(), render_runtime, runtime.camera_config)) {
        std::cerr << "[ERROR] Failed to initialize camera." << std::endl;
        running_flag = false;
        render_running.store(false);
        lifecycle = nullptr;
        asset_lifecycle.StopAndClose();
        return 1;
    }

    std::thread sim_thread(simulation_thread, std::ref(asset_lifecycle));

#if USE_VIEWER
    render_runtime.Run();
#else
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    running_flag = false;
    render_running.store(false);
    sim_thread.join();
    camera_sensor.reset();
    image_adapter.reset();
    asset_lifecycle.StopAndClose();
    lifecycle = nullptr;
    std::cout << "[INFO] TB3 simulation completed successfully." << std::endl;
    return 0;
}
