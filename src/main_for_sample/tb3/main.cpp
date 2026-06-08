#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

#include <mujoco/mujoco.h>

#include "mujoco_debug.hpp"
#if USE_VIEWER
#include "viewer/mujoco_viewer.hpp"
#endif

#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/adapter/geometry_msgs/twist.hpp"
#include "hakoniwa/pdu/adapter/hako_msgs/game_controller_operation.hpp"
#include "hakoniwa/pdu/adapter/nav_msgs/odometry.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/imu.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/joint_state.hpp"
#include "hakoniwa/pdu/adapter/sensor_msgs/laser_scan.hpp"
#include "hakoniwa/pdu/adapter/tf2_msgs/tf_message.hpp"
#include "physics/physics_impl.hpp"
#include "robots/tb3/tb3_robot.hpp"

namespace {
std::shared_ptr<hako::robots::physics::IWorld> world;
std::mutex data_mutex;
bool running_flag = true;
std::string lidar_config_override_path;
using hako::robots::tb3::Tb3RuntimeConfig;

std::filesystem::path repo_root_path()
{
    const char* env = std::getenv("HAKO_TB3_ROOT");
    if (env != nullptr && env[0] != '\0') {
        return std::filesystem::path(env).lexically_normal();
    }

    auto path = std::filesystem::current_path().lexically_normal();
    while (!path.empty()) {
        if (std::filesystem::exists(path / "models/tb3/turtlebot3_burger_world.xml") &&
            std::filesystem::exists(path / "config/tb3-pdudef-compact.json")) {
            return path;
        }
        const auto parent = path.parent_path();
        if (parent == path) {
            break;
        }
        path = parent;
    }

    return std::filesystem::current_path().lexically_normal();
}
const std::string model_path =
    (repo_root_path() / "models/tb3/turtlebot3_burger_world.xml").string();
const std::string hako_config_path = (repo_root_path() / "config/tb3-pdudef-compact.json").string();
const std::string endpoint_config_path = (repo_root_path() / "config/endpoint/tb3_sim_endpoint.json").string();
const std::string lidar_config_path =
    (repo_root_path() / "config/sensors/lidar/lds-02.json").string();
const std::string imu_config_path =
    (repo_root_path() / "config/sensors/imu/tb3-imu.json").string();
const std::string joint_state_config_path =
    (repo_root_path() / "config/sensors/joint_state/tb3-wheel-joint-states.json").string();
const std::string odom_config_path =
    (repo_root_path() / "config/sensors/odometry/tb3-ground-truth-odom.json").string();
const std::string tf_config_path =
    (repo_root_path() / "config/sensors/tf/tb3-basic-tf.json").string();
const std::string left_actuator_config_path =
    (repo_root_path() / "config/actuator/joint/tb3_left_wheel.json").string();
const std::string right_actuator_config_path =
    (repo_root_path() / "config/actuator/joint/tb3_right_wheel.json").string();

std::string resolve_repo_path(const std::string& path)
{
    const std::filesystem::path candidate(path);
    if (candidate.is_absolute()) {
        return candidate.lexically_normal().string();
    }
    return (repo_root_path() / candidate).lexically_normal().string();
}

std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

double get_env_double(const char* name, double default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stod(env);
    } catch (...) {
        return default_value;
    }
}

double get_env_double_compat(const char* name, const char* legacy_name, double default_value)
{
    const char* env = std::getenv(name);
    if (env != nullptr && env[0] != '\0') {
        return get_env_double(name, default_value);
    }
    return get_env_double(legacy_name, default_value);
}

Tb3RuntimeConfig load_runtime_config()
{
    Tb3RuntimeConfig config {};
    config.endpoint_path = get_env_string("HAKO_TB3_ENDPOINT_CONFIG_PATH", endpoint_config_path);
    config.endpoint_name = get_env_string("HAKO_TB3_ENDPOINT_NAME", "tb3_sim_endpoint");
    config.lidar_config = get_env_string("HAKO_TB3_LIDAR_CONFIG_PATH", lidar_config_path);
    if (!lidar_config_override_path.empty()) {
        config.lidar_config = lidar_config_override_path;
    }
    config.lidar_config = resolve_repo_path(config.lidar_config);
    config.imu_config = resolve_repo_path(get_env_string("HAKO_TB3_IMU_CONFIG_PATH", imu_config_path));
    config.joint_state_config =
        resolve_repo_path(get_env_string("HAKO_TB3_JOINT_STATE_CONFIG_PATH", joint_state_config_path));
    config.odom_config = resolve_repo_path(get_env_string("HAKO_TB3_ODOM_CONFIG_PATH", odom_config_path));
    config.tf_config = resolve_repo_path(get_env_string("HAKO_TB3_TF_CONFIG_PATH", tf_config_path));
    config.left_wheel_actuator_config =
        resolve_repo_path(get_env_string("HAKO_TB3_LEFT_ACTUATOR_CONFIG_PATH", left_actuator_config_path));
    config.right_wheel_actuator_config =
        resolve_repo_path(get_env_string("HAKO_TB3_RIGHT_ACTUATOR_CONFIG_PATH", right_actuator_config_path));
    config.max_linear_velocity = get_env_double_compat(
        "HAKO_TB3_MAX_LINEAR_VELOCITY",
        "HAKO_TB3_DRIVE_GAIN",
        0.22);
    config.max_yaw_rate = get_env_double_compat(
        "HAKO_TB3_MAX_YAW_RATE",
        "HAKO_TB3_TURN_GAIN",
        1.2);
    config.max_linear_acceleration = get_env_double(
        "HAKO_TB3_MAX_LINEAR_ACCELERATION",
        0.1);
    config.max_yaw_acceleration = get_env_double(
        "HAKO_TB3_MAX_YAW_ACCELERATION",
        0.5);
    config.command_deadzone = get_env_double(
        "HAKO_TB3_COMMAND_DEADZONE",
        0.1);
    config.wheel_radius = get_env_double("HAKO_TB3_WHEEL_RADIUS", 0.033);
    config.wheel_separation = get_env_double("HAKO_TB3_WHEEL_SEPARATION", 0.16);
    config.max_wheel_angular_velocity = get_env_double_compat(
        "HAKO_TB3_MAX_WHEEL_ANGULAR_VELOCITY",
        "HAKO_TB3_MAX_TORQUE",
        12.0);
    config.max_wheel_angular_acceleration = get_env_double(
        "HAKO_TB3_MAX_WHEEL_ANGULAR_ACCELERATION",
        25.0);
    config.lidar_yaw_bias_deg = get_env_double("HAKO_TB3_LIDAR_YAW_BIAS_DEG", 0.0);
    config.lidar_origin_offset = get_env_double("HAKO_TB3_LIDAR_ORIGIN_OFFSET", 0.0);
    config.asset_name = get_env_string("HAKO_ASSET_NAME", "tb3_sim");
    config.asset_config_path = get_env_string("HAKO_ASSET_CONFIG_PATH", hako_config_path);
    return config;
}

static int my_on_initialize(hako_asset_context_t* context) { (void)context; return 0; }
static int my_on_reset(hako_asset_context_t* context)      { (void)context; return 0; }

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;

    const double sim_timestep  = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);
    const Tb3RuntimeConfig runtime = load_runtime_config();
    hako::robots::tb3::Tb3Robot tb3(world, runtime);
    hakoniwa::pdu::Endpoint endpoint(runtime.endpoint_name, HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    endpoint.open(runtime.endpoint_path);

    const hakoniwa::pdu::PduKey gamepad_key      {"TB3", "hako_cmd_game"};
    const hakoniwa::pdu::PduKey base_pos_key     {"TB3", "base_link_pos"};
    const hakoniwa::pdu::PduKey base_scan_pos_key{"TB3", "base_scan_pos"};
    const hakoniwa::pdu::PduKey laser_scan_key   {"TB3", "laser_scan"};
    const hakoniwa::pdu::PduKey imu_key          {"TB3", "imu"};
    const hakoniwa::pdu::PduKey joint_state_key  {"TB3", "joint_states"};
    const hakoniwa::pdu::PduKey odom_key         {"TB3", "odom"};
    const hakoniwa::pdu::PduKey tf_key           {"TB3", "tf"};

    endpoint.start();
    endpoint.post_start();
    std::cout << "[INFO] TB3 endpoint started successfully." << std::endl;

    hako::robots::pdu::adapter::sensor_msgs::LaserScanPduAdapter laser_scan_adapter(endpoint, laser_scan_key);
    hako::robots::pdu::adapter::sensor_msgs::ImuPduAdapter imu_adapter(endpoint, imu_key);
    hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter joint_state_adapter(endpoint, joint_state_key);
    hako::robots::pdu::adapter::nav_msgs::OdometryPduAdapter odom_adapter(endpoint, odom_key);
    hako::robots::pdu::adapter::tf2_msgs::TfPduAdapter tf_adapter(endpoint, tf_key);
    hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter base_pos_adapter(endpoint, base_pos_key);
    hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter base_scan_pos_adapter(endpoint, base_scan_pos_key);
    hako::robots::tb3::Tb3CommandConfig command_config {};
    command_config.max_linear_velocity = runtime.max_linear_velocity;
    command_config.max_yaw_rate = runtime.max_yaw_rate;
    command_config.command_deadzone = runtime.command_deadzone;
    hako::robots::pdu::adapter::hako_msgs::GamepadCommandPduAdapter gamepad_adapter(
        endpoint,
        gamepad_key,
        command_config);

    std::string tb3_error;
    if (!tb3.Initialize(&tb3_error)) {
        std::cerr << "ERROR: " << tb3_error << std::endl;
        endpoint.stop();
        endpoint.close();
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

            (void)gamepad_adapter.recv(command);
            // --- 制御 ---
            tb3.ApplyCommand(command);
            tb3.Step();
            // --- base_link_pos 送信（1ms周期） ---
            (void)base_pos_adapter.send(tb3.GetBasePosition(), tb3.GetBaseEuler());

            const double sim_time_sec = static_cast<double>(hako_asset_simulation_time()) / 1.0e6;
            if (tb3.MaybeBuildImu(sim_timestep, sim_time_sec, imu_frame)) {
                (void)imu_adapter.send(imu_frame);
            }
            if (tb3.MaybeBuildJointState(sim_timestep, sim_time_sec, joint_state_frame)) {
                (void)joint_state_adapter.send(joint_state_frame);
            }

            if (tb3.MaybeBuildOdometry(sim_timestep, sim_time_sec, odom_frame)) {
                (void)odom_adapter.send(odom_frame);
            }
            if (tb3.MaybeBuildTf(sim_timestep, sim_time_sec, tf_frame)) {
                (void)tf_adapter.send(tf_frame);
            }

            // --- LiDAR スキャン（lidar_period_sec 周期） ---
            // Unity: EventTick() — update_cycle ごとに Scan() → FlushNamedPdu()
            if (tb3.MaybeBuildLaserScan(sim_timestep, laser_scan_frame)) {
                (void)laser_scan_adapter.send(laser_scan_frame);

                // base_scan_pos も同じタイミングでだけ送る
                (void)base_scan_pos_adapter.send(tb3.GetBaseScanPosition(), tb3.GetBaseScanEuler());
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

    (void)endpoint.stop();
    endpoint.close();
    return 0;
}

static hako_asset_callbacks_t my_callback;

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> w)
{
    my_callback.on_initialize          = my_on_initialize;
    my_callback.on_simulation_step     = nullptr;
    my_callback.on_manual_timing_control = my_manual_timing_control;
    my_callback.on_reset               = my_on_reset;

    const Tb3RuntimeConfig runtime = load_runtime_config();
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(w->getModel()->opt.timestep * 1e6);

    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(runtime.asset_name.c_str(), runtime.asset_config_path.c_str(),
                                  &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_register() returns " << ret << std::endl;
        return;
    }
    ret = hako_asset_start();
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_start() returns " << ret << std::endl;
        return;
    }
}

} // namespace

int main(int argc, const char* argv[])
{
    if (argc >= 2) {
        lidar_config_override_path = argv[1];
    }

    std::cout << "[INFO] Creating TB3 world and loading model..." << std::endl;
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
        std::cout << "[INFO] TB3 model loaded successfully from: " << model_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load TB3 model: " << e.what() << std::endl;
        std::cerr << "Please check if the model file exists at: " << model_path << std::endl;
        return 1;
    }

    std::thread sim_thread(simulation_thread, world);

#if USE_VIEWER
    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));
#else
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    running_flag = false;
    sim_thread.join();
    std::cout << "[INFO] TB3 simulation completed successfully." << std::endl;
    return 0;
}
