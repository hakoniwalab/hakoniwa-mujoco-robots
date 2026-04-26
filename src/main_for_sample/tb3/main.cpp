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
#include "mujoco_viewer.hpp"
#endif

#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/hako_msgs/pdu_cpptype_GameControllerOperation.hpp"
#include "hakoniwa/pdu/hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "hakoniwa/pdu/nav_msgs/pdu_cpptype_Odometry.hpp"
#include "hakoniwa/pdu/nav_msgs/pdu_cpptype_conv_Odometry.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_Imu.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_JointState.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_Imu.hpp"
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "hakoniwa/pdu/tf2_msgs/pdu_cpptype_TFMessage.hpp"
#include "hakoniwa/pdu/tf2_msgs/pdu_cpptype_conv_TFMessage.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
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
    const auto source_path = std::filesystem::path(__FILE__).lexically_normal();
    return source_path.parent_path().parent_path().parent_path().parent_path();
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

struct Tb3CommandState {
    HakoCpp_GameControllerOperation gamepad {};
    bool has_input {false};
};

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
    config.drive_gain = get_env_double("HAKO_TB3_DRIVE_GAIN", 0.1);
    config.turn_gain = get_env_double("HAKO_TB3_TURN_GAIN", 0.15);
    config.max_torque = get_env_double("HAKO_TB3_MAX_TORQUE", 1.0);
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

    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_pos_ep(endpoint, base_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_scan_pos_ep(endpoint, base_scan_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_LaserScan, hako::pdu::msgs::sensor_msgs::LaserScan>
        laser_scan_ep(endpoint, laser_scan_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Imu, hako::pdu::msgs::sensor_msgs::Imu>
        imu_ep(endpoint, imu_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_JointState, hako::pdu::msgs::sensor_msgs::JointState>
        joint_state_ep(endpoint, joint_state_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Odometry, hako::pdu::msgs::nav_msgs::Odometry>
        odom_ep(endpoint, odom_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_TFMessage, hako::pdu::msgs::tf2_msgs::TFMessage>
        tf_ep(endpoint, tf_key);

    hako::pdu::msgs::hako_msgs::GameControllerOperation gamepad_conv;
    std::vector<std::byte> gamepad_buf(endpoint.get_pdu_size(gamepad_key), std::byte{0});

    std::string tb3_error;
    if (!tb3.Initialize(&tb3_error)) {
        std::cerr << "ERROR: " << tb3_error << std::endl;
        endpoint.stop();
        endpoint.close();
        return -1;
    }

    HakoCpp_LaserScan laser_scan {};
    HakoCpp_Imu imu {};
    HakoCpp_JointState joint_state {};
    HakoCpp_Odometry odom {};
    HakoCpp_TFMessage tf {};
    HakoCpp_Twist base_scan_pos {};

    int step = 0;
    Tb3CommandState command_state {};

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            // --- gamepad 受信 ---
            HakoCpp_GameControllerOperation latest {};
            size_t gamepad_received = 0;
            const auto gamepad_rc = endpoint.recv(
                gamepad_key,
                std::span<std::byte>(gamepad_buf.data(), gamepad_buf.size()),
                gamepad_received);
            if ((gamepad_rc == HAKO_PDU_ERR_OK) && (gamepad_received > 0)) {
                if (gamepad_conv.pdu2cpp(reinterpret_cast<char*>(gamepad_buf.data()), latest)) {
                    command_state.gamepad = latest;
                    command_state.has_input = true;
                } else {
                    hako_asset_usleep(delta_time_usec * 1000);
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    continue;
                }
            }

            // --- 制御 ---
            tb3.ApplyCommand(command_state.gamepad, command_state.has_input);
            tb3.Step();

            // --- base_link_pos 送信（1ms周期） ---
            HakoCpp_Twist base_pos {};
            tb3.FillBasePose(base_pos);
            (void)base_pos_ep.send(base_pos);

            const double sim_time_sec = static_cast<double>(hako_asset_simulation_time()) / 1.0e6;

            if (tb3.MaybeBuildImu(sim_timestep, sim_time_sec, imu)) {
                (void)imu_ep.send(imu);
            }
            if (tb3.MaybeBuildJointState(sim_timestep, sim_time_sec, joint_state)) {
                (void)joint_state_ep.send(joint_state);
            }
            if (tb3.MaybeBuildOdometry(sim_timestep, sim_time_sec, odom)) {
                (void)odom_ep.send(odom);
            }
            if (tb3.MaybeBuildTf(sim_timestep, sim_time_sec, tf)) {
                (void)tf_ep.send(tf);
            }

            // --- LiDAR スキャン（lidar_period_sec 周期） ---
            // Unity: EventTick() — update_cycle ごとに Scan() → FlushNamedPdu()
            if (tb3.MaybeBuildLaserScan(sim_timestep, laser_scan)) {
                (void)laser_scan_ep.send(laser_scan);

                // base_scan_pos も同じタイミングでだけ送る
                tb3.FillBaseScanPose(base_scan_pos);
                (void)base_scan_pos_ep.send(base_scan_pos);
            }

            // --- デバッグログ（500ステップごと） ---
            if ((step % 500) == 0) {
                tb3.EmitDebugLog(step);
            }
            ++step;
        }

        hako_asset_usleep(delta_time_usec);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        const double sleep_time = sim_timestep - elapsed.count();
        if (sleep_time > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
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
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load TB3 model: " << e.what() << std::endl;
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
