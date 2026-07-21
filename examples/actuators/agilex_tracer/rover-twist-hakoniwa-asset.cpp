#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/adapter/geometry_msgs/twist.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "robots/rover/differential_drive_kinematics.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kDefaultAssetName = "RoverTwistAsset";
constexpr const char* kDefaultModelPath =
    "../hakoniwa-mbody-registry/bodies/agilex_tracer/generated/tracer_v1.minimal_world.xml";
constexpr const char* kDefaultLeftConfigPath =
    "config/actuator/joint/agilex_tracer_left_wheel.json";
constexpr const char* kDefaultRightConfigPath =
    "config/actuator/joint/agilex_tracer_right_wheel.json";
constexpr const char* kDefaultPduDefPath =
    "config/rover-twist-pdudef-compact.json";
constexpr const char* kDefaultEndpointConfigPath =
    "config/endpoint/rover_twist_endpoint.json";
constexpr const char* kDefaultEndpointName = "rover_twist_endpoint";
constexpr const char* kDefaultCmdVelPduName = "cmd_vel";

constexpr double kWheelRadius = 0.082;
constexpr double kWheelSeparation = 0.34;
constexpr double kMaxWheelAngularVelocity = 18.0;
constexpr double kMaxLinearX = 0.6;
constexpr double kMaxAngularZ = 2.0;

std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::shared_ptr<hako::robots::actuator::IJointActuator> left_actuator;
std::shared_ptr<hako::robots::actuator::IJointActuator> right_actuator;
std::unique_ptr<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter> cmd_vel_adapter;

std::string model_path;
std::string left_config_path;
std::string right_config_path;
std::string pdu_def_path;
std::string endpoint_config_path;
std::string asset_name;
std::string endpoint_name;
std::string cmd_vel_pdu_name;
bool viewer_enabled {true};

std::atomic_bool running {true};
std::atomic_bool endpoint_ready {false};
std::atomic_bool viewer_running {true};
std::mutex mujoco_mutex;
HakoCpp_Twist latest_cmd {};

constexpr hako::robots::rover::DifferentialDriveKinematics kTracerKinematics {
    kWheelRadius,
    kWheelSeparation,
    -1.0,
    1.0,
    kMaxWheelAngularVelocity,
};

std::string EnvOrDefault(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0') {
        return value;
    }
    return fallback;
}

bool EnvBoolOrDefault(const char* name, bool fallback)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF");
}

double Clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

hako::robots::rover::WheelVelocityTargets TwistToWheelTargets(const HakoCpp_Twist& cmd)
{
    const double linear_x = Clamp(cmd.linear.x, -kMaxLinearX, kMaxLinearX);
    const double angular_z = Clamp(cmd.angular.z, -kMaxAngularZ, kMaxAngularZ);
    return hako::robots::rover::ToWheelVelocityTargets(
        linear_x,
        angular_z,
        kTracerKinematics);
}

int FindJointQposAddr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    return joint_id < 0 ? -1 : model->jnt_qposadr[joint_id];
}

void PrintUsage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program
        << " [--no-viewer] [model.xml] [left-config.json] [right-config.json] [pdu-def.json] [endpoint.json]\n\n"
        << "Defaults:\n"
        << "  model.xml         " << kDefaultModelPath << "\n"
        << "  left-config.json  " << kDefaultLeftConfigPath << "\n"
        << "  right-config.json " << kDefaultRightConfigPath << "\n"
        << "  pdu-def.json      " << kDefaultPduDefPath << "\n"
        << "  endpoint.json     " << kDefaultEndpointConfigPath << "\n\n"
        << "Options:\n"
        << "  --no-viewer       run the Hakoniwa asset without a MuJoCo viewer\n\n"
        << "Environment:\n"
        << "  HAKO_ROVER_TWIST_ASSET_NAME     default " << kDefaultAssetName << "\n"
        << "  HAKO_ROVER_TWIST_ENDPOINT_NAME  default " << kDefaultEndpointName << "\n"
        << "  HAKO_ROVER_TWIST_PDU_NAME       default " << kDefaultCmdVelPduName << "\n"
        << "  HAKO_ROVER_TWIST_ENABLE_VIEWER  default 1\n";
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
    std::lock_guard<std::mutex> lock(mujoco_mutex);
    mj_resetData(world->getModel(), world->getData());
    mj_forward(world->getModel(), world->getData());
    latest_cmd = HakoCpp_Twist {};
    return 0;
}

int IsForceStop()
{
    return running.load() ? 0 : 1;
}

static int OnManualTimingControl(hako_asset_context_t* context)
{
    (void)context;
    auto* model = world->getModel();
    auto* data = world->getData();
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(model->opt.timestep * 1.0e6);
    const int base_qpos_addr = FindJointQposAddr(model, "base_freejoint");
    int step = 0;

    std::cout << "[INFO] Rover Twist Hakoniwa asset started." << std::endl;
    while (running.load()) {
        if (endpoint_ready.load()) {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            HakoCpp_Twist incoming {};
            if (cmd_vel_adapter != nullptr && cmd_vel_adapter->recv(incoming)) {
                latest_cmd = incoming;
            }
            const auto wheel_targets = TwistToWheelTargets(latest_cmd);
            left_actuator->SetTarget(wheel_targets.left);
            right_actuator->SetTarget(wheel_targets.right);
            world->advanceTimeStep();

            if ((step % 250) == 0 && base_qpos_addr >= 0) {
                std::cout
                    << std::fixed << std::setprecision(3)
                    << "time=" << std::setw(7) << data->time
                    << " base=("
                    << data->qpos[base_qpos_addr + 0] << ", "
                    << data->qpos[base_qpos_addr + 1] << ", "
                    << data->qpos[base_qpos_addr + 2] << ")"
                    << " cmd=("
                    << latest_cmd.linear.x << ", "
                    << latest_cmd.angular.z << ")"
                    << " wheel=("
                    << wheel_targets.left << ", "
                    << wheel_targets.right << ")"
                    << std::endl;
            }
            ++step;
        }
        hako_asset_usleep(delta_time_usec);
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> positional_args;
    viewer_enabled = EnvBoolOrDefault("HAKO_ROVER_TWIST_ENABLE_VIEWER", true);
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
        if (arg == "--no-viewer") {
            viewer_enabled = false;
            continue;
        }
        positional_args.push_back(arg);
    }

    model_path = positional_args.size() > 0 ? positional_args[0] : kDefaultModelPath;
    left_config_path = positional_args.size() > 1 ? positional_args[1] : kDefaultLeftConfigPath;
    right_config_path = positional_args.size() > 2 ? positional_args[2] : kDefaultRightConfigPath;
    pdu_def_path = positional_args.size() > 3 ? positional_args[3] : kDefaultPduDefPath;
    endpoint_config_path = positional_args.size() > 4 ? positional_args[4] : kDefaultEndpointConfigPath;
    asset_name = EnvOrDefault("HAKO_ROVER_TWIST_ASSET_NAME", kDefaultAssetName);
    endpoint_name = EnvOrDefault("HAKO_ROVER_TWIST_ENDPOINT_NAME", kDefaultEndpointName);
    cmd_vel_pdu_name = EnvOrDefault("HAKO_ROVER_TWIST_PDU_NAME", kDefaultCmdVelPduName);

    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    left_actuator = world->createJointActuator();
    right_actuator = world->createJointActuator();
    if (!left_actuator->LoadConfig(left_config_path) ||
        !right_actuator->LoadConfig(right_config_path))
    {
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
        std::cerr << "[ERROR] hako_asset_register() returns " << register_result << std::endl;
        hako_conductor_stop();
        return 1;
    }

    endpoint = std::make_unique<hakoniwa::pdu::Endpoint>(
        endpoint_name,
        HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    if (endpoint->open(endpoint_config_path) != HAKO_PDU_ERR_OK ||
        endpoint->start() != HAKO_PDU_ERR_OK)
    {
        std::cerr << "[ERROR] Failed to open/start endpoint: "
                  << endpoint_config_path << std::endl;
        hako_conductor_stop();
        return 1;
    }

    cmd_vel_adapter =
        std::make_unique<hako::robots::pdu::adapter::geometry_msgs::TwistPosePduAdapter>(
            *endpoint,
            hakoniwa::pdu::PduKey {asset_name, cmd_vel_pdu_name});

    std::cout << "[INFO] Starting rover twist asset with:" << std::endl;
    std::cout << "  model        : " << model_path << std::endl;
    std::cout << "  left config  : " << left_config_path << std::endl;
    std::cout << "  right config : " << right_config_path << std::endl;
    std::cout << "  pdu_def      : " << pdu_def_path << std::endl;
    std::cout << "  endpoint     : " << endpoint_config_path << std::endl;
    std::cout << "  viewer       : " << (viewer_enabled ? "enabled" : "disabled") << std::endl;
    std::cout << "[INFO] asset=" << asset_name
              << " cmd_vel=" << cmd_vel_pdu_name << std::endl;

    int start_result = 0;
    std::atomic_bool asset_thread_finished {false};
    std::thread asset_thread([&]() {
        start_result = hako_asset_start_no_wait(IsForceStop);
        asset_thread_finished.store(true);
        running.store(false);
        viewer_running.store(false);
    });

    if (viewer_enabled) {
        try {
            MujocoRenderRuntime render_runtime(
                world->getModel(),
                world->getData(),
                viewer_running,
                mujoco_mutex,
                MujocoRenderWindowMode::Visible);
            render_runtime.Run();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to run MuJoCo viewer: " << e.what() << std::endl;
            running.store(false);
            viewer_running.store(false);
        }
    } else {
        while (running.load()) {
            hako_asset_usleep(100000);
        }
    }

    const bool asset_finished_before_viewer_return = asset_thread_finished.load();
    running.store(false);
    viewer_running.store(false);
    if (asset_thread.joinable()) {
        asset_thread.join();
    }
    if (endpoint != nullptr) {
        (void)endpoint->stop();
        endpoint->close();
        endpoint.reset();
    }
    cmd_vel_adapter.reset();
    left_actuator.reset();
    right_actuator.reset();
    hako_conductor_stop();
    if (start_result != 0 && asset_finished_before_viewer_return) {
        std::cerr << "[ERROR] hako_asset_start_no_wait() returns "
                  << start_result << std::endl;
        return 1;
    }
    return 0;
}
