#include "actuator.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/adapter/sensor_msgs/joint_state.hpp"
#include "hakoniwa/pdu/adapter/std_msgs/float64_multi_array.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kDefaultAssetName = "ShadowHandAsset";
constexpr const char* kDefaultModelPath =
    "thirdparty/mujoco_menagerie/shadow_hand/scene_right.xml";
constexpr const char* kDefaultJointStateConfigPath =
    "config/sensors/joint_state/shadow-hand-joint-states.json";
constexpr const char* kDefaultPduDefPath =
    "config/shadow-hand-pdudef-compact.json";
constexpr const char* kDefaultEndpointConfigPath =
    "config/endpoint/shadow_hand_endpoint.json";
constexpr const char* kDefaultEndpointName = "shadow_hand_endpoint";
constexpr const char* kDefaultCommandPduName = "actuator_position_targets";
constexpr const char* kDefaultJointStatePduName = "joint_states";

constexpr std::array<const char*, 20> kActuatorNames {
    "rh_A_WRJ2",
    "rh_A_WRJ1",
    "rh_A_THJ5",
    "rh_A_THJ4",
    "rh_A_THJ3",
    "rh_A_THJ2",
    "rh_A_THJ1",
    "rh_A_FFJ4",
    "rh_A_FFJ3",
    "rh_A_FFJ0",
    "rh_A_MFJ4",
    "rh_A_MFJ3",
    "rh_A_MFJ0",
    "rh_A_RFJ4",
    "rh_A_RFJ3",
    "rh_A_RFJ0",
    "rh_A_LFJ5",
    "rh_A_LFJ4",
    "rh_A_LFJ3",
    "rh_A_LFJ0",
};

struct CommandBinding
{
    std::shared_ptr<hako::robots::actuator::INamedActuator> actuator;
    std::string name;
    std::string target_name;
};

std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::unique_ptr<hako::robots::pdu::adapter::std_msgs::Float64MultiArrayPduAdapter>
    command_adapter;
std::unique_ptr<hako::robots::sensor::JointStateSensor> joint_state_sensor;
std::unique_ptr<hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter>
    joint_state_adapter;

std::string model_path;
std::string joint_state_config_path;
std::string pdu_def_path;
std::string endpoint_config_path;
std::string asset_name;
std::string endpoint_name;
std::string command_pdu_name;
std::string joint_state_pdu_name;
bool viewer_enabled {true};

std::atomic_bool running {true};
std::atomic_bool endpoint_ready {false};
std::atomic_bool viewer_running {true};
std::mutex mujoco_mutex;
std::vector<CommandBinding> bindings;
std::vector<double> open_targets;
std::vector<double> latest_targets;
bool command_received_once = false;

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

void PrintUsage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program
        << " [--no-viewer] [model.xml] [joint-state-config.json] [pdu-def.json] [endpoint.json]\n\n"
        << "Defaults:\n"
        << "  model.xml             " << kDefaultModelPath << "\n"
        << "  joint-state-config    " << kDefaultJointStateConfigPath << "\n"
        << "  pdu-def.json          " << kDefaultPduDefPath << "\n"
        << "  endpoint.json         " << kDefaultEndpointConfigPath << "\n\n"
        << "Environment:\n"
        << "  HAKO_SHADOW_HAND_ASSET_NAME      default " << kDefaultAssetName << "\n"
        << "  HAKO_SHADOW_HAND_ENDPOINT_NAME   default " << kDefaultEndpointName << "\n"
        << "  HAKO_SHADOW_HAND_COMMAND_PDU     default " << kDefaultCommandPduName << "\n"
        << "  HAKO_SHADOW_HAND_STATE_PDU       default " << kDefaultJointStatePduName << "\n"
        << "  HAKO_SHADOW_HAND_ENABLE_VIEWER   default 1\n";
}

std::vector<double> BuildOpenTargets(const mjModel* model)
{
    std::vector<double> out;
    out.reserve(kActuatorNames.size());
    for (const char* actuator_name : kActuatorNames) {
        const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name);
        if (actuator_id < 0) {
            throw std::runtime_error(std::string("actuator not found: ") + actuator_name);
        }
        double target = 0.0;
        if (model->actuator_ctrllimited[actuator_id]) {
            const double lo = model->actuator_ctrlrange[2 * actuator_id + 0];
            const double hi = model->actuator_ctrlrange[2 * actuator_id + 1];
            if (target < lo || target > hi) {
                target = 0.5 * (lo + hi);
            }
        }
        out.push_back(target);
    }
    return out;
}

std::vector<CommandBinding> BindActuators(
    const std::shared_ptr<hako::robots::physics::IWorld>& bind_world)
{
    std::vector<CommandBinding> out;
    out.reserve(kActuatorNames.size());
    for (const char* actuator_name : kActuatorNames) {
        hako::robots::actuator::NamedActuatorConfig config {};
        config.actuator_name = actuator_name;
        config.type = hako::robots::actuator::ActuatorType::Position;

        auto actuator = bind_world->createNamedActuator();
        if (actuator == nullptr || !actuator->BindConfig(config)) {
            throw std::runtime_error(std::string("failed to bind actuator: ") + actuator_name);
        }
        out.push_back(CommandBinding {
            actuator,
            actuator_name,
            actuator->GetBinding().target_name,
        });
    }
    return out;
}

void ResetToOpen()
{
    mj_resetData(world->getModel(), world->getData());
    latest_targets = open_targets;
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        bindings[i].actuator->SetTarget(latest_targets[i]);
    }
    mj_forward(world->getModel(), world->getData());
}

void ApplyLatestTargets()
{
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        bindings[i].actuator->SetTarget(latest_targets[i]);
    }
}

bool ReceiveCommand()
{
    std::vector<double> command;
    if (command_adapter == nullptr || !command_adapter->recv(command)) {
        return false;
    }
    if (command.size() != kActuatorNames.size()) {
        static bool warned = false;
        if (!warned) {
            std::cerr << "[WARN] ignoring actuator_position_targets with size "
                      << command.size() << ", expected " << kActuatorNames.size()
                      << std::endl;
            warned = true;
        }
        return false;
    }
    latest_targets = command;
    command_received_once = true;
    return true;
}

void PrintMapping()
{
    std::cout << "[INFO] Shadow Hand actuator command order:" << std::endl;
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        std::cout << "  [" << std::setw(2) << i << "] "
                  << bindings[i].name << " -> "
                  << bindings[i].target_name << std::endl;
    }
}

double JointPosition(const char* joint_name)
{
    const int joint_id = mj_name2id(world->getModel(), mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        return 0.0;
    }
    return world->getData()->qpos[world->getModel()->jnt_qposadr[joint_id]];
}

void PrintState()
{
    auto* data = world->getData();
    std::cout
        << std::fixed << std::setprecision(3)
        << "time=" << std::setw(7) << data->time
        << " FFJ0_target=" << latest_targets[9]
        << " FFJ2=" << JointPosition("rh_FFJ2")
        << " FFJ1=" << JointPosition("rh_FFJ1")
        << " THJ1_target=" << latest_targets[6]
        << " THJ1=" << JointPosition("rh_THJ1")
        << std::endl;
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
    ResetToOpen();
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
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(model->opt.timestep * 1.0e6);
    const double sim_timestep = model->opt.timestep;
    const int command_probe_steps =
        std::max(1, static_cast<int>(0.5 / std::max(sim_timestep, 1.0e-6)));
    int step = 0;

    std::cout << "[INFO] Shadow Hand Hakoniwa asset started." << std::endl;
    while (running.load()) {
        if (endpoint_ready.load()) {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            if (command_received_once || (step % command_probe_steps) == 0) {
                (void)ReceiveCommand();
            }
            ApplyLatestTargets();
            world->advanceTimeStep();

            if (joint_state_sensor != nullptr &&
                joint_state_adapter != nullptr &&
                joint_state_sensor->ShouldUpdate(sim_timestep))
            {
                hako::robots::sensor::JointStateFrame frame {};
                joint_state_sensor->Build(frame);
                frame.header.frame_id = "";
                frame.header.stamp_sec = world->getData()->time;
                if (!joint_state_adapter->send(frame)) {
                    std::cerr << "[WARN] Failed to send joint_states PDU." << std::endl;
                }
            }
            if ((step % 250) == 0) {
                PrintState();
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
    viewer_enabled = EnvBoolOrDefault("HAKO_SHADOW_HAND_ENABLE_VIEWER", true);
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
    joint_state_config_path = positional_args.size() > 1 ? positional_args[1] : kDefaultJointStateConfigPath;
    pdu_def_path = positional_args.size() > 2 ? positional_args[2] : kDefaultPduDefPath;
    endpoint_config_path = positional_args.size() > 3 ? positional_args[3] : kDefaultEndpointConfigPath;
    asset_name = EnvOrDefault("HAKO_SHADOW_HAND_ASSET_NAME", kDefaultAssetName);
    endpoint_name = EnvOrDefault("HAKO_SHADOW_HAND_ENDPOINT_NAME", kDefaultEndpointName);
    command_pdu_name = EnvOrDefault("HAKO_SHADOW_HAND_COMMAND_PDU", kDefaultCommandPduName);
    joint_state_pdu_name = EnvOrDefault("HAKO_SHADOW_HAND_STATE_PDU", kDefaultJointStatePduName);

    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
        bindings = BindActuators(world);
        open_targets = BuildOpenTargets(world->getModel());
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to initialize Shadow Hand model: "
                  << e.what() << std::endl;
        return 1;
    }
    latest_targets = open_targets;
    ResetToOpen();

    joint_state_sensor = std::make_unique<hako::robots::sensor::JointStateSensor>(world);
    try {
        if (!joint_state_sensor->LoadConfig(joint_state_config_path)) {
            std::cerr << "[ERROR] Failed to load joint state config: "
                      << joint_state_config_path << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load joint state config: "
                  << e.what() << std::endl;
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

    command_adapter =
        std::make_unique<hako::robots::pdu::adapter::std_msgs::Float64MultiArrayPduAdapter>(
            *endpoint,
            hakoniwa::pdu::PduKey {asset_name, command_pdu_name});
    joint_state_adapter =
        std::make_unique<hako::robots::pdu::adapter::sensor_msgs::JointStatePduAdapter>(
            *endpoint,
            hakoniwa::pdu::PduKey {asset_name, joint_state_pdu_name});

    std::cout << "[INFO] Starting Shadow Hand asset with:" << std::endl;
    std::cout << "  model           : " << model_path << std::endl;
    std::cout << "  joint state cfg : " << joint_state_config_path << std::endl;
    std::cout << "  pdu_def         : " << pdu_def_path << std::endl;
    std::cout << "  endpoint        : " << endpoint_config_path << std::endl;
    std::cout << "  viewer          : " << (viewer_enabled ? "enabled" : "disabled") << std::endl;
    std::cout << "[INFO] asset=" << asset_name
              << " command_pdu=" << command_pdu_name
              << " joint_state_pdu=" << joint_state_pdu_name << std::endl;
    PrintMapping();

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
    command_adapter.reset();
    joint_state_adapter.reset();
    joint_state_sensor.reset();
    hako_conductor_stop();
    if (start_result != 0 && asset_finished_before_viewer_return) {
        std::cerr << "[ERROR] hako_asset_start_no_wait() returns "
                  << start_result << std::endl;
        return 1;
    }
    return 0;
}
