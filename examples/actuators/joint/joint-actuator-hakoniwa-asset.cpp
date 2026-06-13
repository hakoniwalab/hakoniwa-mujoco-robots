#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/adapter/std_msgs/float64.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultAssetName = "JointActuatorAsset";
constexpr const char* kDefaultModelPath =
    "models/actuators/joint/position-velocity-actuator-sample.xml";
constexpr const char* kDefaultPositionConfigPath =
    "config/actuator/joint/sample_position_actuator.json";
constexpr const char* kDefaultVelocityConfigPath =
    "config/actuator/joint/sample_velocity_actuator.json";
constexpr const char* kDefaultPduDefPath =
    "config/joint-actuator-pdudef-compact.json";
constexpr const char* kDefaultEndpointConfigPath =
    "config/endpoint/joint_actuator_endpoint.json";
constexpr const char* kDefaultEndpointName = "joint_actuator_endpoint";

std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::unique_ptr<hakoniwa::pdu::Endpoint> endpoint;
std::shared_ptr<hako::robots::actuator::IJointActuator> position_actuator;
std::shared_ptr<hako::robots::actuator::IJointActuator> velocity_actuator;
std::unique_ptr<hako::robots::pdu::adapter::std_msgs::Float64PduAdapter>
    position_adapter;
std::unique_ptr<hako::robots::pdu::adapter::std_msgs::Float64PduAdapter>
    velocity_adapter;

std::string model_path;
std::string position_config_path;
std::string velocity_config_path;
std::string pdu_def_path;
std::string endpoint_config_path;
std::string asset_name;
std::string endpoint_name;

std::atomic_bool running {true};
std::atomic_bool viewer_running {true};
std::atomic_bool endpoint_ready {false};
std::atomic_bool paused {false};
std::atomic_bool reset_requested {false};
std::atomic_bool print_help_requested {false};
std::mutex mujoco_mutex;

int position_qpos_addr = -1;
int velocity_qvel_addr = -1;

bool RecvAndApplyJointCommand(
    hako::robots::pdu::adapter::std_msgs::Float64PduAdapter& adapter,
    hako::robots::actuator::IJointActuator& actuator)
{
    double target = 0.0;
    if (!adapter.recv(target)) {
        return false;
    }
    actuator.SetTarget(target);
    return true;
}

std::string EnvOrDefault(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0') {
        return value;
    }
    return fallback;
}

int FindJointQposAddr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        return -1;
    }
    return model->jnt_qposadr[joint_id];
}

int FindJointQvelAddr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        return -1;
    }
    return model->jnt_dofadr[joint_id];
}

void PrintUsage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program
        << " [model.xml] [position-config.json] [velocity-config.json] "
           "[pdu-def.json] [endpoint.json]\n\n"
        << "Defaults:\n"
        << "  model.xml             " << kDefaultModelPath << "\n"
        << "  position-config.json  " << kDefaultPositionConfigPath << "\n"
        << "  velocity-config.json  " << kDefaultVelocityConfigPath << "\n"
        << "  pdu-def.json          " << kDefaultPduDefPath << "\n"
        << "  endpoint.json         " << kDefaultEndpointConfigPath << "\n\n"
        << "Environment:\n"
        << "  HAKO_JOINT_ACTUATOR_ASSET_NAME     PDU robot/asset name, default "
        << kDefaultAssetName << "\n"
        << "  HAKO_JOINT_ACTUATOR_ENDPOINT_NAME  endpoint name, default "
        << kDefaultEndpointName << "\n";
}

void PrintHelp()
{
    std::cout << R"(
Controls:
  p      : pause / resume MuJoCo stepping
  r      : reset MuJoCo simulation state
  h      : show help
  q / Esc: quit asset and viewer

PDU:
  The Python sender writes std_msgs/Float64 commands to:
    JointActuatorAsset/position_target
    JointActuatorAsset/velocity_target
)" << std::endl;
}

void PrintState()
{
    auto* data = world->getData();
    if (data == nullptr || position_qpos_addr < 0 || velocity_qvel_addr < 0) {
        return;
    }
    std::cout
        << std::fixed << std::setprecision(3)
        << "time=" << std::setw(6) << data->time
        << " pos_angle=" << std::setw(7) << data->qpos[position_qpos_addr]
        << " vel_qvel=" << std::setw(7) << data->qvel[velocity_qvel_addr]
        << std::endl;
}

void ResetSimulation()
{
    auto* model = world->getModel();
    auto* data = world->getData();
    mj_resetData(model, data);
    mj_forward(model, data);
    std::cout << "[INFO] reset" << std::endl;
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
    ResetSimulation();
    return 0;
}

int IsForceStop()
{
    return running.load() && viewer_running.load() ? 0 : 1;
}

static int OnManualTimingControl(hako_asset_context_t* context)
{
    (void)context;
    auto* model = world->getModel();
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(model->opt.timestep * 1.0e6);
    const double sim_timestep = model->opt.timestep;
    int step = 0;

    std::cout << "[INFO] Joint Actuator Hakoniwa asset started." << std::endl;
    PrintHelp();

    while (running.load() && viewer_running.load()) {
        if (endpoint_ready.load()) {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            if (reset_requested.exchange(false)) {
                ResetSimulation();
            }
            if (position_actuator->ShouldUpdate(sim_timestep)) {
                (void)RecvAndApplyJointCommand(*position_adapter, *position_actuator);
            }
            if (velocity_actuator->ShouldUpdate(sim_timestep)) {
                (void)RecvAndApplyJointCommand(*velocity_adapter, *velocity_actuator);
            }
            if (!paused.load()) {
                world->advanceTimeStep();
            }
            if ((step % 100) == 0) {
                PrintState();
            }
            ++step;
        }

        if (print_help_requested.exchange(false)) {
            PrintHelp();
        }
        hako_asset_usleep(delta_time_usec);
    }

    return 0;
}

void HandleViewerKey(int key, int action)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }
    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        running.store(false);
        viewer_running.store(false);
    } else if (key == GLFW_KEY_P) {
        paused.store(!paused.load());
        std::cout << "[INFO] " << (paused.load() ? "paused" : "resumed") << std::endl;
    } else if (key == GLFW_KEY_R) {
        reset_requested.store(true);
    } else if (key == GLFW_KEY_H) {
        print_help_requested.store(true);
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage(argv[0]);
        return 0;
    }

    model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    position_config_path = argc > 2 ? argv[2] : kDefaultPositionConfigPath;
    velocity_config_path = argc > 3 ? argv[3] : kDefaultVelocityConfigPath;
    pdu_def_path = argc > 4 ? argv[4] : kDefaultPduDefPath;
    endpoint_config_path = argc > 5 ? argv[5] : kDefaultEndpointConfigPath;
    asset_name =
        EnvOrDefault("HAKO_JOINT_ACTUATOR_ASSET_NAME", kDefaultAssetName);
    endpoint_name =
        EnvOrDefault("HAKO_JOINT_ACTUATOR_ENDPOINT_NAME", kDefaultEndpointName);

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

    position_actuator = world->createJointActuator();
    velocity_actuator = world->createJointActuator();
    if (!position_actuator->LoadConfig(position_config_path)) {
        std::cerr << "[ERROR] Failed to load position actuator config: "
                  << position_config_path << std::endl;
        return 1;
    }
    if (!velocity_actuator->LoadConfig(velocity_config_path)) {
        std::cerr << "[ERROR] Failed to load velocity actuator config: "
                  << velocity_config_path << std::endl;
        return 1;
    }

    position_qpos_addr =
        FindJointQposAddr(model, position_actuator->GetConfig().joint_name.c_str());
    velocity_qvel_addr =
        FindJointQvelAddr(model, velocity_actuator->GetConfig().joint_name.c_str());

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

    const std::string position_pdu_name =
        position_actuator->GetConfig().pdu_config.pdu_name;
    const std::string velocity_pdu_name =
        velocity_actuator->GetConfig().pdu_config.pdu_name;
    const hakoniwa::pdu::PduKey position_key {asset_name, position_pdu_name};
    const hakoniwa::pdu::PduKey velocity_key {asset_name, velocity_pdu_name};
    position_adapter =
        std::make_unique<hako::robots::pdu::adapter::std_msgs::Float64PduAdapter>(
            *endpoint,
            position_key);
    velocity_adapter =
        std::make_unique<hako::robots::pdu::adapter::std_msgs::Float64PduAdapter>(
            *endpoint,
            velocity_key);

    std::cout << "[INFO] Starting joint actuator asset with:" << std::endl;
    std::cout << "  model           : " << model_path << std::endl;
    std::cout << "  position config : " << position_config_path << std::endl;
    std::cout << "  velocity config : " << velocity_config_path << std::endl;
    std::cout << "  pdu_def         : " << pdu_def_path << std::endl;
    std::cout << "  endpoint        : " << endpoint_config_path << std::endl;
    std::cout << "[INFO] asset=" << asset_name
              << " position_pdu=" << position_pdu_name
              << " velocity_pdu=" << velocity_pdu_name
              << " position_rate_hz="
              << position_actuator->GetConfig().pdu_config.update_rate_hz
              << " velocity_rate_hz="
              << velocity_actuator->GetConfig().pdu_config.update_rate_hz
              << std::endl;

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
    render_runtime.SetKeyCallback(
        [&](int key, int action, int mods) {
            (void)mods;
            HandleViewerKey(key, action);
        });
    render_runtime.Run();

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
    position_adapter.reset();
    velocity_adapter.reset();
    position_actuator.reset();
    velocity_actuator.reset();
    hako_conductor_stop();
    if (start_result != 0 && asset_finished_before_viewer_return) {
        std::cerr << "[ERROR] hako_asset_start_no_wait() returns "
                  << start_result << std::endl;
        return 1;
    }
    return 0;
}
