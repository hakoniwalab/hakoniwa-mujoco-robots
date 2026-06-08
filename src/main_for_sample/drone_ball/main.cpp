#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_debug.hpp"
#if USE_VIEWER
#include "viewer/mujoco_viewer.hpp"
#endif

#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa.hpp"
#include "physics/physics_impl.hpp"

namespace {
std::shared_ptr<hako::robots::physics::IWorld> world;
std::mutex data_mutex;
bool running_flag = true;
bool disable_conductor_start = false;
bool disable_viewer = false;

std::filesystem::path repo_root_path()
{
    const char* env = std::getenv("HAKO_DRONE_ROOT");
    if (env != nullptr && env[0] != '\0') {
        return std::filesystem::path(env).lexically_normal();
    }
    return std::filesystem::current_path().lexically_normal();
}

const std::string model_path =
    (repo_root_path() / "models/drone/drone.xml").string();
const std::string pdudef_path =
    (repo_root_path() / "models/drone/drone-pdudef-current.json").string();
const std::string endpoint_path =
    (repo_root_path() / "models/drone/endpoint/drone_ball_endpoint.json").string();
const std::string bindings_path =
    (repo_root_path() / "models/drone/mujoco-pdu-bindings.json").string();

std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

std::string resolve_repo_path(const std::string& path)
{
    const std::filesystem::path candidate(path);
    if (candidate.is_absolute()) {
        return candidate.lexically_normal().string();
    }
    return (repo_root_path() / candidate).lexically_normal().string();
}

struct DroneBallRuntimeConfig {
    std::string asset_name {};
    std::string asset_config_path {};
    std::string endpoint_config_path {};
    std::string bindings_config_path {};
    double impulse_restitution_coefficient {0.3};
    double impulse_relative_normal_speed_threshold {0.2};
    int impulse_cooldown_steps {100};
    bool disable_conductor_start {false};
    bool disable_viewer {false};
};

DroneBallRuntimeConfig load_runtime_config()
{
    DroneBallRuntimeConfig config {};
    config.asset_name = get_env_string("HAKO_ASSET_NAME", "drone_ball_sim");
    config.asset_config_path =
        resolve_repo_path(get_env_string("HAKO_ASSET_CONFIG_PATH", pdudef_path));
    config.endpoint_config_path =
        resolve_repo_path(get_env_string("HAKO_DRONE_ENDPOINT_CONFIG_PATH", endpoint_path));
    config.bindings_config_path =
        resolve_repo_path(get_env_string("HAKO_DRONE_BINDINGS_PATH", bindings_path));
    config.impulse_restitution_coefficient = 0.3;
    config.impulse_relative_normal_speed_threshold = 0.2;
    config.impulse_cooldown_steps = 100;
    config.disable_conductor_start = disable_conductor_start;
    config.disable_viewer = disable_viewer;
    return config;
}

void run_viewer_or_wait(
    mjModel* model,
    mjData* data,
    std::mutex& mutex,
    bool& running,
    bool viewer_disabled)
{
#if USE_VIEWER
    if (!viewer_disabled) {
        viewer_thread(model, data, std::ref(running), std::ref(mutex));
        return;
    }
#else
    (void)model;
    (void)data;
    (void)mutex;
    (void)viewer_disabled;
#endif
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

static int my_on_initialize(hako_asset_context_t* context) { (void)context; return 0; }
static int my_on_reset(hako_asset_context_t* context) { (void)context; return 0; }

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;

    const auto runtime = load_runtime_config();
    hakoniwa::pdu::Endpoint endpoint(runtime.asset_name, HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    if (endpoint.open(runtime.endpoint_config_path) != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to open endpoint config: "
                  << runtime.endpoint_config_path << std::endl;
        return -1;
    }

    std::vector<hakoniwa::PduBoundRigidBodyConfig> bindings;
    try {
        bindings = hakoniwa::PduBoundRigidBodyBindingsLoader::load(
            runtime.bindings_config_path,
            runtime.endpoint_config_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load bindings: " << e.what() << std::endl;
        endpoint.close();
        return -1;
    }

    std::vector<std::unique_ptr<hakoniwa::MirroredRigidBody>> mirrored_bodies;
    std::vector<std::unique_ptr<hakoniwa::ControllableRigidBody>> controllable_bodies;
    std::vector<hakoniwa::MirroredRigidBody*> mirrored_body_ptrs;
    mirrored_bodies.reserve(bindings.size());
    controllable_bodies.reserve(bindings.size());
    mirrored_body_ptrs.reserve(bindings.size());

    for (const auto& binding : bindings) {
        if (binding.type == hakoniwa::PduBoundRigidBodyType::Mirrored) {
            mirrored_bodies.push_back(
                std::make_unique<hakoniwa::MirroredRigidBody>(world, endpoint, binding));
            mirrored_body_ptrs.push_back(mirrored_bodies.back().get());
        } else {
            controllable_bodies.push_back(
                std::make_unique<hakoniwa::ControllableRigidBody>(world, endpoint, binding));
        }
    }
    hakoniwa::ImpulseDisturbanceSender::Config impulse_config {};
    impulse_config.restitution_coefficient = runtime.impulse_restitution_coefficient;
    impulse_config.relative_normal_speed_threshold =
        runtime.impulse_relative_normal_speed_threshold;
    impulse_config.cooldown_steps = runtime.impulse_cooldown_steps;
    hakoniwa::ImpulseDisturbanceSender impulse_sender(
        world,
        mirrored_body_ptrs,
        impulse_config, 
        "Ball-1");

    if (endpoint.start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to start endpoint." << std::endl;
        endpoint.close();
        return -1;
    }
    if (endpoint.post_start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to complete endpoint post_start." << std::endl;
        endpoint.stop();
        endpoint.close();
        return -1;
    }

    const double simulation_timestep = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            for (auto& body : mirrored_bodies) {
                (void)body->mirror_from_pdu();
            }
            for (auto& body : controllable_bodies) {
                (void)body->process_input_events();
            }
            world->advanceTimeStep();
            impulse_sender.emit_collision_impulses();
            for (auto& body : controllable_bodies) {
                (void)body->publish_state();
            }
        }

        hako_asset_usleep(delta_time_usec);
        auto end = std::chrono::steady_clock::now();
        const auto elapsed = end - start;
        const auto target = std::chrono::duration<double>(simulation_timestep);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }

    (void)endpoint.stop();
    endpoint.close();
    return 0;
}

static hako_asset_callbacks_t my_callback;

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> w)
{
    my_callback.on_initialize = my_on_initialize;
    my_callback.on_simulation_step = nullptr;
    my_callback.on_manual_timing_control = my_manual_timing_control;
    my_callback.on_reset = my_on_reset;

    const auto runtime = load_runtime_config();
    const hako_time_t delta_time_usec =
        static_cast<hako_time_t>(w->getModel()->opt.timestep * 1e6);
    if (!runtime.disable_conductor_start) {
        hako_conductor_start(delta_time_usec, 100000);
    } else {
        std::cout << "[INFO] Conductor start is disabled by option." << std::endl;
    }
    int ret = hako_asset_register(
        runtime.asset_name.c_str(),
        runtime.asset_config_path.c_str(),
        &my_callback,
        delta_time_usec,
        HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        std::cerr << "[ERROR] hako_asset_register() returns " << ret << std::endl;
        return;
    }
    ret = hako_asset_start();
    if (ret != 0) {
        std::cerr << "[ERROR] hako_asset_start() returns " << ret << std::endl;
        return;
    }
}
}  // namespace

int main(int argc, const char* argv[])
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--disable-conductor-start") {
            disable_conductor_start = true;
            continue;
        }
        if (arg == "--disable-viewer") {
            disable_viewer = true;
            continue;
        }
        std::cerr << "[ERROR] Unknown option: " << arg << std::endl;
        return 1;
    }

    std::cout << "[INFO] Creating drone-ball world and loading model..." << std::endl;
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
        std::cout << "[INFO] Drone-ball model loaded successfully from: " << model_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    std::thread sim_thread(simulation_thread, world);
    const auto runtime = load_runtime_config();
    run_viewer_or_wait(
        world->getModel(),
        world->getData(),
        data_mutex,
        running_flag,
        runtime.disable_viewer);

    running_flag = false;
    sim_thread.join();
    std::cout << "[INFO] Drone-ball simulation completed successfully." << std::endl;
    return 0;
}
