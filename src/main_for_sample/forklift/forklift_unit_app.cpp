#include <mujoco/mujoco.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <cstdlib>
#include <memory>

#include "mujoco_debug.hpp"
#include "mujoco_viewer.hpp"

#include "physics/physics_impl.hpp"
#include "actuator/actuator_impl.hpp"
#include "robots/forklift.hpp"
#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "forklift_unit_app.hpp"
#include "rd_light_integration.hpp"
#include "forklift_simulation_loop.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift-unit.xml";
static const char* config_path = "config/forklift-unit-compact.json";
static std::mutex data_mutex;
static bool running_flag = true;

static std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

static int my_on_initialize(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

static int my_on_reset(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;
    ForkliftSimulationLoop loop(world, data_mutex, running_flag);
    return loop.run();
}

static hako_asset_callbacks_t my_callback;

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    my_callback.on_initialize = my_on_initialize;
    my_callback.on_simulation_step = nullptr;
    my_callback.on_manual_timing_control = my_manual_timing_control;
    my_callback.on_reset = my_on_reset;

    const std::string asset_name = get_env_string("HAKO_ASSET_NAME", "forklift");
    const std::string config_path_env = get_env_string("HAKO_ASSET_CONFIG_PATH", config_path);
    const bool rd_lite_enabled = RdLightIntegration::is_enabled_from_env();
    const bool rd_lite_initial_owner = RdLightIntegration::is_initial_owner_from_env();
    double simulation_timestep = world->getModel()->opt.timestep;
    hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
    if (!rd_lite_enabled || rd_lite_initial_owner) {
        hako_conductor_start(delta_time_usec, 100000);
    } else {
        std::cout << "[INFO] RD-lite standby mode: skip hako_conductor_start()" << std::endl;
    }
    int ret = hako_asset_register(asset_name.c_str(), config_path_env.c_str(), &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
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

ForkliftUnitApplication::ForkliftUnitApplication() = default;
ForkliftUnitApplication::~ForkliftUnitApplication() = default;

int ForkliftUnitApplication::run(int argc, const char* argv[])
{
    (void)argc;
    (void)argv;

    std::cout << "[INFO] Creating world and loading model..." << std::endl;
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << e.what() << std::endl;
        return 1;
    }

    std::thread sim_thread(simulation_thread, world);

#if USE_VIEWER
    std::cout << "[INFO] Starting viewer..." << std::endl;
    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));
#else
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[INFO] Simulation thread finished." << std::endl;
#endif

    running_flag = false;
    sim_thread.join();
    std::cout << "[INFO] Simulation completed successfully." << std::endl;
    return 0;
}
