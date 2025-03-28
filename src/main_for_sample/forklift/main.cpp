#include <mujoco/mujoco.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "mujoco_debug.hpp"
#include "mujoco_viewer.hpp"

#include "physics/physics_impl.hpp"
#include "actuator/actuator_impl.hpp"
#include "robots/forklift.hpp"
#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "controller/forklift_controller.hpp"
#include "include/hako_asset.h"
#include "include/hako_conductor.h"
#include "hakoniwa/pdu/gamepad.hpp"
#include "hakoniwa/pdu/adapter/forklift_operation_adapter.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift.xml";
static const char* config_path = "config/custom.json";
static std::mutex data_mutex;
static bool running_flag = true;
static hako_time_t delta_time_usec = 20000;

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

    double simulation_timestep = world->getModel()->opt.timestep;
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    hako::robots::controller::ForkliftController controller(world);
    hako::robots::PhysicsObject pallet(world, "pallet");

    controller.setVelocityCommand(0.0, 0.0);
    controller.setLiftTarget(0.0);
    std::string robot_name = "forklift";
    hako::robots::pdu::GamePad pad(robot_name, 0);
    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        double sim_time = world->getData()->time;

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (pad.load()) {
                hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                auto command = adapter.convert(pad);
                controller.setLiftTarget(command.lift_position);
                controller.setVelocityCommand(command.linear_velocity, command.yaw_rate);
                std::cout << "[INFO] Forklift command: "
                    << "linear_velocity=" << command.linear_velocity
                    << ", yaw_rate=" << command.yaw_rate
                    << ", lift_position=" << command.lift_position
                    << ", emergency_stop=" << command.emergency_stop
                    << std::endl;
                if (command.emergency_stop) {
                    std::cout << "[INFO] Emergency stop triggered!" << std::endl;
                }
            }
            controller.update();
            world->advanceTimeStep();

#if false
            if (sim_time > 10.0) {
                controller.setLiftTarget(0.2);
                controller.setVelocityCommand(0.2, 0.3);
            }
            const auto& forklift = controller.getForklift();
            std::cout << "[forklift] pos:  " << forklift.getPosition().to_string()
                << ", vel: " << forklift.getVelocity().to_string()
                << ", euler: " << forklift.getEuler().to_string() << std::endl;

            std::cout << "[lift_arm] pos: " << forklift.getLiftPosition().to_string() << std::endl;
            std::cout << "[pallet]   pos: " << pallet.getPosition().to_string() << std::endl;
#endif
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double sleep_time = simulation_timestep - elapsed.count();
        hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }
    
    return 0;
}
static hako_asset_callbacks_t my_callback = {
    .on_initialize = my_on_initialize,
    .on_manual_timing_control = my_manual_timing_control,
    .on_simulation_step = nullptr,
    .on_reset = my_on_reset
};
void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    const char* asset_name = "forklift";
    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(asset_name, config_path, &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_register() returns " << ret << std::endl;
        return;
    }
    double simulation_timestep = world->getModel()->opt.timestep;
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    ret = hako_asset_start();
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_start() returns " << ret << std::endl;
        return;
    }
}


int main(int argc, const char* argv[])
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

    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));

    running_flag = false;
    sim_thread.join();

    std::cout << "[INFO] Simulation completed successfully." << std::endl;
    return 0;
}
