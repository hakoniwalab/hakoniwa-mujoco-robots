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

#include "hakoniwa/pdu/gamepad.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift.xml";

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world,
    bool& running_flag,
    std::mutex& mutex)
{
    double simulation_timestep = world->getModel()->opt.timestep;
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    hako::robots::controller::ForkliftController controller(world);
    hako::robots::PhysicsObject pallet(world, "pallet");

    controller.setVelocityCommand(0.2, 0.0);
    controller.setLiftTarget(0.0);

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        double sim_time = world->getData()->time;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (sim_time > 10.0) {
                controller.setLiftTarget(0.2);
                controller.setVelocityCommand(0.2, 0.3);
            }
            controller.update();
            world->advanceTimeStep();

            const auto& forklift = controller.getForklift();
            std::cout << "[forklift] pos:  " << forklift.getPosition().to_string()
                << ", vel: " << forklift.getVelocity().to_string()
                << ", euler: " << forklift.getEuler().to_string() << std::endl;

            std::cout << "[lift_arm] pos: " << forklift.getLiftPosition().to_string() << std::endl;
            std::cout << "[pallet]   pos: " << pallet.getPosition().to_string() << std::endl;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double sleep_time = simulation_timestep - elapsed.count();
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
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

    std::mutex data_mutex;
    bool running_flag = true;

    std::thread sim_thread(simulation_thread, world, std::ref(running_flag), std::ref(data_mutex));

    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));

    running_flag = false;
    sim_thread.join();

    std::cout << "[INFO] Simulation completed successfully." << std::endl;
    return 0;
}
