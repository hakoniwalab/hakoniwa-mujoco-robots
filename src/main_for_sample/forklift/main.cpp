#include <mujoco/mujoco.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <mutex>
#include "mujoco_debug.hpp"
#include "mujoco_viewer.hpp"
#include "physics/physics_impl.hpp"
#include "actuator/actuator_impl.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;

static const std::string model_path = "models/forklift/forklift.xml";

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world,
    bool& running_flag,
    std::mutex& mutex)
{
    double simulation_timestep = world->getModel()->opt.timestep;  // **XMLから `timestep` を取得**
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;
    auto pallet   = world->getRigidBody("pallet");
    auto forklift = world->getRigidBody("forklift_base");
    auto lift     = world->getRigidBody("lift_arm");
    auto left_motor  = std::make_shared<hako::robots::actuator::impl::TorqueActuatorImpl>(world->getModel(), world->getData(), "left_motor");
    auto right_motor = std::make_shared<hako::robots::actuator::impl::TorqueActuatorImpl>(world->getModel(), world->getData(), "right_motor");
    auto lift_motor  = std::make_shared<hako::robots::actuator::impl::TorqueActuatorImpl>(world->getModel(), world->getData(), "lift_motor");
        
    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        double sim_time = world->getData()->time;
        {
            std::lock_guard<std::mutex> lock(mutex);
            left_motor->SetTorque(0.2);
            right_motor->SetTorque(0.2);
            if (sim_time > 10.0) {
                lift_motor->SetTorque(0.23); // フォークリフトのリフト
            } else {
                lift_motor->SetTorque(0.0); // フォークリフトのリフト
            }
            world->advanceTimeStep();
            auto pos = forklift->GetPosition();
            auto vel = forklift->GetVelocity();
            auto ang = forklift->GetEuler();
            auto lift_arm_pos = lift->GetPosition();
            auto pallet_pos = pallet->GetPosition();

            std::cout << "[forklift] pos: " << pos.to_string() << ", vel: " << vel.to_string()
                 << ", euler: " << ang.to_string() << std::endl;
            std::cout << "[lift_arm] pos: " << lift_arm_pos.to_string() << std::endl;
            std::cout << "[pallet] pos: " << pallet_pos.to_string() << std::endl;
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

    // viewer_thread に model/data を渡す
    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));

    running_flag = false;
    sim_thread.join();

    std::cout << "[INFO] Simulation completed successfully." << std::endl;
    return 0;
}
