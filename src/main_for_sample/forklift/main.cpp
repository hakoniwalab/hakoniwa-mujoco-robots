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

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift.xml";

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world,
                       bool& running_flag,
                       std::mutex& mutex)
{
    double simulation_timestep = world->getModel()->opt.timestep;
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    hako::robots::Forklift forklift(world);
    hako::robots::PhysicsObject pallet(world, "pallet");
    hako::robots::controller::SliderController lift_ctrl(3.0, 1.5, 1.0); // kp, ki, kd

    double tread = forklift.getTreadWidth();
    hako::robots::controller::DifferentialDriveController drive_ctrl(1.5, 0.0, 0.1, tread); // kp, ki, kd, tread_width
    double target_lift_pos = 0.2; // m
    double target_v = 0.2;
    double target_w = 0.0;

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        double sim_time = world->getData()->time;
        {
            std::lock_guard<std::mutex> lock(mutex);
            double left_motor_torque = 0.0;
            double right_motor_torque = 0.0;
            drive_ctrl.update(target_v, target_w,
                              forklift.getBodyVelocity().x, forklift.getBodyAngularVelocity().z,
                              simulation_timestep,
                              left_motor_torque, right_motor_torque);
            forklift.drive_motor(left_motor_torque, right_motor_torque);
            auto lift_pos = forklift.getLiftPosition();
            if (sim_time > 10.0) {
                target_w = 0.3;
                double torque = lift_ctrl.update(lift_pos, target_lift_pos, simulation_timestep);
                std::cout << "[INFO] lift_ctrl: " << torque << std::endl;
                forklift.drive_lift(torque);
            } else {
                forklift.drive_lift(0.0);
            }

            world->advanceTimeStep();

            auto pos = forklift.getPosition();
            auto vel = forklift.getVelocity();
            auto ang = forklift.getEuler();

            std::cout << "[forklift] pos: " << pos.to_string()
                      << ", vel: " << vel.to_string()
                      << ", euler: " << ang.to_string() << std::endl;

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
