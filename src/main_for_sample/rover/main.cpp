#include <mujoco/mujoco.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "mujoco_debug.hpp"
#if USE_VIEWER
#include "mujoco_viewer.hpp"
#endif

#include "physics/physics_impl.hpp"
#include "actuator/actuator_impl.hpp"
#include "robots/forklift.hpp"
#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "controller/forklift_controller.hpp"
#include "include/hako_asset.h"
#include "include/hako_conductor.h"
//#include "hakoniwa/pdu/gamepad.hpp"
#include "hakoniwa/pdu/adapter/forklift_operation_adapter.hpp"
//#include "hakoniwa/pdu/msgs/geometry_msgs/Twist.hpp"
//#include "hakoniwa/pdu/msgs/std_msgs/Float64.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/rover.xml";
static const char* config_path = "../../oss/hakoniwa-unity-drone/simulation/rover.json";
static std::mutex data_mutex;
static bool running_flag = true;

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
class HakoObject {
private:
    hako::robots::PhysicsObject obj;
public:
    HakoObject(const std::string& name, std::shared_ptr<hako::robots::physics::IWorld> world)
        : obj(world, name)
        {
        }

    void flush() {
        #if 0 //TODO
        auto pos = hako::pdu::msgs::geometry_msgs::Twist(obj.getModelName(), 0);
        HakoCpp_Twist& pos_data = pos.getData();
        pos_data.linear.x = obj.getPosition().x;
        pos_data.linear.y = obj.getPosition().y;
        pos_data.linear.z = obj.getPosition().z;
        pos_data.angular.x = obj.getEuler().x;
        pos_data.angular.y = obj.getEuler().y;
        pos_data.angular.z = obj.getEuler().z;
        pos.flush();
        #endif
    }
};

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;

    double simulation_timestep = world->getModel()->opt.timestep;
    hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
    std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

    hako::robots::controller::ForkliftController controller(world);
    controller.setVelocityCommand(0.0, 0.0);
    controller.setLiftTarget(0.0);
    std::string robot_name = "Rover";
    double delta_pos = simulation_timestep * 0.1;;
    controller.set_delta_pos(delta_pos);

    //hako::robots::pdu::GamePad pad(robot_name, 0);
    //TODO
    #if 0
    hako::pdu::msgs::geometry_msgs::Twist forklift_pos(robot_name, 1);
    hako::pdu::msgs::std_msgs::Float64 lift_pos(robot_name, 2);
    HakoCpp_Twist& forklift_pos_data = forklift_pos.getData();
    HakoCpp_Float64& lift_pos_data = lift_pos.getData();
    #endif
    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            #if 0
            if (pad.load()) {
                hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                adapter.setAxisYawIndex(0);
                auto command = adapter.convert(pad);
                controller.update_target_lift_z(command.lift_position);
                controller.setVelocityCommand(command.linear_velocity, command.yaw_rate);
            }
            #endif
            controller.update();
            world->advanceTimeStep();

            //TODO
            #if 0
            //flush pos of forklift
            forklift_pos_data.linear.x = controller.getForklift().getPosition().x;
            forklift_pos_data.linear.y = controller.getForklift().getPosition().y;
            forklift_pos_data.linear.z = controller.getForklift().getPosition().z;
            forklift_pos_data.angular.x = controller.getForklift().getEuler().x;
            forklift_pos_data.angular.y = controller.getForklift().getEuler().y;
            forklift_pos_data.angular.z = controller.getForklift().getEuler().z;
            forklift_pos.flush();

            //flush pos of lift
            lift_pos_data.data = controller.getForklift().getLiftPosition().z;
            lift_pos.flush();
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
static hako_asset_callbacks_t my_callback;
void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world)
{
     my_callback.on_initialize = my_on_initialize;
     my_callback.on_simulation_step = nullptr;
     my_callback.on_manual_timing_control = my_manual_timing_control;
     my_callback.on_reset = my_on_reset;
     
    const char* asset_name = "forklift";
    double simulation_timestep = world->getModel()->opt.timestep;
    hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(asset_name, config_path, &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
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

#if USE_VIEWER
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
