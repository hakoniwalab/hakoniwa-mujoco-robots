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
#include "hakoniwa/pdu/adapter/forklift_operation_adapter.hpp"
#include "hakoniwa/pdu/pdu.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"
#include "hako_msgs/pdu_ctype_GameControllerOperation.h"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"

//#include "hakoniwa/pdu/gamepad.hpp"
//#include "hakoniwa/pdu/msgs/geometry_msgs/Twist.hpp"
//#include "hakoniwa/pdu/msgs/std_msgs/Float64.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift.xml";
static const char* config_path = "config/safety-forklift-pdu.json";
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

    using PduT = hako::robots::pdu::PDU<
        Hako_Twist, HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>;

    // ★ 非constの実体をメンバで持つ（初期化順に注意：posより前に宣言）
    std::string robot_name_;
    std::string pdu_name_;
    PduT pos;

public:
    // ★ nameは by-value にして非constのlvalueとして使えるようにする
    HakoObject(std::string name, std::shared_ptr<hako::robots::physics::IWorld> world)
        : obj(world, name)                  // 先に PhysicsObject
        , robot_name_(name)                 // 非constの実体
        , pdu_name_("pos")                  // 非constの実体
        , pos(robot_name_, pdu_name_)       // ★ 非const lvalue をPDUに渡す
    {}

    void flush() {
        HakoCpp_Twist pos_data{};           // ゼロ初期化
        pos.load(pos_data);
        pos_data.linear.x = obj.getPosition().x;
        pos_data.linear.y = obj.getPosition().y;
        pos_data.linear.z = obj.getPosition().z;
        pos_data.angular.x = obj.getEuler().x;
        pos_data.angular.y = obj.getEuler().y;
        pos_data.angular.z = obj.getEuler().z;
        pos.flush(pos_data);
    }
};


static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;
    try {
        double simulation_timestep = world->getModel()->opt.timestep;
        hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
        std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;
        std::string robot_name = "forklift";
        std::string robot_name2 = "forklift_fork";
        std::string pdu_pad_name = "hako_cmd_game";
        std::string pdu_pos_name = "pos";
        std::string pdu_height_name = "height";
        //forklift::hako_cmd_game
        DECLARE_PDU_INSTANCE(hako::pdu::msgs::hako_msgs, GameControllerOperation, pad, robot_name, pdu_pad_name);
        //forklift::pos
        DECLARE_PDU_INSTANCE(hako::pdu::msgs::geometry_msgs, Twist, forklift_pos, robot_name, pdu_pos_name);
        //forklift::height
        DECLARE_PDU_INSTANCE(hako::pdu::msgs::std_msgs, Float64, lift_pos, robot_name, pdu_height_name);
        //forklift_fork::pos
        DECLARE_PDU_INSTANCE(hako::pdu::msgs::geometry_msgs, Twist, forklift_fork_pos, robot_name2, pdu_pos_name);

        hako::robots::controller::ForkliftController controller(world);
        HakoObject pallet1("pallet1", world);
        HakoObject pallet2("pallet2", world);
        HakoObject shelf("shelf", world);
        HakoObject cargo1("cargo1", world);
        HakoObject cargo2("cargo2", world);
        HakoObject cargo3("cargo3", world);
        HakoObject cargo4("cargo4", world);
        controller.setVelocityCommand(0.0, 0.0);
        controller.setLiftTarget(0.0);
        double delta_pos = simulation_timestep * 0.1;;
        controller.set_delta_pos(delta_pos);


        HakoCpp_Twist forklift_pos_data = {};
        HakoCpp_Twist forklift_fork_pos_data = {};
        HakoCpp_Float64 lift_pos_data = {};
        HakoCpp_GameControllerOperation pad_data = {};
        while (running_flag) {
            auto start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (pad.load(pad_data)) {
                    hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                    auto command = adapter.convert(pad_data);
                    controller.update_target_lift_z(command.lift_position);
                    controller.setVelocityCommand(command.linear_velocity, command.yaw_rate);
                }
                controller.update();
                world->advanceTimeStep();

                lift_pos.load(lift_pos_data);
                forklift_pos.load(forklift_pos_data);
                forklift_fork_pos.load(forklift_fork_pos_data);
                //flush pos of forklift
                forklift_pos_data.linear.x = controller.getForklift().getPosition().x;
                forklift_pos_data.linear.y = controller.getForklift().getPosition().y;
                forklift_pos_data.linear.z = controller.getForklift().getPosition().z;
                forklift_pos_data.angular.x = controller.getForklift().getEuler().x;
                forklift_pos_data.angular.y = controller.getForklift().getEuler().y;
                forklift_pos_data.angular.z = controller.getForklift().getEuler().z;
                forklift_pos.flush(forklift_pos_data);

                //flush pos of lift
                lift_pos_data.data = controller.getForklift().getLiftPosition().z;
                lift_pos.flush(lift_pos_data);

                //flush pos of fork
                forklift_fork_pos_data.linear.x = controller.getForklift().getLiftWorldPosition().x;
                forklift_fork_pos_data.linear.y = controller.getForklift().getLiftWorldPosition().y;
                forklift_fork_pos_data.linear.z = controller.getForklift().getLiftWorldPosition().z;
                forklift_fork_pos_data.angular.x = controller.getForklift().getLiftEuler().x;
                forklift_fork_pos_data.angular.y = controller.getForklift().getLiftEuler().y;
                forklift_fork_pos_data.angular.z = controller.getForklift().getLiftEuler().z;
                forklift_fork_pos.flush(forklift_fork_pos_data);

                //flush pos of pallet
                pallet1.flush();
                pallet2.flush();
                shelf.flush();
                cargo1.flush();
                cargo2.flush();
                cargo3.flush();
                cargo4.flush();
            }

            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double sleep_time = simulation_timestep - elapsed.count();
            hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
        }
    } catch (const std::exception& e) {
        std::fflush(stdout);
        std::cerr << "Exception in simulation thread: " << e.what() << std::endl;
        running_flag = false;
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
