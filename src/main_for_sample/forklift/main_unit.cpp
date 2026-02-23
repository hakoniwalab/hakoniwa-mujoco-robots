#include <mujoco/mujoco.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <stdexcept>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdint>

#include "mujoco_debug.hpp"
#include "mujoco_viewer.hpp"

#include "physics/physics_impl.hpp"
#include "actuator/actuator_impl.hpp"
#include "robots/forklift.hpp"
#include "controller/slider_controller.hpp"
#include "controller/differential_drive_controller.hpp"
#include "controller/forklift_controller.hpp"
#include "hako_asset.h"
#include "hako_asset_pdu.hpp"
#include "hako_conductor.h"
#include "hakoniwa/pdu/adapter/forklift_operation_adapter.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"
#include "std_msgs/pdu_cpptype_conv_Int32.hpp"
#include "hako_msgs/pdu_ctype_GameControllerOperation.h"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "hakoniwa_mujoco_context.hpp"

std::shared_ptr<hako::robots::physics::IWorld> world;
static const std::string model_path = "models/forklift/forklift-unit.xml";
static const char* config_path = "config/forklift-unit-compact.json";
static std::mutex data_mutex;
static bool running_flag = true;

static std::string now_local_time_string()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tmv {};
#if defined(_WIN32)
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tmv, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

static double get_motion_gain()
{
    const double default_gain = 0.2;
    const char* env = std::getenv("HAKO_FORKLIFT_MOTION_GAIN");
    if (env == nullptr) {
        return default_gain;
    }
    try {
        double v = std::stod(env);
        if (v > 0.0) {
            return v;
        }
    } catch (...) {
    }
    return default_gain;
}

static bool resolve_pdu_info(
    const std::string& robot_name,
    const std::string& pdu_name,
    int& pdu_size,
    int& channel_id)
{
    std::vector<hako::asset::Robot> robots;
    if (!hako::asset::hako_asset_get_pdus(robots)) {
        std::cerr << "[ERROR] Failed to get PDU configuration" << std::endl;
        return false;
    }
    for (const auto& robot : robots) {
        if (robot.name != robot_name) {
            continue;
        }
        for (const auto& writer : robot.pdu_writers) {
            if (writer.org_name == pdu_name) {
                pdu_size = writer.pdu_size;
                channel_id = writer.channel_id;
                return true;
            }
        }
        for (const auto& reader : robot.pdu_readers) {
            if (reader.org_name == pdu_name) {
                pdu_size = reader.pdu_size;
                channel_id = reader.channel_id;
                return true;
            }
        }
    }
    return false;
}

template <typename CppType, typename Convertor>
class PduChannel {
private:
    std::string robot_name_;
    int channel_id_;
    int pdu_size_;
    Convertor convertor_;
    std::vector<char> buffer_;

public:
    PduChannel(const std::string& robot_name, const std::string& pdu_name)
        : robot_name_(robot_name), channel_id_(-1), pdu_size_(0)
    {
        if (!resolve_pdu_info(robot_name, pdu_name, pdu_size_, channel_id_)) {
            throw std::runtime_error(
                "PDU not found: robot=" + robot_name + " pdu=" + pdu_name);
        }
        buffer_.resize(static_cast<size_t>(pdu_size_));
    }

    bool load(CppType& data)
    {
        if (hako_asset_pdu_read(robot_name_.c_str(), channel_id_, buffer_.data(), buffer_.size()) != 0) {
            return false;
        }
        auto* meta = reinterpret_cast<const HakoPduMetaDataType*>(buffer_.data());
        if (HAKO_PDU_METADATA_IS_INVALID(meta)) {
            return false;
        }
        if (hako_get_base_ptr_pdu(static_cast<void*>(buffer_.data())) == nullptr) {
            return false;
        }
        return convertor_.pdu2cpp(buffer_.data(), data);
    }

    bool flush(CppType& data)
    {
        int actual_size = convertor_.cpp2pdu(data, buffer_.data(), static_cast<int>(buffer_.size()));
        if (actual_size <= 0 || actual_size > static_cast<int>(buffer_.size())) {
            return false;
        }
        return hako_asset_pdu_write(robot_name_.c_str(), channel_id_, buffer_.data(), static_cast<size_t>(actual_size)) == 0;
    }
};

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
    try {
        double simulation_timestep = world->getModel()->opt.timestep;
        hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
        std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

        std::string robot_name = "forklift";
        std::string robot_name2 = "forklift_fork";

        PduChannel<HakoCpp_GameControllerOperation, hako::pdu::msgs::hako_msgs::GameControllerOperation> pad(robot_name, "hako_cmd_game");
        PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_pos(robot_name, "pos");
        PduChannel<HakoCpp_Float64, hako::pdu::msgs::std_msgs::Float64> lift_pos(robot_name, "height");
        PduChannel<HakoCpp_Int32, hako::pdu::msgs::std_msgs::Int32> phase_pos(robot_name, "phase");
        PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_fork_pos(robot_name2, "pos");

        hako::robots::controller::ForkliftController controller(world);
        controller.setVelocityCommand(0.0, 0.0);
        controller.setLiftTarget(0.0);
        controller.set_delta_pos(simulation_timestep * get_motion_gain());
        HakoniwaMujocoContext mujoco_ctx(world, "./tmp/hakoniwa-forklift-unit.state");
        std::filesystem::create_directories("./logs");
        std::ofstream recovery_log("./logs/forklift-unit-recovery.log", std::ios::app);
        recovery_log << std::fixed << std::setprecision(6);
        HakoniwaMujocoContext::ForkliftState loaded_state;
        HakoniwaMujocoContext::ControlState control_state;
        bool restored = mujoco_ctx.restore_forklift_state(&loaded_state, &control_state);
        if (restored) {
            controller.setLiftTarget(loaded_state.lift_qpos);
            controller.setVelocityCommand(control_state.target_linear_velocity, control_state.target_yaw_rate);
            std::cout << "[INFO] Resume forklift state from: " << mujoco_ctx.state_file_path() << std::endl;
            std::cout << "[INFO] Resume control phase=" << control_state.phase
                      << " target(v,yaw,lift)=("
                      << control_state.target_linear_velocity << ", "
                      << control_state.target_yaw_rate << ", "
                      << control_state.target_lift_z << ")"
                      << " step=" << control_state.sim_step << std::endl;
        }
        const auto initial_pos = controller.getForklift().getPosition();
        const auto initial_euler = controller.getForklift().getEuler();
        const auto initial_lift = controller.getForklift().getLiftPosition();
        recovery_log
            << "[" << now_local_time_string() << "] START"
            << " restored=" << (restored ? "yes" : "no")
            << " state_file=" << mujoco_ctx.state_file_path()
            << " pos=(" << initial_pos.x << "," << initial_pos.y << "," << initial_pos.z << ")"
            << " euler=(" << initial_euler.x << "," << initial_euler.y << "," << initial_euler.z << ")"
            << " lift_z=" << initial_lift.z
            << " phase=" << control_state.phase
            << " target_v=" << control_state.target_linear_velocity
            << " target_yaw=" << control_state.target_yaw_rate
            << " target_lift=" << control_state.target_lift_z
            << " step=" << control_state.sim_step
            << std::endl;

        HakoCpp_Twist forklift_pos_data = {};
        HakoCpp_Twist forklift_fork_pos_data = {};
        HakoCpp_Float64 lift_pos_data = {};
        HakoCpp_Int32 phase_pos_data = {};
        HakoCpp_GameControllerOperation pad_data = {};
        int step_count = 0;

        while (running_flag) {
            auto start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (pad.load(pad_data)) {
                    hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                    auto command = adapter.convert(pad_data);
                    control_state.target_linear_velocity = command.linear_velocity;
                    control_state.target_yaw_rate = command.yaw_rate;
                    controller.update_target_lift_z(command.lift_position);
                    controller.setVelocityCommand(command.linear_velocity, command.yaw_rate);
                    const double kVelEps = 1e-4;
                    if (control_state.phase <= 0) {
                        if (command.linear_velocity > kVelEps) {
                            control_state.phase = 1;
                        } else if (command.linear_velocity < -kVelEps) {
                            control_state.phase = 2;
                        } else if (std::abs(command.yaw_rate) > 1e-6 || std::abs(command.lift_position) > 1e-6) {
                            control_state.phase = 1;
                        }
                    } else if (control_state.phase == 1) {
                        if (command.linear_velocity < -kVelEps) {
                            control_state.phase = 2;
                        }
                    } else {
                        // phase=2 is latched during one simulation session.
                        control_state.phase = 2;
                    }
                }

                controller.update();
                world->advanceTimeStep();

                forklift_pos_data.linear.x = controller.getForklift().getPosition().x;
                forklift_pos_data.linear.y = controller.getForklift().getPosition().y;
                forklift_pos_data.linear.z = controller.getForklift().getPosition().z;
                forklift_pos_data.angular.x = controller.getForklift().getEuler().x;
                forklift_pos_data.angular.y = controller.getForklift().getEuler().y;
                forklift_pos_data.angular.z = controller.getForklift().getEuler().z;
                (void)forklift_pos.flush(forklift_pos_data);

                lift_pos_data.data = controller.getForklift().getLiftPosition().z;
                (void)lift_pos.flush(lift_pos_data);
                control_state.target_lift_z = lift_pos_data.data;
                phase_pos_data.data = static_cast<int32_t>(control_state.phase);
                (void)phase_pos.flush(phase_pos_data);

                forklift_fork_pos_data.linear.x = controller.getForklift().getLiftWorldPosition().x;
                forklift_fork_pos_data.linear.y = controller.getForklift().getLiftWorldPosition().y;
                forklift_fork_pos_data.linear.z = controller.getForklift().getLiftWorldPosition().z;
                forklift_fork_pos_data.angular.x = controller.getForklift().getLiftEuler().x;
                forklift_fork_pos_data.angular.y = controller.getForklift().getLiftEuler().y;
                forklift_fork_pos_data.angular.z = controller.getForklift().getLiftEuler().z;
                (void)forklift_fork_pos.flush(forklift_fork_pos_data);

                step_count++;
                control_state.sim_step = static_cast<std::uint64_t>(step_count);
                if (mujoco_ctx.should_autosave(step_count)) {
                    (void)mujoco_ctx.save_forklift_state_with_control(&control_state);
                    const auto p = controller.getForklift().getPosition();
                    const auto e = controller.getForklift().getEuler();
                    const auto l = controller.getForklift().getLiftPosition();
                    recovery_log
                        << "[" << now_local_time_string() << "] AUTOSAVE"
                        << " step=" << control_state.sim_step
                        << " pos=(" << p.x << "," << p.y << "," << p.z << ")"
                        << " euler=(" << e.x << "," << e.y << "," << e.z << ")"
                        << " lift_z=" << l.z
                        << " phase=" << control_state.phase
                        << std::endl;
                }
            }

            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double sleep_time = simulation_timestep - elapsed.count();
            hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
        }
        (void)mujoco_ctx.save_forklift_state_with_control(&control_state);
        std::cout << "[INFO] Saved forklift state to: " << mujoco_ctx.state_file_path() << std::endl;
        const auto final_pos = controller.getForklift().getPosition();
        const auto final_euler = controller.getForklift().getEuler();
        const auto final_lift = controller.getForklift().getLiftPosition();
        recovery_log
            << "[" << now_local_time_string() << "] END"
            << " saved=yes"
            << " state_file=" << mujoco_ctx.state_file_path()
            << " pos=(" << final_pos.x << "," << final_pos.y << "," << final_pos.z << ")"
            << " euler=(" << final_euler.x << "," << final_euler.y << "," << final_euler.z << ")"
            << " lift_z=" << final_lift.z
            << " phase=" << control_state.phase
            << " target_v=" << control_state.target_linear_velocity
            << " target_yaw=" << control_state.target_yaw_rate
            << " target_lift=" << control_state.target_lift_z
            << " step=" << control_state.sim_step
            << std::endl;
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
