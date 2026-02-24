#include "forklift_simulation_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "controller/forklift_controller.hpp"
#include "hako_asset.h"
#include "hakoniwa/pdu/adapter/forklift_operation_adapter.hpp"
#include "hako_msgs/pdu_ctype_GameControllerOperation.h"
#include "hakoniwa_mujoco_context.hpp"
#include "rd_light_integration.hpp"
#include "forklift_visibility_controller.hpp"
#include "forklift_trace_logger.hpp"
#include "forklift_recovery_logger.hpp"
#include "forklift_pdu_runtime.hpp"

namespace {
std::string now_local_time_string()
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

int get_env_int(const char* name, int default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stoi(env);
    } catch (...) {
        return default_value;
    }
}

double get_env_double(const char* name, double default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stod(env);
    } catch (...) {
        return default_value;
    }
}

std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

double get_motion_gain()
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

bool read_file_bytes(const std::string& path, std::vector<std::uint8_t>& out)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.good()) {
        return false;
    }
    ifs.seekg(0, std::ios::end);
    std::streamoff n = ifs.tellg();
    if (n < 0) {
        return false;
    }
    ifs.seekg(0, std::ios::beg);
    out.resize(static_cast<std::size_t>(n));
    if (n == 0) {
        return true;
    }
    ifs.read(reinterpret_cast<char*>(out.data()), n);
    return ifs.good();
}

bool write_file_bytes(const std::string& path, const std::vector<std::uint8_t>& data)
{
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs.good()) {
        return false;
    }
    if (!data.empty()) {
        ofs.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    }
    return ofs.good();
}

void apply_restored_control_state(
    hako::robots::controller::ForkliftController& controller,
    const HakoniwaMujocoContext::ControlState& control_state)
{
    hako::robots::controller::ForkliftController::InternalState internal {};
    internal.target_lift_z = control_state.target_lift_z;
    internal.target_linear_vel = control_state.target_linear_velocity;
    internal.target_yaw_rate = control_state.target_yaw_rate;
    internal.lift_pid.integral = control_state.lift_pid_integral;
    internal.lift_pid.prev_error = control_state.lift_pid_prev_error;
    internal.drive_v_pid.integral = control_state.drive_v_pid_integral;
    internal.drive_v_pid.prev_error = control_state.drive_v_pid_prev_error;
    internal.drive_w_pid.integral = control_state.drive_w_pid_integral;
    internal.drive_w_pid.prev_error = control_state.drive_w_pid_prev_error;
    controller.set_internal_state(internal);
}
} // namespace

ForkliftSimulationLoop::ForkliftSimulationLoop(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    std::mutex& data_mutex,
    bool& running_flag)
    : world_(std::move(world))
    , data_mutex_(data_mutex)
    , running_flag_(running_flag)
{
}

int ForkliftSimulationLoop::run()
{
    try {
        double simulation_timestep = world_->getModel()->opt.timestep;
        hako_time_t delta_time_usec = static_cast<hako_time_t>(simulation_timestep * 1e6);
        std::cout << "[INFO] Simulation timestep: " << simulation_timestep << " sec" << std::endl;

        std::string asset_name = get_env_string("HAKO_ASSET_NAME", "forklift");
        std::string robot_name = get_env_string("HAKO_FORKLIFT_ROBOT_NAME", asset_name);
        std::string robot_name2 = "forklift_fork";
        ForkliftPduRuntime pdu_runtime(robot_name, robot_name2);

        ForkliftVisibilityController visibility;
        (void)visibility.initialize(world_->getModel(), world_->getData(), "forklift_base");
        visibility.set_standby_alpha(static_cast<float>(get_env_double("HAKO_RD_LITE_STANDBY_ALPHA", 0.2)));
        visibility.set_standby_tint(static_cast<float>(get_env_double("HAKO_RD_LITE_STANDBY_TINT", 0.35)));
        visibility.set_standby_weld_enabled(get_env_int("HAKO_RD_LITE_STANDBY_WELD", 0) != 0);
        visibility.set_standby_zero_dynamics(get_env_int("HAKO_RD_LITE_STANDBY_ZERO_DYNAMICS", 1) != 0);

        hako::robots::controller::ForkliftController controller(world_);
        controller.setVelocityCommand(0.0, 0.0);
        controller.setLiftTarget(0.0);
        controller.set_delta_pos(simulation_timestep * get_motion_gain());
        HakoniwaMujocoContext mujoco_ctx(world_, "./tmp/hakoniwa-forklift-unit.state");
        const bool rd_lite_enabled = (get_env_int("HAKO_RD_LITE_ENABLE", 0) != 0);
        const bool local_state_enabled = (get_env_int("HAKO_LOCAL_STATE_ENABLE", rd_lite_enabled ? 0 : 1) != 0);
        const std::string session_ts = now_local_time_string();
        ForkliftRecoveryLogger recovery_logger;
        ForkliftTraceLogger trace_logger(session_ts);
        HakoniwaMujocoContext::ForkliftState loaded_state;
        HakoniwaMujocoContext::ControlState control_state;
        bool restored = false;
        if (local_state_enabled) {
            restored = mujoco_ctx.restore_forklift_state(&loaded_state, &control_state);
        } else {
            std::cout << "[INFO] Local state restore is disabled (HAKO_LOCAL_STATE_ENABLE=0)." << std::endl;
        }
        if (restored) {
            apply_restored_control_state(controller, control_state);
            std::cout << "[INFO] Resume forklift state from: " << mujoco_ctx.state_file_path() << std::endl;
            std::cout << "[INFO] Resume control phase=" << control_state.phase
                      << " target(v,yaw,lift)=("
                      << control_state.target_linear_velocity << ", "
                      << control_state.target_yaw_rate << ", "
                      << control_state.target_lift_z << ")"
                      << " step=" << control_state.sim_step << std::endl;
            std::cout << "[INFO] Restore PID policy: restore-all-pid" << std::endl;
        }
        const auto initial_pos = controller.getForklift().getPosition();
        const auto initial_euler = controller.getForklift().getEuler();
        const auto initial_lift = controller.getForklift().getLiftPosition();
        {
            ForkliftRecoverySample s {};
            s.restored = restored;
            s.state_file = mujoco_ctx.state_file_path();
            s.pos_x = initial_pos.x;
            s.pos_y = initial_pos.y;
            s.pos_z = initial_pos.z;
            s.euler_x = initial_euler.x;
            s.euler_y = initial_euler.y;
            s.euler_z = initial_euler.z;
            s.lift_z = initial_lift.z;
            s.phase = control_state.phase;
            s.target_v = control_state.target_linear_velocity;
            s.target_yaw = control_state.target_yaw_rate;
            s.target_lift = control_state.target_lift_z;
            s.step = control_state.sim_step;
            recovery_logger.log_start(now_local_time_string(), s);
        }

        HakoCpp_GameControllerOperation pad_data = {};
        int step_count = 0;
        if (restored && control_state.sim_step > 0) {
            const auto max_int = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
            step_count = static_cast<int>(std::min(control_state.sim_step, max_int));
        }
        int resumed_step_base = step_count;
        auto rd_save_context_payload = [&](std::vector<std::uint8_t>& out_bytes) -> bool {
            if (!mujoco_ctx.save_forklift_state_with_control(&control_state)) {
                return false;
            }
            return read_file_bytes(mujoco_ctx.state_file_path(), out_bytes);
        };
        auto rd_restore_context_payload = [&](const std::vector<std::uint8_t>& in_bytes) -> bool {
            if (!write_file_bytes(mujoco_ctx.state_file_path(), in_bytes)) {
                return false;
            }
            HakoniwaMujocoContext::ForkliftState restored_state {};
            HakoniwaMujocoContext::ControlState restored_control {};
            if (!mujoco_ctx.restore_forklift_state(&restored_state, &restored_control)) {
                return false;
            }
            loaded_state = restored_state;
            control_state = restored_control;
            apply_restored_control_state(controller, control_state);
            const auto max_int = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
            step_count = static_cast<int>(std::min(control_state.sim_step, max_int));
            resumed_step_base = step_count;
            restored = true;
            return true;
        };
        RdLightIntegration rd_integration;
        (void)rd_integration.initialize(robot_name, rd_save_context_payload, rd_restore_context_payload);
        bool prev_allow_step = rd_integration.is_local_owner();

        while (running_flag_) {
            auto start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                const double sim_time_sec = static_cast<double>(step_count) * simulation_timestep;
                const double sim_time_sec_from_resume =
                    static_cast<double>(step_count - resumed_step_base) * simulation_timestep;
                bool pad_loaded = false;
                double cmd_v = 0.0;
                double cmd_yaw = 0.0;
                double cmd_lift = 0.0;
                const bool in_resume_hold_window = false;
                if (rd_integration.is_enabled()) {
                    const auto cur_pos = controller.getForklift().getPosition();
                    if (!rd_integration.tick(cur_pos.x)) {
                        std::cerr << "[WARN] RD-lite tick failed" << std::endl;
                    }
                }
                const bool allow_step = rd_integration.is_local_owner();
                if (allow_step != prev_allow_step) {
                    std::cout << "[RD-LITE] render owner_active=" << (allow_step ? "yes" : "no")
                              << " step=" << step_count << std::endl;
                }
                if (prev_allow_step && !allow_step) {
                    HakoniwaMujocoContext::ForkliftState standby_state {};
                    if (!mujoco_ctx.restore_forklift_state(&standby_state, nullptr)) {
                        std::cerr << "[WARN] RD-lite standby sync failed: restore_forklift_state()" << std::endl;
                    }
                    if (world_ && world_->getData()) {
                        mjData* d = world_->getData();
                        const int nu = (world_->getModel() != nullptr) ? world_->getModel()->nu : 0;
                        for (int i = 0; i < nu; i++) {
                            d->ctrl[i] = 0.0;
                        }
                    }
                }
                visibility.set_owner_active(allow_step);
                if (!allow_step) {
                    if (rd_integration.standby_physics_enabled()) {
                        if (world_ && world_->getData()) {
                            mjData* d = world_->getData();
                            const int nu = (world_->getModel() != nullptr) ? world_->getModel()->nu : 0;
                            for (int i = 0; i < nu; i++) {
                                d->ctrl[i] = 0.0;
                            }
                        }
                        world_->advanceTimeStep();
                    }
                    prev_allow_step = allow_step;
                    hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
                    continue;
                }
                if (pdu_runtime.load_pad(pad_data)) {
                    pad_loaded = true;
                    hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                    auto command = adapter.convert(pad_data);
                    cmd_v = command.linear_velocity;
                    cmd_yaw = command.yaw_rate;
                    cmd_lift = command.lift_position;
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
                        control_state.phase = 2;
                    }
                }

                controller.update();
                world_->advanceTimeStep();

                control_state.target_linear_velocity = controller.getTargetLinearVel();
                control_state.target_yaw_rate = controller.getTargetYawRate();
                control_state.target_lift_z = controller.getLiftTarget();
                pdu_runtime.publish_state(controller, control_state);

                step_count++;
                control_state.sim_step = static_cast<std::uint64_t>(step_count);
                auto internal = controller.get_internal_state();
                control_state.lift_pid_integral = internal.lift_pid.integral;
                control_state.lift_pid_prev_error = internal.lift_pid.prev_error;
                control_state.drive_v_pid_integral = internal.drive_v_pid.integral;
                control_state.drive_v_pid_prev_error = internal.drive_v_pid.prev_error;
                control_state.drive_w_pid_integral = internal.drive_w_pid.integral;
                control_state.drive_w_pid_prev_error = internal.drive_w_pid.prev_error;

                if (trace_logger.restore_debug_enabled() && restored &&
                    (sim_time_sec_from_resume >= -1e-9) &&
                    (sim_time_sec_from_resume <= trace_logger.restore_debug_window_sec())) {
                    const auto body_v = controller.getForklift().getBodyVelocity();
                    const auto body_w = controller.getForklift().getBodyAngularVelocity();
                    ForkliftRestoreDebugSample s {};
                    s.restored = restored;
                    s.step = step_count;
                    s.sim_time_sec = sim_time_sec;
                    s.sim_time_from_resume = sim_time_sec_from_resume;
                    s.pad_loaded = pad_loaded;
                    s.in_resume_hold = in_resume_hold_window;
                    s.cmd_v = cmd_v;
                    s.cmd_yaw = cmd_yaw;
                    s.cmd_lift = cmd_lift;
                    s.target_v = control_state.target_linear_velocity;
                    s.target_yaw = control_state.target_yaw_rate;
                    s.target_lift = control_state.target_lift_z;
                    s.body_vx = body_v.x;
                    s.body_wz = body_w.z;
                    s.left_torque = controller.getLastLeftTorque();
                    s.right_torque = controller.getLastRightTorque();
                    s.lift_torque = controller.getLastLiftTorque();
                    s.drive_v_pid_integral = control_state.drive_v_pid_integral;
                    s.drive_v_pid_prev_error = control_state.drive_v_pid_prev_error;
                    s.drive_w_pid_integral = control_state.drive_w_pid_integral;
                    s.drive_w_pid_prev_error = control_state.drive_w_pid_prev_error;
                    trace_logger.log_restore_debug(s);
                }
                if ((step_count % trace_logger.trace_every_steps()) == 0) {
                    const auto p = controller.getForklift().getPosition();
                    const auto e = controller.getForklift().getEuler();
                    const auto v = controller.getForklift().getBodyVelocity();
                    const auto w = controller.getForklift().getBodyAngularVelocity();
                    const auto l = controller.getForklift().getLiftPosition();
                    ForkliftTraceSample s {};
                    s.restored = restored;
                    s.step = step_count;
                    s.sim_time_sec = sim_time_sec;
                    s.pos_x = p.x;
                    s.pos_y = p.y;
                    s.pos_z = p.z;
                    s.yaw = e.z;
                    s.body_vx = v.x;
                    s.body_wz = w.z;
                    s.lift_z = l.z;
                    s.target_v = control_state.target_linear_velocity;
                    s.target_yaw = control_state.target_yaw_rate;
                    s.target_lift = control_state.target_lift_z;
                    s.phase = control_state.phase;
                    trace_logger.log_trace(s);
                }
                if (rd_integration.is_enabled() &&
                    ((step_count % rd_integration.status_log_every_steps()) == 0)) {
                    const auto p = controller.getForklift().getPosition();
                    const bool local_owner = rd_integration.is_local_owner();
                    std::cout << "[RD-LITE] status"
                              << " step=" << step_count
                              << " owner=" << (local_owner ? "yes" : "no")
                              << " pos_x=" << p.x
                              << " dist_to_release=" << (rd_integration.release_x() - p.x)
                              << " dist_to_home=" << (p.x - rd_integration.home_x())
                              << std::endl;
                }
                if (local_state_enabled && mujoco_ctx.should_autosave(step_count)) {
                    (void)mujoco_ctx.save_forklift_state_with_control(&control_state);
                    const auto p = controller.getForklift().getPosition();
                    const auto e = controller.getForklift().getEuler();
                    const auto l = controller.getForklift().getLiftPosition();
                    ForkliftRecoveryAutosaveSample s {};
                    s.step = control_state.sim_step;
                    s.pos_x = p.x;
                    s.pos_y = p.y;
                    s.pos_z = p.z;
                    s.euler_x = e.x;
                    s.euler_y = e.y;
                    s.euler_z = e.z;
                    s.lift_z = l.z;
                    s.phase = control_state.phase;
                    recovery_logger.log_autosave(now_local_time_string(), s);
                }
                prev_allow_step = allow_step;
            }

            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double sleep_time = simulation_timestep - elapsed.count();
            hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
        }
        if (local_state_enabled) {
            (void)mujoco_ctx.save_forklift_state_with_control(&control_state);
            std::cout << "[INFO] Saved forklift state to: " << mujoco_ctx.state_file_path() << std::endl;
        } else {
            std::cout << "[INFO] Local state save skipped (HAKO_LOCAL_STATE_ENABLE=0)." << std::endl;
        }
        const auto final_pos = controller.getForklift().getPosition();
        const auto final_euler = controller.getForklift().getEuler();
        const auto final_lift = controller.getForklift().getLiftPosition();
        {
            ForkliftRecoverySample s {};
            s.restored = true;
            s.state_file = mujoco_ctx.state_file_path();
            s.pos_x = final_pos.x;
            s.pos_y = final_pos.y;
            s.pos_z = final_pos.z;
            s.euler_x = final_euler.x;
            s.euler_y = final_euler.y;
            s.euler_z = final_euler.z;
            s.lift_z = final_lift.z;
            s.phase = control_state.phase;
            s.target_v = control_state.target_linear_velocity;
            s.target_yaw = control_state.target_yaw_rate;
            s.target_lift = control_state.target_lift_z;
            s.step = control_state.sim_step;
            recovery_logger.log_end(now_local_time_string(), s);
        }
    } catch (const std::exception& e) {
        std::fflush(stdout);
        std::cerr << "Exception in simulation thread: " << e.what() << std::endl;
        running_flag_ = false;
    }
    return 0;
}
