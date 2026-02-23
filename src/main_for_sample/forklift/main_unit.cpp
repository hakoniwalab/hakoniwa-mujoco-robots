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
#include <limits>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include <array>

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
#include "rd_lite/rd_lite.hpp"

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

static int get_env_int(const char* name, int default_value)
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

static double get_env_double(const char* name, double default_value)
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

static std::string get_trace_file_path()
{
    const char* env = std::getenv("HAKO_FORKLIFT_TRACE_FILE");
    if (env == nullptr || env[0] == '\0') {
        return "./logs/forklift-unit-trace.csv";
    }
    return std::string(env);
}

static std::string get_recovery_log_file_path()
{
    const char* env = std::getenv("HAKO_FORKLIFT_RECOVERY_LOG_FILE");
    if (env == nullptr || env[0] == '\0') {
        return "./logs/forklift-unit-recovery.log";
    }
    return std::string(env);
}

static std::string get_restore_debug_file_path()
{
    const char* env = std::getenv("HAKO_FORKLIFT_RESTORE_DEBUG_FILE");
    if (env == nullptr || env[0] == '\0') {
        return "./logs/forklift-unit-restore-debug.csv";
    }
    return std::string(env);
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

class ForkliftVisibilityController {
public:
    bool initialize(mjModel* model, mjData* data, const char* base_body_name)
    {
        if (initialized_ || model == nullptr || base_body_name == nullptr) {
            return false;
        }
        model_ = model;
        data_ = data;
        const int base_body_id = mj_name2id(model_, mjOBJ_BODY, base_body_name);
        if (base_body_id < 0) {
            return false;
        }
        std::unordered_set<int> subtree_body_ids;
        for (int b = 0; b < model_->nbody; b++) {
            int cur = b;
            while (cur >= 0) {
                if (cur == base_body_id) {
                    subtree_body_ids.insert(b);
                    break;
                }
                if (cur == 0) {
                    break;
                }
                cur = model_->body_parentid[cur];
            }
        }
        for (int g = 0; g < model_->ngeom; g++) {
            if (subtree_body_ids.find(model_->geom_bodyid[g]) == subtree_body_ids.end()) {
                continue;
            }
            geom_ids_.push_back(g);
            std::array<float, 4> rgba {};
            rgba[0] = model_->geom_rgba[(g * 4) + 0];
            rgba[1] = model_->geom_rgba[(g * 4) + 1];
            rgba[2] = model_->geom_rgba[(g * 4) + 2];
            rgba[3] = model_->geom_rgba[(g * 4) + 3];
            original_rgba_.push_back(rgba);
            original_contype_.push_back(model_->geom_contype[g]);
            original_conaffinity_.push_back(model_->geom_conaffinity[g]);
        }
        std::unordered_set<int> dof_set;
        for (int b : subtree_body_ids) {
            const int jadr = model_->body_jntadr[b];
            const int jnum = model_->body_jntnum[b];
            for (int j = jadr; j < (jadr + jnum); j++) {
                const int dadr = model_->jnt_dofadr[j];
                int dnum = 0;
                switch (model_->jnt_type[j]) {
                case mjJNT_FREE: dnum = 6; break;
                case mjJNT_BALL: dnum = 3; break;
                case mjJNT_SLIDE:
                case mjJNT_HINGE: dnum = 1; break;
                default: dnum = 0; break;
                }
                for (int k = 0; k < dnum; k++) {
                    dof_set.insert(dadr + k);
                }
            }
        }
        subtree_dof_ids_.assign(dof_set.begin(), dof_set.end());
        std::sort(subtree_dof_ids_.begin(), subtree_dof_ids_.end());
        ground_geom_id_ = mj_name2id(model_, mjOBJ_GEOM, "ground");
        if (ground_geom_id_ >= 0) {
            original_ground_contype_ = model_->geom_contype[ground_geom_id_];
            original_ground_conaffinity_ = model_->geom_conaffinity[ground_geom_id_];
        }
        standby_weld_eq_id_ = mj_name2id(model_, mjOBJ_EQUALITY, "forklift_standby_weld");
        initialized_ = !geom_ids_.empty();
        return initialized_;
    }

    void set_standby_alpha(float alpha)
    {
        standby_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
    }

    void set_standby_tint(float tint)
    {
        standby_tint_ = std::clamp(tint, 0.0f, 1.0f);
    }

    void set_standby_weld_enabled(bool enabled)
    {
        standby_weld_enabled_ = enabled;
    }

    void set_standby_zero_dynamics(bool enabled)
    {
        standby_zero_dynamics_ = enabled;
    }

    void set_owner_active(bool owner_active)
    {
        if (!initialized_ || model_ == nullptr || owner_active_ == owner_active) {
            return;
        }
        // Rendering: standby -> semi-transparent/inactive appearance
        for (std::size_t i = 0; i < geom_ids_.size(); i++) {
            const int g = geom_ids_[i];
            if (owner_active) {
                model_->geom_rgba[(g * 4) + 0] = original_rgba_[i][0];
                model_->geom_rgba[(g * 4) + 1] = original_rgba_[i][1];
                model_->geom_rgba[(g * 4) + 2] = original_rgba_[i][2];
                model_->geom_rgba[(g * 4) + 3] = original_rgba_[i][3];
            } else {
                model_->geom_rgba[(g * 4) + 0] = original_rgba_[i][0] * standby_tint_;
                model_->geom_rgba[(g * 4) + 1] = original_rgba_[i][1] * standby_tint_;
                model_->geom_rgba[(g * 4) + 2] = original_rgba_[i][2] * standby_tint_;
                model_->geom_rgba[(g * 4) + 3] = standby_alpha_;
            }
        }
        if (owner_active) {
            // Restore original collision masks.
            for (std::size_t i = 0; i < geom_ids_.size(); i++) {
                const int g = geom_ids_[i];
                model_->geom_contype[g] = original_contype_[i];
                model_->geom_conaffinity[g] = original_conaffinity_[i];
            }
            if (ground_geom_id_ >= 0) {
                model_->geom_contype[ground_geom_id_] = original_ground_contype_;
                model_->geom_conaffinity[ground_geom_id_] = original_ground_conaffinity_;
            }
            if (data_ != nullptr && standby_weld_eq_id_ >= 0 && standby_weld_enabled_) {
                data_->eq_active[standby_weld_eq_id_] = 0;
            }
        } else {
            // Standby collision rule:
            // - collide with ground only
            // - no collision with any other rigid body
            // - optional weld freeze (disabled by default)
            constexpr int kStandbyType = 2;
            for (int g : geom_ids_) {
                model_->geom_contype[g] = 0;
                model_->geom_conaffinity[g] = kStandbyType;
            }
            if (ground_geom_id_ >= 0) {
                model_->geom_contype[ground_geom_id_] = (original_ground_contype_ | kStandbyType);
                model_->geom_conaffinity[ground_geom_id_] = original_ground_conaffinity_;
            }
            if (data_ != nullptr && standby_weld_eq_id_ >= 0 && standby_weld_enabled_) {
                data_->eq_active[standby_weld_eq_id_] = 1;
            }
            if (data_ != nullptr && standby_zero_dynamics_) {
                for (int d : subtree_dof_ids_) {
                    data_->qvel[d] = 0.0;
                    data_->qacc[d] = 0.0;
                    data_->qacc_warmstart[d] = 0.0;
                }
                for (int i = 0; i < model_->nu; i++) {
                    data_->ctrl[i] = 0.0;
                }
                for (int i = 0; i < model_->na; i++) {
                    data_->act[i] = 0.0;
                }
            }
        }
        if (model_ != nullptr && data_ != nullptr) {
            mj_forward(model_, data_);
        }
        owner_active_ = owner_active;
    }

private:
    mjModel* model_ {nullptr};
    mjData* data_ {nullptr};
    bool initialized_ {false};
    bool owner_active_ {true};
    std::vector<int> geom_ids_ {};
    std::vector<std::array<float, 4>> original_rgba_ {};
    std::vector<int> subtree_dof_ids_ {};
    std::vector<int> original_contype_ {};
    std::vector<int> original_conaffinity_ {};
    int ground_geom_id_ {-1};
    int original_ground_contype_ {1};
    int original_ground_conaffinity_ {1};
    int standby_weld_eq_id_ {-1};
    float standby_alpha_ {0.0f};
    float standby_tint_ {0.35f};
    bool standby_weld_enabled_ {false};
    bool standby_zero_dynamics_ {true};
};

static std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

static bool read_file_bytes(const std::string& path, std::vector<std::uint8_t>& out)
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

static bool write_file_bytes(const std::string& path, const std::vector<std::uint8_t>& data)
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

static void apply_restored_control_state(
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

        std::string asset_name = get_env_string("HAKO_ASSET_NAME", "forklift");
        std::string robot_name = get_env_string("HAKO_FORKLIFT_ROBOT_NAME", asset_name);
        std::string robot_name2 = "forklift_fork";

        PduChannel<HakoCpp_GameControllerOperation, hako::pdu::msgs::hako_msgs::GameControllerOperation> pad(robot_name, "hako_cmd_game");
        PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_pos(robot_name, "pos");
        PduChannel<HakoCpp_Float64, hako::pdu::msgs::std_msgs::Float64> lift_pos(robot_name, "height");
        PduChannel<HakoCpp_Int32, hako::pdu::msgs::std_msgs::Int32> phase_pos(robot_name, "phase");
        PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_fork_pos(robot_name2, "pos");
        ForkliftVisibilityController visibility;
        (void)visibility.initialize(world->getModel(), world->getData(), "forklift_base");
        visibility.set_standby_alpha(static_cast<float>(get_env_double("HAKO_RD_LITE_STANDBY_ALPHA", 0.2)));
        visibility.set_standby_tint(static_cast<float>(get_env_double("HAKO_RD_LITE_STANDBY_TINT", 0.35)));
        visibility.set_standby_weld_enabled(get_env_int("HAKO_RD_LITE_STANDBY_WELD", 0) != 0);
        visibility.set_standby_zero_dynamics(get_env_int("HAKO_RD_LITE_STANDBY_ZERO_DYNAMICS", 1) != 0);

        hako::robots::controller::ForkliftController controller(world);
        controller.setVelocityCommand(0.0, 0.0);
        controller.setLiftTarget(0.0);
        controller.set_delta_pos(simulation_timestep * get_motion_gain());
        HakoniwaMujocoContext mujoco_ctx(world, "./tmp/hakoniwa-forklift-unit.state");
        const bool rd_lite_enabled = (get_env_int("HAKO_RD_LITE_ENABLE", 0) != 0);
        const bool local_state_enabled = (get_env_int("HAKO_LOCAL_STATE_ENABLE", rd_lite_enabled ? 0 : 1) != 0);
        std::filesystem::create_directories("./logs");
        const std::string recovery_log_file = get_recovery_log_file_path();
        std::ofstream recovery_log(recovery_log_file, std::ios::app);
        recovery_log << std::fixed << std::setprecision(6);
        const std::string trace_file = get_trace_file_path();
        const int trace_every_steps = std::max(1, get_env_int("HAKO_FORKLIFT_TRACE_EVERY_STEPS", 10));
        const bool trace_file_exists = std::filesystem::exists(trace_file);
        std::ofstream trace_log(trace_file, std::ios::app);
        const bool restore_debug_enabled = (get_env_int("HAKO_FORKLIFT_RESTORE_DEBUG", 1) != 0);
        const double restore_debug_window_sec = get_env_double("HAKO_FORKLIFT_RESTORE_DEBUG_WINDOW_SEC", 3.0);
        const std::string restore_debug_file = get_restore_debug_file_path();
        const bool restore_debug_exists = std::filesystem::exists(restore_debug_file);
        std::ofstream restore_debug_log(restore_debug_file, std::ios::app);
        if (!trace_file_exists) {
            trace_log
                << "session_ts,restored,step,sim_time_sec,pos_x,pos_y,pos_z,yaw,body_vx,body_wz,"
                << "lift_z,target_v,target_yaw,target_lift,phase\n";
        }
        if (!restore_debug_exists) {
            restore_debug_log
                << "session_ts,restored,step,sim_time_sec,sim_time_from_resume,pad_loaded,in_resume_hold,"
                << "cmd_v,cmd_yaw,cmd_lift,target_v,target_yaw,target_lift,body_vx,body_wz,"
                << "left_torque,right_torque,lift_torque,"
                << "drive_v_pid_integral,drive_v_pid_prev_error,drive_w_pid_integral,drive_w_pid_prev_error\n";
        }
        const std::string session_ts = now_local_time_string();
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
        if (restored && control_state.sim_step > 0) {
            const auto max_int = static_cast<std::uint64_t>(std::numeric_limits<int>::max());
            step_count = static_cast<int>(std::min(control_state.sim_step, max_int));
        }
        int resumed_step_base = step_count;
        const double resume_cmd_hold_sec = get_env_double("HAKO_FORKLIFT_RESUME_CMD_HOLD_SEC", 2.0);
        const int rd_status_log_every_steps = std::max(1, get_env_int("HAKO_RD_LITE_STATUS_LOG_EVERY_STEPS", 1000));
        double rd_release_x = get_env_double("HAKO_RD_LITE_RELEASE_X", get_env_double("FORWARD_GOAL_X", 5.0));
        double rd_home_x = get_env_double("HAKO_RD_LITE_HOME_X", get_env_double("HOME_GOAL_X", 0.0));
        const bool rd_lite_standby_physics = (get_env_int("HAKO_RD_LITE_STANDBY_PHYSICS", 1) != 0);

        std::unique_ptr<hako::rd_lite::HakoPduRuntimeStatusStore> rd_status_store;
        std::unique_ptr<hako::rd_lite::HakoPduRuntimeContextStore> rd_context_store;
        std::unique_ptr<hako::rd_lite::RdLiteCoordinator> rd_coordinator;
        if (rd_lite_enabled) {
            hako::rd_lite::RdLiteConfig rd_cfg {};
            rd_cfg.asset_name = robot_name;
            rd_cfg.node_id = static_cast<std::uint8_t>(std::clamp(get_env_int("HAKO_RD_LITE_NODE_ID", 1), 0, 255));
            rd_cfg.peer_node_id = static_cast<std::uint8_t>(std::clamp(get_env_int("HAKO_RD_LITE_PEER_NODE_ID", 2), 0, 255));
            rd_cfg.initial_owner = (get_env_int("HAKO_RD_LITE_INITIAL_OWNER", 1) != 0);
            rd_cfg.config_hash = static_cast<std::uint32_t>(get_env_int("HAKO_RD_LITE_CONFIG_HASH", 0));
            rd_cfg.release_x = get_env_double("HAKO_RD_LITE_RELEASE_X", get_env_double("FORWARD_GOAL_X", 5.0));
            rd_cfg.home_x = get_env_double("HAKO_RD_LITE_HOME_X", get_env_double("HOME_GOAL_X", 0.0));
            rd_cfg.goal_tolerance = get_env_double("GOAL_TOLERANCE", 0.03);
            rd_cfg.switch_timeout_sec = get_env_double("HAKO_RD_LITE_SWITCH_TIMEOUT_SEC", 2.0);
            rd_cfg.max_context_bytes = static_cast<std::size_t>(std::max(0, get_env_int("HAKO_RD_LITE_MAX_CONTEXT_BYTES", 4096)));
            rd_cfg.runtime_status_org_name = get_env_string("HAKO_RD_LITE_RUNTIME_STATUS_NAME", "runtime_status");
            rd_cfg.runtime_context_org_name = get_env_string("HAKO_RD_LITE_RUNTIME_CONTEXT_NAME", "runtime_context");

            rd_status_store = std::make_unique<hako::rd_lite::HakoPduRuntimeStatusStore>(
                rd_cfg.asset_name, rd_cfg.runtime_status_org_name);
            rd_context_store = std::make_unique<hako::rd_lite::HakoPduRuntimeContextStore>(
                rd_cfg.asset_name, rd_cfg.runtime_context_org_name);
            if (!rd_status_store->initialize() || !rd_context_store->initialize()) {
                std::cerr << "[WARN] RD-lite disabled: runtime_status/runtime_context PDU not found for asset="
                          << rd_cfg.asset_name << std::endl;
                rd_status_store.reset();
                rd_context_store.reset();
            } else {
                rd_coordinator = std::make_unique<hako::rd_lite::RdLiteCoordinator>(
                    rd_cfg, *rd_status_store, *rd_context_store);
                rd_coordinator->set_logger([](const std::string& msg) {
                    std::cout << "[RD-LITE] " << msg << std::endl;
                });
                bool rd_init_ok = true;
                if (rd_cfg.initial_owner) {
                    rd_init_ok = rd_coordinator->initialize();
                } else {
                    std::cout << "[INFO] RD-lite standby mode: skip runtime_status initialization" << std::endl;
                }
                if (!rd_init_ok) {
                    std::cerr << "[WARN] RD-lite disabled: coordinator initialize failed on initial owner" << std::endl;
                    rd_coordinator.reset();
                    rd_status_store.reset();
                    rd_context_store.reset();
                } else {
                    std::cout << "[INFO] RD-lite enabled. node_id="
                              << static_cast<int>(rd_cfg.node_id)
                              << " peer_id=" << static_cast<int>(rd_cfg.peer_node_id)
                              << " owner=" << (rd_cfg.initial_owner ? "yes" : "no")
                              << " release_x=" << rd_cfg.release_x
                              << " home_x=" << rd_cfg.home_x
                              << " tol=" << rd_cfg.goal_tolerance
                              << std::endl;
                }
            }
        }

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
        bool prev_allow_step = (!rd_coordinator) || rd_coordinator->is_local_owner();

        while (running_flag) {
            auto start = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                const double sim_time_sec = static_cast<double>(step_count) * simulation_timestep;
                const double sim_time_sec_from_resume =
                    static_cast<double>(step_count - resumed_step_base) * simulation_timestep;
                bool pad_loaded = false;
                double cmd_v = 0.0;
                double cmd_yaw = 0.0;
                double cmd_lift = 0.0;
                const bool in_resume_hold_window =
                    restored && (sim_time_sec_from_resume <= resume_cmd_hold_sec);
                if (rd_coordinator) {
                    const auto cur_pos = controller.getForklift().getPosition();
                    if (!rd_coordinator->tick(cur_pos.x, rd_save_context_payload, rd_restore_context_payload)) {
                        std::cerr << "[WARN] RD-lite tick failed" << std::endl;
                    }
                }
                const bool allow_step = (!rd_coordinator) || rd_coordinator->is_local_owner();
                if (allow_step != prev_allow_step) {
                    std::cout << "[RD-LITE] render owner_active=" << (allow_step ? "yes" : "no")
                              << " step=" << step_count << std::endl;
                }
                if (prev_allow_step && !allow_step) {
                    // On owner->standby transition, keep standby forklift at the same
                    // handoff snapshot pose that was saved for RuntimeContext transfer.
                    HakoniwaMujocoContext::ForkliftState standby_state {};
                    if (!mujoco_ctx.restore_forklift_state(&standby_state, nullptr)) {
                        std::cerr << "[WARN] RD-lite standby sync failed: restore_forklift_state()" << std::endl;
                    }
                    if (world && world->getData()) {
                        mjData* d = world->getData();
                        const int nu = (world->getModel() != nullptr) ? world->getModel()->nu : 0;
                        for (int i = 0; i < nu; i++) {
                            d->ctrl[i] = 0.0;
                        }
                    }
                }
                visibility.set_owner_active(allow_step);
                if (!allow_step) {
                    // Standby instance can optionally keep physics stepping without publishing control/PDU.
                    if (rd_lite_standby_physics) {
                        if (world && world->getData()) {
                            mjData* d = world->getData();
                            const int nu = (world->getModel() != nullptr) ? world->getModel()->nu : 0;
                            for (int i = 0; i < nu; i++) {
                                d->ctrl[i] = 0.0;
                            }
                        }
                        world->advanceTimeStep();
                    }
                    prev_allow_step = allow_step;
                    hako_asset_usleep(static_cast<hako_time_t>(delta_time_usec));
                    continue;
                }
                if (pad.load(pad_data)) {
                    pad_loaded = true;
                    hako::robots::pdu::adapter::ForkliftOperationCommand adapter;
                    auto command = adapter.convert(pad_data);
                    cmd_v = command.linear_velocity;
                    cmd_yaw = command.yaw_rate;
                    cmd_lift = command.lift_position;
                    if (!in_resume_hold_window) {
                        control_state.target_linear_velocity = command.linear_velocity;
                        control_state.target_yaw_rate = command.yaw_rate;
                        controller.update_target_lift_z(command.lift_position);
                        controller.setVelocityCommand(command.linear_velocity, command.yaw_rate);
                    }
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
                control_state.target_linear_velocity = controller.getTargetLinearVel();
                control_state.target_yaw_rate = controller.getTargetYawRate();
                control_state.target_lift_z = controller.getLiftTarget();
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
                auto internal = controller.get_internal_state();
                control_state.lift_pid_integral = internal.lift_pid.integral;
                control_state.lift_pid_prev_error = internal.lift_pid.prev_error;
                control_state.drive_v_pid_integral = internal.drive_v_pid.integral;
                control_state.drive_v_pid_prev_error = internal.drive_v_pid.prev_error;
                control_state.drive_w_pid_integral = internal.drive_w_pid.integral;
                control_state.drive_w_pid_prev_error = internal.drive_w_pid.prev_error;
                if (restore_debug_enabled && restore_debug_log.good() && restored &&
                    (sim_time_sec_from_resume >= -1e-9) &&
                    (sim_time_sec_from_resume <= restore_debug_window_sec)) {
                    const auto body_v = controller.getForklift().getBodyVelocity();
                    const auto body_w = controller.getForklift().getBodyAngularVelocity();
                    restore_debug_log
                        << session_ts << ","
                        << (restored ? 1 : 0) << ","
                        << step_count << ","
                        << sim_time_sec << ","
                        << sim_time_sec_from_resume << ","
                        << (pad_loaded ? 1 : 0) << ","
                        << (in_resume_hold_window ? 1 : 0) << ","
                        << cmd_v << ","
                        << cmd_yaw << ","
                        << cmd_lift << ","
                        << control_state.target_linear_velocity << ","
                        << control_state.target_yaw_rate << ","
                        << control_state.target_lift_z << ","
                        << body_v.x << ","
                        << body_w.z << ","
                        << controller.getLastLeftTorque() << ","
                        << controller.getLastRightTorque() << ","
                        << controller.getLastLiftTorque() << ","
                        << control_state.drive_v_pid_integral << ","
                        << control_state.drive_v_pid_prev_error << ","
                        << control_state.drive_w_pid_integral << ","
                        << control_state.drive_w_pid_prev_error
                        << "\n";
                    restore_debug_log.flush();
                }
                if ((step_count % trace_every_steps) == 0) {
                    const auto p = controller.getForklift().getPosition();
                    const auto e = controller.getForklift().getEuler();
                    const auto v = controller.getForklift().getBodyVelocity();
                    const auto w = controller.getForklift().getBodyAngularVelocity();
                    const auto l = controller.getForklift().getLiftPosition();
                    trace_log
                        << session_ts << ","
                        << (restored ? 1 : 0) << ","
                        << step_count << ","
                        << sim_time_sec << ","
                        << p.x << "," << p.y << "," << p.z << ","
                        << e.z << ","
                        << v.x << ","
                        << w.z << ","
                        << l.z << ","
                        << control_state.target_linear_velocity << ","
                        << control_state.target_yaw_rate << ","
                        << control_state.target_lift_z << ","
                        << control_state.phase
                        << "\n";
                    trace_log.flush();
                }
                if (rd_lite_enabled && ((step_count % rd_status_log_every_steps) == 0)) {
                    const auto p = controller.getForklift().getPosition();
                    const bool local_owner = (!rd_coordinator) || rd_coordinator->is_local_owner();
                    std::cout << "[RD-LITE] status"
                              << " step=" << step_count
                              << " owner=" << (local_owner ? "yes" : "no")
                              << " pos_x=" << p.x
                              << " dist_to_release=" << (rd_release_x - p.x)
                              << " dist_to_home=" << (p.x - rd_home_x)
                              << std::endl;
                }
                if (local_state_enabled && mujoco_ctx.should_autosave(step_count)) {
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

    const std::string asset_name = get_env_string("HAKO_ASSET_NAME", "forklift");
    const std::string config_path_env = get_env_string("HAKO_ASSET_CONFIG_PATH", config_path);
    const bool rd_lite_enabled = (get_env_int("HAKO_RD_LITE_ENABLE", 0) != 0);
    const bool rd_lite_initial_owner = (get_env_int("HAKO_RD_LITE_INITIAL_OWNER", 1) != 0);
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
