#include "forklift_trace_logger.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace {
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
} // namespace

ForkliftTraceLogger::ForkliftTraceLogger(const std::string& session_ts)
    : session_ts_(session_ts)
{
    trace_every_steps_ = std::max(1, get_env_int("HAKO_FORKLIFT_TRACE_EVERY_STEPS", 10));
    restore_debug_enabled_ = (get_env_int("HAKO_FORKLIFT_RESTORE_DEBUG", 1) != 0);
    restore_debug_window_sec_ = get_env_double("HAKO_FORKLIFT_RESTORE_DEBUG_WINDOW_SEC", 3.0);

    std::filesystem::create_directories("./logs");
    const std::string trace_file = get_env_string("HAKO_FORKLIFT_TRACE_FILE", "./logs/forklift-unit-trace.csv");
    const bool trace_exists = std::filesystem::exists(trace_file);
    trace_log_.open(trace_file, std::ios::app);
    if (!trace_exists && trace_log_.good()) {
        trace_log_
            << "session_ts,restored,step,sim_time_sec,pos_x,pos_y,pos_z,yaw,body_vx,body_wz,"
            << "lift_z,target_v,target_yaw,target_lift,phase\n";
    }

    const std::string restore_debug_file =
        get_env_string("HAKO_FORKLIFT_RESTORE_DEBUG_FILE", "./logs/forklift-unit-restore-debug.csv");
    const bool restore_exists = std::filesystem::exists(restore_debug_file);
    restore_debug_log_.open(restore_debug_file, std::ios::app);
    if (!restore_exists && restore_debug_log_.good()) {
        restore_debug_log_
            << "session_ts,restored,step,sim_time_sec,sim_time_from_resume,pad_loaded,in_resume_hold,"
            << "cmd_v,cmd_yaw,cmd_lift,target_v,target_yaw,target_lift,body_vx,body_wz,"
            << "left_torque,right_torque,lift_torque,"
            << "drive_v_pid_integral,drive_v_pid_prev_error,drive_w_pid_integral,drive_w_pid_prev_error\n";
    }
}

int ForkliftTraceLogger::trace_every_steps() const
{
    return trace_every_steps_;
}

bool ForkliftTraceLogger::restore_debug_enabled() const
{
    return restore_debug_enabled_;
}

double ForkliftTraceLogger::restore_debug_window_sec() const
{
    return restore_debug_window_sec_;
}

void ForkliftTraceLogger::log_trace(const ForkliftTraceSample& sample)
{
    if (!trace_log_.good()) {
        return;
    }
    trace_log_
        << session_ts_ << ","
        << (sample.restored ? 1 : 0) << ","
        << sample.step << ","
        << sample.sim_time_sec << ","
        << sample.pos_x << "," << sample.pos_y << "," << sample.pos_z << ","
        << sample.yaw << ","
        << sample.body_vx << ","
        << sample.body_wz << ","
        << sample.lift_z << ","
        << sample.target_v << ","
        << sample.target_yaw << ","
        << sample.target_lift << ","
        << sample.phase
        << "\n";
    trace_log_.flush();
}

void ForkliftTraceLogger::log_restore_debug(const ForkliftRestoreDebugSample& sample)
{
    if (!restore_debug_enabled_ || !restore_debug_log_.good()) {
        return;
    }
    restore_debug_log_
        << session_ts_ << ","
        << (sample.restored ? 1 : 0) << ","
        << sample.step << ","
        << sample.sim_time_sec << ","
        << sample.sim_time_from_resume << ","
        << (sample.pad_loaded ? 1 : 0) << ","
        << (sample.in_resume_hold ? 1 : 0) << ","
        << sample.cmd_v << ","
        << sample.cmd_yaw << ","
        << sample.cmd_lift << ","
        << sample.target_v << ","
        << sample.target_yaw << ","
        << sample.target_lift << ","
        << sample.body_vx << ","
        << sample.body_wz << ","
        << sample.left_torque << ","
        << sample.right_torque << ","
        << sample.lift_torque << ","
        << sample.drive_v_pid_integral << ","
        << sample.drive_v_pid_prev_error << ","
        << sample.drive_w_pid_integral << ","
        << sample.drive_w_pid_prev_error
        << "\n";
    restore_debug_log_.flush();
}
