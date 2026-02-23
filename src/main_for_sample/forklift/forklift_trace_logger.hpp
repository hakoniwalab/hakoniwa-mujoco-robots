#pragma once

#include <fstream>
#include <string>

struct ForkliftTraceSample {
    bool restored {false};
    int step {0};
    double sim_time_sec {0.0};
    double pos_x {0.0};
    double pos_y {0.0};
    double pos_z {0.0};
    double yaw {0.0};
    double body_vx {0.0};
    double body_wz {0.0};
    double lift_z {0.0};
    double target_v {0.0};
    double target_yaw {0.0};
    double target_lift {0.0};
    int phase {0};
};

struct ForkliftRestoreDebugSample {
    bool restored {false};
    int step {0};
    double sim_time_sec {0.0};
    double sim_time_from_resume {0.0};
    bool pad_loaded {false};
    bool in_resume_hold {false};
    double cmd_v {0.0};
    double cmd_yaw {0.0};
    double cmd_lift {0.0};
    double target_v {0.0};
    double target_yaw {0.0};
    double target_lift {0.0};
    double body_vx {0.0};
    double body_wz {0.0};
    double left_torque {0.0};
    double right_torque {0.0};
    double lift_torque {0.0};
    double drive_v_pid_integral {0.0};
    double drive_v_pid_prev_error {0.0};
    double drive_w_pid_integral {0.0};
    double drive_w_pid_prev_error {0.0};
};

class ForkliftTraceLogger {
public:
    explicit ForkliftTraceLogger(const std::string& session_ts);

    int trace_every_steps() const;
    bool restore_debug_enabled() const;
    double restore_debug_window_sec() const;

    void log_trace(const ForkliftTraceSample& sample);
    void log_restore_debug(const ForkliftRestoreDebugSample& sample);

private:
    std::string session_ts_ {};
    int trace_every_steps_ {10};
    bool restore_debug_enabled_ {true};
    double restore_debug_window_sec_ {3.0};
    std::ofstream trace_log_ {};
    std::ofstream restore_debug_log_ {};
};
