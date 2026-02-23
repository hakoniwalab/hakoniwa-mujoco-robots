#include "forklift_recovery_logger.hpp"

#include <cstdlib>
#include <filesystem>
#include <iomanip>

namespace {
std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}
} // namespace

ForkliftRecoveryLogger::ForkliftRecoveryLogger()
{
    std::filesystem::create_directories("./logs");
    const std::string path =
        get_env_string("HAKO_FORKLIFT_RECOVERY_LOG_FILE", "./logs/forklift-unit-recovery.log");
    log_.open(path, std::ios::app);
    log_ << std::fixed << std::setprecision(6);
}

void ForkliftRecoveryLogger::log_start(const std::string& ts, const ForkliftRecoverySample& s)
{
    if (!log_.good()) {
        return;
    }
    log_ << "[" << ts << "] START"
         << " restored=" << (s.restored ? "yes" : "no")
         << " state_file=" << s.state_file
         << " pos=(" << s.pos_x << "," << s.pos_y << "," << s.pos_z << ")"
         << " euler=(" << s.euler_x << "," << s.euler_y << "," << s.euler_z << ")"
         << " lift_z=" << s.lift_z
         << " phase=" << s.phase
         << " target_v=" << s.target_v
         << " target_yaw=" << s.target_yaw
         << " target_lift=" << s.target_lift
         << " step=" << s.step
         << std::endl;
}

void ForkliftRecoveryLogger::log_autosave(const std::string& ts, const ForkliftRecoveryAutosaveSample& s)
{
    if (!log_.good()) {
        return;
    }
    log_ << "[" << ts << "] AUTOSAVE"
         << " step=" << s.step
         << " pos=(" << s.pos_x << "," << s.pos_y << "," << s.pos_z << ")"
         << " euler=(" << s.euler_x << "," << s.euler_y << "," << s.euler_z << ")"
         << " lift_z=" << s.lift_z
         << " phase=" << s.phase
         << std::endl;
}

void ForkliftRecoveryLogger::log_end(const std::string& ts, const ForkliftRecoverySample& s)
{
    if (!log_.good()) {
        return;
    }
    log_ << "[" << ts << "] END"
         << " saved=yes"
         << " state_file=" << s.state_file
         << " pos=(" << s.pos_x << "," << s.pos_y << "," << s.pos_z << ")"
         << " euler=(" << s.euler_x << "," << s.euler_y << "," << s.euler_z << ")"
         << " lift_z=" << s.lift_z
         << " phase=" << s.phase
         << " target_v=" << s.target_v
         << " target_yaw=" << s.target_yaw
         << " target_lift=" << s.target_lift
         << " step=" << s.step
         << std::endl;
}
