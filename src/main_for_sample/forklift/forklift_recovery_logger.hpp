#pragma once

#include <cstdint>
#include <fstream>
#include <string>

struct ForkliftRecoverySample {
    bool restored {false};
    std::string state_file {};
    double pos_x {0.0};
    double pos_y {0.0};
    double pos_z {0.0};
    double euler_x {0.0};
    double euler_y {0.0};
    double euler_z {0.0};
    double lift_z {0.0};
    int phase {0};
    double target_v {0.0};
    double target_yaw {0.0};
    double target_lift {0.0};
    std::uint64_t step {0};
};

struct ForkliftRecoveryAutosaveSample {
    std::uint64_t step {0};
    double pos_x {0.0};
    double pos_y {0.0};
    double pos_z {0.0};
    double euler_x {0.0};
    double euler_y {0.0};
    double euler_z {0.0};
    double lift_z {0.0};
    int phase {0};
};

class ForkliftRecoveryLogger {
public:
    ForkliftRecoveryLogger();

    void log_start(const std::string& ts, const ForkliftRecoverySample& s);
    void log_autosave(const std::string& ts, const ForkliftRecoveryAutosaveSample& s);
    void log_end(const std::string& ts, const ForkliftRecoverySample& s);

private:
    std::ofstream log_ {};
};
