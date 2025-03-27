#include "mujoco_debug.hpp"

std::string get_joint_type_by_name(const mjModel* model, const std::string& joint_name) {
    int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
        return "[ERROR] Joint not found: " + joint_name;
    }

    int type = model->jnt_type[joint_id];
    switch (type) {
        case mjJNT_HINGE: return "Hinge (Revolute)";
        case mjJNT_SLIDE: return "Slide (Prismatic)";
        case mjJNT_BALL: return "Ball (Spherical)";
        case mjJNT_FREE: return "Free (6DOF)";
        default: return "Unknown";
    }
}
void print_body_inertia_by_name(const mjModel* model, const std::string& body_name) {
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id != -1) {
        std::cout << "[Body Inertia] " << body_name 
                  << " | Mass: " << model->body_mass[body_id]
                  << " | Inertia Tensor: (" << model->body_inertia[3 * body_id] << ", "
                                            << model->body_inertia[3 * body_id + 1] << ", "
                                            << model->body_inertia[3 * body_id + 2] << ")"
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
    }
}
void print_actuator_range_by_name(const mjModel* model, const std::string& actuator_name) {
    int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id != -1) {
        std::cout << "[Actuator Range] " << actuator_name
                  << " | Control Range: (" << model->actuator_ctrlrange[2 * actuator_id] << ", "
                                           << model->actuator_ctrlrange[2 * actuator_id + 1] << ")"
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Actuator not found: " << actuator_name << std::endl;
    }
}

void print_joint_state_by_name(const mjModel* model, const mjData* data, const std::string& joint_name) {
    int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id != -1) {
        std::cout << "[Joint] " << joint_name 
                  << " | qpos: " << data->qpos[joint_id]
                  << ", qvel: " << data->qvel[joint_id] 
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Joint not found: " << joint_name << std::endl;
    }
}

void print_body_state_by_name(const mjModel* model, const mjData* data, const std::string& body_name) {
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id != -1) {
        std::cout << "[Body] " << body_name 
                  << " | Position: (" << data->xpos[3 * body_id] << ", "
                                      << data->xpos[3 * body_id + 1] << ", "
                                      << data->xpos[3 * body_id + 2] << ")" 
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
    }
}

void print_actuator_by_name(const mjModel* model, const mjData* data, const std::string& actuator_name) {
    int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id != -1) {
        std::cout << "[Actuator] " << actuator_name 
                  << " | Control Input: " << data->ctrl[actuator_id] 
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Actuator not found: " << actuator_name << std::endl;
    }
}
void print_hinge_joint_state_deg(const mjModel* model, const mjData* data, const std::string& joint_name) {
    int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id != -1) {
        double angle_deg = data->qpos[joint_id] * (180.0 / M_PI);  // ラジアンを度に変換
        std::cout << "[Hinge Joint] " << joint_name 
                  << " | Angle (deg): " << angle_deg 
                  << "° | Angular Velocity (rad/s): " << data->qvel[joint_id] 
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Joint not found: " << joint_name << std::endl;
    }
}

// クォータニオン → オイラー角（ロール・ピッチ・ヨー）の変換関数
void quat_to_euler(const double* quat, double& roll, double& pitch, double& yaw) {
    double w = quat[0], x = quat[1], y = quat[2], z = quat[3];

    // Roll (X軸回転)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y軸回転)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);  // 90度 (Gimbal lock)
    else
        pitch = std::asin(sinp);

    // Yaw (Z軸回転)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ボディの姿勢をラジアンで出力
void print_body_orientation_by_name_rad(const mjModel* model, const mjData* data, const std::string& body_name) {
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id != -1) {
        double roll, pitch, yaw;
        quat_to_euler(&data->xquat[4 * body_id], roll, pitch, yaw);
        
        std::cout << "[Body Orientation (rad)] " << body_name 
                  << " | Roll: " << roll
                  << ", Pitch: " << pitch
                  << ", Yaw: " << yaw
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
    }
}

// ボディの姿勢を度数法で出力
void print_body_orientation_by_name_deg(const mjModel* model, const mjData* data, const std::string& body_name) {
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id != -1) {
        double roll, pitch, yaw;
        quat_to_euler(&data->xquat[4 * body_id], roll, pitch, yaw);

        std::cout << "[Body Orientation (deg)] " << body_name 
                  << " | Roll: " << roll * (180.0 / M_PI) << "°"
                  << ", Pitch: " << pitch * (180.0 / M_PI) << "°"
                  << ", Yaw: " << yaw * (180.0 / M_PI) << "°"
                  << std::endl;
    } else {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
    }
}

void print_all_states(const mjModel* model, const mjData* data) {
    std::cout << "========== Simulation State ==========" << std::endl;
    
    // 基本的な時間情報
    std::cout << "[Time] Simulation Time: " << data->time << " s" << std::endl;

    // 関節の状態
    print_hinge_joint_state_deg(model, data, "left_wheel_hinge");
    print_hinge_joint_state_deg(model, data, "right_wheel_hinge");

    // 剛体の状態
    print_body_state_by_name(model, data, "tb3_base");
    print_body_orientation_by_name_deg(model, data, "tb3_base");
    print_body_state_by_name(model, data, "left_wheel");
    print_body_orientation_by_name_deg(model, data, "left_wheel");
    print_body_state_by_name(model, data, "right_wheel");
    print_body_orientation_by_name_deg(model, data, "right_wheel");

    // アクチュエータの制御入力
    print_actuator_by_name(model, data, "left_motor");
    print_actuator_by_name(model, data, "right_motor");

    std::cout << "=======================================" << std::endl;
}
