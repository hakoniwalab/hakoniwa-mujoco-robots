#include "mujoco_debug.hpp"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

void quat_to_euler(const double* quat, double& roll, double& pitch, double& yaw) {
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(kPi / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

std::string get_joint_type_by_name(const mjModel* model, const std::string& joint_name) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
        return "[ERROR] Joint not found: " + joint_name;
    }

    switch (model->jnt_type[joint_id]) {
        case mjJNT_HINGE:
            return "Hinge (Revolute)";
        case mjJNT_SLIDE:
            return "Slide (Prismatic)";
        case mjJNT_BALL:
            return "Ball (Spherical)";
        case mjJNT_FREE:
            return "Free (6DOF)";
        default:
            return "Unknown";
    }
}

void print_body_inertia_by_name(const mjModel* model, const std::string& body_name) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id == -1) {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
        return;
    }

    std::cout << "[Body Inertia] " << body_name
              << " | Mass: " << model->body_mass[body_id]
              << " | Inertia Tensor: (" << model->body_inertia[3 * body_id] << ", "
              << model->body_inertia[3 * body_id + 1] << ", "
              << model->body_inertia[3 * body_id + 2] << ")" << std::endl;
}

void print_actuator_range_by_name(const mjModel* model, const std::string& actuator_name) {
    const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id == -1) {
        std::cerr << "[ERROR] Actuator not found: " << actuator_name << std::endl;
        return;
    }

    std::cout << "[Actuator Range] " << actuator_name
              << " | Control Range: (" << model->actuator_ctrlrange[2 * actuator_id] << ", "
              << model->actuator_ctrlrange[2 * actuator_id + 1] << ")" << std::endl;
}

void print_joint_state_by_name(const mjModel* model, const mjData* data, const std::string& joint_name) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
        std::cerr << "[ERROR] Joint not found: " << joint_name << std::endl;
        return;
    }

    std::cout << "[Joint] " << joint_name
              << " | qpos: " << data->qpos[joint_id]
              << ", qvel: " << data->qvel[joint_id] << std::endl;
}

void print_body_state_by_name(const mjModel* model, const mjData* data, const std::string& body_name) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id == -1) {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
        return;
    }

    std::cout << "[Body] " << body_name
              << " | Position: (" << data->xpos[3 * body_id] << ", "
              << data->xpos[3 * body_id + 1] << ", "
              << data->xpos[3 * body_id + 2] << ")" << std::endl;
}

void print_actuator_by_name(const mjModel* model, const mjData* data, const std::string& actuator_name) {
    const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id == -1) {
        std::cerr << "[ERROR] Actuator not found: " << actuator_name << std::endl;
        return;
    }

    std::cout << "[Actuator] " << actuator_name
              << " | Control Input: " << data->ctrl[actuator_id] << std::endl;
}

void print_hinge_joint_state_deg(const mjModel* model, const mjData* data, const std::string& joint_name) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
        std::cerr << "[ERROR] Joint not found: " << joint_name << std::endl;
        return;
    }

    const double angle_deg = data->qpos[joint_id] * (180.0 / kPi);
    std::cout << "[Hinge Joint] " << joint_name
              << " | Angle (deg): " << angle_deg
              << " | Angular Velocity (rad/s): " << data->qvel[joint_id] << std::endl;
}

void print_body_orientation_by_name_rad(const mjModel* model, const mjData* data, const std::string& body_name) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id == -1) {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
        return;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quat_to_euler(&data->xquat[4 * body_id], roll, pitch, yaw);

    std::cout << "[Body Orientation (rad)] " << body_name
              << " | Roll: " << roll
              << ", Pitch: " << pitch
              << ", Yaw: " << yaw << std::endl;
}

void print_body_orientation_by_name_deg(const mjModel* model, const mjData* data, const std::string& body_name) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body_id == -1) {
        std::cerr << "[ERROR] Body not found: " << body_name << std::endl;
        return;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quat_to_euler(&data->xquat[4 * body_id], roll, pitch, yaw);

    std::cout << "[Body Orientation (deg)] " << body_name
              << " | Roll: " << roll * (180.0 / kPi)
              << ", Pitch: " << pitch * (180.0 / kPi)
              << ", Yaw: " << yaw * (180.0 / kPi) << std::endl;
}

void print_all_states(const mjModel* model, const mjData* data) {
    std::cout << "========== Simulation State ==========" << std::endl;
    std::cout << "[Time] Simulation Time: " << data->time << " s" << std::endl;

    print_hinge_joint_state_deg(model, data, "left_wheel_hinge");
    print_hinge_joint_state_deg(model, data, "right_wheel_hinge");
    print_body_state_by_name(model, data, "tb3_base");
    print_body_orientation_by_name_deg(model, data, "tb3_base");
    print_body_state_by_name(model, data, "left_wheel");
    print_body_orientation_by_name_deg(model, data, "left_wheel");
    print_body_state_by_name(model, data, "right_wheel");
    print_body_orientation_by_name_deg(model, data, "right_wheel");
    print_actuator_by_name(model, data, "left_motor");
    print_actuator_by_name(model, data, "right_motor");

    std::cout << "=======================================" << std::endl;
}
