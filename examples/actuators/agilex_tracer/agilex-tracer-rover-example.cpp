#include "physics/physics_impl.hpp"
#include "robots/rover/differential_drive_kinematics.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kDefaultModelPath =
    "../hakoniwa-mbody-registry/bodies/agilex_tracer/generated/tracer_v1.minimal_world.xml";
constexpr const char* kDefaultLeftConfigPath =
    "config/actuator/joint/agilex_tracer_left_wheel.json";
constexpr const char* kDefaultRightConfigPath =
    "config/actuator/joint/agilex_tracer_right_wheel.json";

constexpr const char* kBaseFreejointName = "base_freejoint";
constexpr const char* kLeftWheelJointName = "left_wheel";
constexpr const char* kRightWheelJointName = "right_wheel";

constexpr double kWheelRadius = 0.082;
constexpr double kWheelSeparation = 0.34;
constexpr double kMaxWheelAngularVelocity = 18.0;
constexpr double kLinearStep = 0.05;
constexpr double kAngularStep = 0.2;
constexpr double kLinearMin = -0.6;
constexpr double kLinearMax = 0.6;
constexpr double kAngularMin = -2.0;
constexpr double kAngularMax = 2.0;

struct AppState
{
    bool running {true};
    bool paused {false};
    bool reset_requested {false};
    bool print_help {false};
    double linear_x {0.0};
    double angular_z {0.0};
};

constexpr hako::robots::rover::DifferentialDriveKinematics kTracerKinematics {
    kWheelRadius,
    kWheelSeparation,
    -1.0,
    1.0,
    kMaxWheelAngularVelocity,
};

int find_joint_qpos_addr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        throw std::runtime_error(std::string("joint not found: ") + joint_name);
    }
    return model->jnt_qposadr[joint_id];
}

int find_joint_qvel_addr(const mjModel* model, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        throw std::runtime_error(std::string("joint not found: ") + joint_name);
    }
    return model->jnt_dofadr[joint_id];
}

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [model.xml] [left-config.json] [right-config.json]\n"
        << "  " << program << " --check [model.xml] [left-config.json] [right-config.json]\n\n"
        << "Default:\n"
        << "  model       : " << kDefaultModelPath << "\n"
        << "  left config : " << kDefaultLeftConfigPath << "\n"
        << "  right config: " << kDefaultRightConfigPath << "\n";
}

void print_help()
{
    std::cout << R"(
Controls:
  w / s  : increase / decrease linear.x target
  a / d  : increase / decrease angular.z target
  Space  : stop linear and angular targets
  r      : reset simulation state and targets
  p      : pause / resume physics
  h      : show help
  q / Esc: quit

Viewer:
  Use the mouse to rotate / zoom the MuJoCo viewer.
)" << std::endl;
}

void print_state(
    double sim_time,
    const mjData* data,
    int base_qpos_addr,
    int left_qvel_addr,
    int right_qvel_addr,
    double linear_x,
    double angular_z,
    double left_target,
    double right_target)
{
    std::cout
        << std::fixed << std::setprecision(3)
        << "time=" << std::setw(7) << sim_time
        << " base=("
        << std::setw(6) << data->qpos[base_qpos_addr + 0] << ", "
        << std::setw(6) << data->qpos[base_qpos_addr + 1] << ", "
        << std::setw(6) << data->qpos[base_qpos_addr + 2] << ")"
        << " cmd=("
        << std::setw(5) << linear_x << " m/s, "
        << std::setw(5) << angular_z << " rad/s)"
        << " wheel_target=("
        << std::setw(6) << left_target << ", "
        << std::setw(6) << right_target << ")"
        << " wheel_qvel=("
        << std::setw(6) << data->qvel[left_qvel_addr] << ", "
        << std::setw(6) << data->qvel[right_qvel_addr] << ")"
        << std::endl;
}

void handle_viewer_key(AppState& state, int key, int action)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state.running = false;
    } else if (key == GLFW_KEY_W) {
        state.linear_x = std::clamp(state.linear_x + kLinearStep, kLinearMin, kLinearMax);
    } else if (key == GLFW_KEY_S) {
        state.linear_x = std::clamp(state.linear_x - kLinearStep, kLinearMin, kLinearMax);
    } else if (key == GLFW_KEY_A) {
        state.angular_z = std::clamp(state.angular_z + kAngularStep, kAngularMin, kAngularMax);
    } else if (key == GLFW_KEY_D) {
        state.angular_z = std::clamp(state.angular_z - kAngularStep, kAngularMin, kAngularMax);
    } else if (key == GLFW_KEY_SPACE) {
        state.linear_x = 0.0;
        state.angular_z = 0.0;
    } else if (key == GLFW_KEY_R) {
        state.reset_requested = true;
    } else if (key == GLFW_KEY_P) {
        state.paused = !state.paused;
    } else if (key == GLFW_KEY_H) {
        state.print_help = true;
    }
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--help") {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    const bool check_only = argc > 1 && std::string(argv[1]) == "--check";
    const int arg_offset = check_only ? 1 : 0;
    const std::string model_path = argc > (1 + arg_offset) ? argv[1 + arg_offset] : kDefaultModelPath;
    const std::string left_config_path = argc > (2 + arg_offset) ? argv[2 + arg_offset] : kDefaultLeftConfigPath;
    const std::string right_config_path = argc > (3 + arg_offset) ? argv[3 + arg_offset] : kDefaultRightConfigPath;

    auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: failed to load model: " << model_path
                  << ": " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    auto left_actuator = world->createJointActuator();
    auto right_actuator = world->createJointActuator();

    if (!left_actuator->LoadConfig(left_config_path)) {
        std::cerr << "ERROR: failed to load left actuator config: "
                  << left_config_path << std::endl;
        return EXIT_FAILURE;
    }
    if (!right_actuator->LoadConfig(right_config_path)) {
        std::cerr << "ERROR: failed to load right actuator config: "
                  << right_config_path << std::endl;
        return EXIT_FAILURE;
    }

    mjModel* model = world->getModel();
    mjData* data = world->getData();
    const int base_qpos_addr = find_joint_qpos_addr(model, kBaseFreejointName);
    const int left_qvel_addr = find_joint_qvel_addr(model, kLeftWheelJointName);
    const int right_qvel_addr = find_joint_qvel_addr(model, kRightWheelJointName);

    if (check_only) {
        constexpr double kCheckLinearX = 0.2;
        constexpr double kCheckAngularZ = 0.0;
        const auto wheel_targets =
            hako::robots::rover::ToWheelVelocityTargets(
                kCheckLinearX,
                kCheckAngularZ,
                kTracerKinematics);

        for (int i = 0; i < 500; ++i) {
            left_actuator->SetTarget(wheel_targets.left);
            right_actuator->SetTarget(wheel_targets.right);
            world->advanceTimeStep();
        }

        const double forward_displacement = data->qpos[base_qpos_addr + 0];
        const bool finite_pose =
            std::isfinite(data->qpos[base_qpos_addr + 0]) &&
            std::isfinite(data->qpos[base_qpos_addr + 1]) &&
            std::isfinite(data->qpos[base_qpos_addr + 2]);
        print_state(
            data->time,
            data,
            base_qpos_addr,
            left_qvel_addr,
            right_qvel_addr,
            kCheckLinearX,
            kCheckAngularZ,
            wheel_targets.left,
            wheel_targets.right);
        if (!finite_pose) {
            std::cerr << "ERROR: base pose is not finite" << std::endl;
            return EXIT_FAILURE;
        }
        if (forward_displacement < 0.02) {
            std::cerr
                << "ERROR: forward displacement is too small: "
                << forward_displacement << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "check ok" << std::endl;
        return EXIT_SUCCESS;
    }

    AppState state {};
    bool viewer_running = true;
    std::mutex mujoco_mutex;

    std::cout << "AgileX Tracer Rover Actuator Example" << std::endl;
    std::cout << "model       : " << model_path << std::endl;
    std::cout << "left config : " << left_config_path << std::endl;
    std::cout << "right config: " << right_config_path << "\n" << std::endl;
    std::cout << "This MuJoCo-only sample uses linear.x/angular.z style targets,\n"
              << "converts them to left/right wheel velocity targets, and applies\n"
              << "the targets through JointActuatorImpl before adding Hakoniwa PDU integration.\n"
              << std::endl;
    print_help();

    int step = 0;
    MujocoRenderRuntime render_runtime(
        model,
        data,
        viewer_running,
        mujoco_mutex,
        MujocoRenderWindowMode::Visible);
    render_runtime.SetKeyCallback(
        [&](int key, int action, int mods) {
            (void)mods;
            handle_viewer_key(state, key, action);
            if (!state.running) {
                viewer_running = false;
            }
        });
    render_runtime.SetPreRenderCallback([&]() {
        if (!state.running) {
            viewer_running = false;
            return;
        }

        if (state.reset_requested) {
            mj_resetData(model, data);
            mj_forward(model, data);
            state.linear_x = 0.0;
            state.angular_z = 0.0;
            state.reset_requested = false;
            std::cout << "reset" << std::endl;
        }

        const auto wheel_targets =
            hako::robots::rover::ToWheelVelocityTargets(
                state.linear_x,
                state.angular_z,
                kTracerKinematics);

        left_actuator->SetTarget(wheel_targets.left);
        right_actuator->SetTarget(wheel_targets.right);
        if (!state.paused) {
            world->advanceTimeStep();
        }

        if (state.print_help) {
            print_help();
            state.print_help = false;
        }

        if ((step % 60) == 0) {
            print_state(
                data->time,
                data,
                base_qpos_addr,
                left_qvel_addr,
                right_qvel_addr,
                state.linear_x,
                state.angular_z,
                wheel_targets.left,
                wheel_targets.right);
        }
        ++step;
    });

    render_runtime.Run();
    std::cout << "bye" << std::endl;
    return EXIT_SUCCESS;
}
