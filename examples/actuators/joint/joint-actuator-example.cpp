#include "physics/physics_impl.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kDefaultModelPath =
    "models/actuators/joint/position-velocity-actuator-sample.xml";
constexpr const char* kDefaultPositionConfigPath =
    "config/actuator/joint/sample_position_actuator.json";
constexpr const char* kDefaultVelocityConfigPath =
    "config/actuator/joint/sample_velocity_actuator.json";

constexpr double kPositionStep = 0.1;
constexpr double kVelocityStep = 0.5;
constexpr double kPositionMin = -0.8;
constexpr double kPositionMax = 0.8;
constexpr double kVelocityMin = -4.0;
constexpr double kVelocityMax = 4.0;

struct AppState
{
    bool running {true};
    bool paused {false};
    bool reset_requested {false};
    bool print_help {false};
    double position_target {0.0};
    double velocity_target {0.0};
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
        << "  " << program << " [model.xml] [position-config.json] [velocity-config.json]\n\n"
        << "Default:\n"
        << "  model          : " << kDefaultModelPath << "\n"
        << "  position config: " << kDefaultPositionConfigPath << "\n"
        << "  velocity config: " << kDefaultVelocityConfigPath << "\n";
}

void print_help()
{
    std::cout << R"(
Controls:
  a / d  : decrease / increase position target
  j / l  : decrease / increase velocity target
  Space  : stop velocity target
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
    double position_target,
    double position_angle,
    double velocity_target,
    double velocity_qvel)
{
    std::cout
        << std::fixed << std::setprecision(3)
        << "time=" << std::setw(6) << sim_time
        << " pos_target=" << std::setw(6) << position_target
        << " pos_angle=" << std::setw(7) << position_angle
        << " vel_target=" << std::setw(6) << velocity_target
        << " vel_qvel=" << std::setw(7) << velocity_qvel
        << std::endl;
}

void handle_viewer_key(AppState& state, int key, int action)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state.running = false;
    } else if (key == GLFW_KEY_A) {
        state.position_target = std::clamp(
            state.position_target - kPositionStep,
            kPositionMin,
            kPositionMax);
    } else if (key == GLFW_KEY_D) {
        state.position_target = std::clamp(
            state.position_target + kPositionStep,
            kPositionMin,
            kPositionMax);
    } else if (key == GLFW_KEY_J) {
        state.velocity_target = std::clamp(
            state.velocity_target - kVelocityStep,
            kVelocityMin,
            kVelocityMax);
    } else if (key == GLFW_KEY_L) {
        state.velocity_target = std::clamp(
            state.velocity_target + kVelocityStep,
            kVelocityMin,
            kVelocityMax);
    } else if (key == GLFW_KEY_SPACE) {
        state.velocity_target = 0.0;
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

    const std::string model_path = argc > 1 ? argv[1] : kDefaultModelPath;
    const std::string position_config_path = argc > 2 ? argv[2] : kDefaultPositionConfigPath;
    const std::string velocity_config_path = argc > 3 ? argv[3] : kDefaultVelocityConfigPath;

    auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: failed to load model: " << model_path
                  << ": " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    auto position_actuator = world->createJointActuator();
    auto velocity_actuator = world->createJointActuator();

    if (!position_actuator->LoadConfig(position_config_path)) {
        std::cerr << "ERROR: failed to load position actuator config: "
                  << position_config_path << std::endl;
        return EXIT_FAILURE;
    }
    if (!velocity_actuator->LoadConfig(velocity_config_path)) {
        std::cerr << "ERROR: failed to load velocity actuator config: "
                  << velocity_config_path << std::endl;
        return EXIT_FAILURE;
    }

    mjModel* model = world->getModel();
    mjData* data = world->getData();
    const int position_qpos_addr = find_joint_qpos_addr(model, "position_hinge");
    const int velocity_qvel_addr = find_joint_qvel_addr(model, "velocity_hinge");

    AppState state {};
    bool viewer_running = true;
    std::mutex mujoco_mutex;

    std::cout << "Hakoniwa Joint Actuator Example" << std::endl;
    std::cout << "model          : " << model_path << std::endl;
    std::cout << "position config: " << position_config_path << std::endl;
    std::cout << "velocity config: " << velocity_config_path << "\n" << std::endl;
    std::cout << "This example uses MJCF <position> and <velocity> actuators directly.\n"
              << "JointActuatorImpl only loads JSON, resolves the MJCF actuator, clamps the target,\n"
              << "and writes the target to data->ctrl[actuator_id].\n"
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
            state.position_target = 0.0;
            state.velocity_target = 0.0;
            state.reset_requested = false;
            std::cout << "reset" << std::endl;
        }

        position_actuator->SetTarget(state.position_target);
        velocity_actuator->SetTarget(state.velocity_target);
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
                state.position_target,
                data->qpos[position_qpos_addr],
                state.velocity_target,
                data->qvel[velocity_qvel_addr]);
        }
        ++step;
    });

    render_runtime.Run();
    std::cout << "bye" << std::endl;
    return EXIT_SUCCESS;
}
