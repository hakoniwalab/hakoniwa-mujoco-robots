#include "physics/physics_impl.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

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

mjModel* g_model = nullptr;
mjvCamera* g_camera = nullptr;
mjvScene* g_scene = nullptr;
bool g_mouse_button_left = false;
bool g_mouse_button_right = false;
bool g_mouse_shift_down = false;
double g_last_x = 0.0;
double g_last_y = 0.0;

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

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }

    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state == nullptr) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state->running = false;
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_A) {
        state->position_target = std::clamp(
            state->position_target - kPositionStep,
            kPositionMin,
            kPositionMax);
    } else if (key == GLFW_KEY_D) {
        state->position_target = std::clamp(
            state->position_target + kPositionStep,
            kPositionMin,
            kPositionMax);
    } else if (key == GLFW_KEY_J) {
        state->velocity_target = std::clamp(
            state->velocity_target - kVelocityStep,
            kVelocityMin,
            kVelocityMax);
    } else if (key == GLFW_KEY_L) {
        state->velocity_target = std::clamp(
            state->velocity_target + kVelocityStep,
            kVelocityMin,
            kVelocityMax);
    } else if (key == GLFW_KEY_SPACE) {
        state->velocity_target = 0.0;
    } else if (key == GLFW_KEY_R) {
        state->reset_requested = true;
    } else if (key == GLFW_KEY_P) {
        state->paused = !state->paused;
    } else if (key == GLFW_KEY_H) {
        state->print_help = true;
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    g_mouse_button_left = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
    g_mouse_button_right = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
    g_mouse_shift_down = (mods & GLFW_MOD_SHIFT) != 0;
    glfwGetCursorPos(window, &g_last_x, &g_last_y);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
    (void)window;
    if (!g_mouse_button_left && !g_mouse_button_right) {
        return;
    }

    const double dx = xpos - g_last_x;
    const double dy = ypos - g_last_y;
    g_last_x = xpos;
    g_last_y = ypos;

    int mode = mjMOUSE_MOVE_V;
    if (g_mouse_button_left) {
        mode = mjMOUSE_ROTATE_H;
    } else if (g_mouse_shift_down && g_mouse_button_right) {
        mode = mjMOUSE_ZOOM;
    }

    if (g_model != nullptr && g_scene != nullptr && g_camera != nullptr) {
        mjv_moveCamera(g_model, mode, dx / 200.0, dy / 200.0, g_scene, g_camera);
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    if (g_model != nullptr && g_scene != nullptr && g_camera != nullptr) {
        mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, 0.05 * yoffset, g_scene, g_camera);
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

    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed." << std::endl;
        return EXIT_FAILURE;
    }

    GLFWwindow* window = glfwCreateWindow(900, 650, "Hakoniwa Joint Actuator Example", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        std::cerr << "GLFW window creation failed." << std::endl;
        return EXIT_FAILURE;
    }

    AppState state {};
    glfwSetWindowUserPointer(window, &state);
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    camera.azimuth = 135.0;
    camera.elevation = -25.0;
    camera.distance = 2.4;
    camera.lookat[0] = 0.0;
    camera.lookat[1] = 0.0;
    camera.lookat[2] = 0.45;
    mjv_makeScene(model, &scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);
    g_model = model;
    g_camera = &camera;
    g_scene = &scene;

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
    while (state.running && !glfwWindowShouldClose(window)) {
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

        int framebuffer_width = 0;
        int framebuffer_height = 0;
        glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
        const mjrRect viewport = {0, 0, framebuffer_width, framebuffer_height};

        mjv_updateScene(
            model,
            data,
            &option,
            nullptr,
            &camera,
            mjCAT_ALL,
            &scene);
        mjr_render(viewport, &scene, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();

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
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    g_model = nullptr;
    g_camera = nullptr;
    g_scene = nullptr;
    glfwDestroyWindow(window);
    glfwTerminate();
    std::cout << "bye" << std::endl;
    return EXIT_SUCCESS;
}
