#include "viewer/mujoco_viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kDefaultModelPath =
    "thirdparty/mujoco_menagerie/unitree_go1/scene.xml";
constexpr const char* kHomeKeyName = "home";
constexpr double kPerturbationRad = 0.12;
constexpr int kHomeSteps = 250;
constexpr int kPerturbSteps = 250;

constexpr std::array<const char*, 12> kActuatorNames {
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
};

struct JointBinding
{
    const char* actuator_name;
    std::string joint_name;
    int actuator_id {-1};
    int joint_id {-1};
    int qpos_addr {-1};
    int qvel_addr {-1};
};

struct AppState
{
    bool running {true};
    bool paused {false};
    bool reset_requested {false};
    bool print_help {false};
    bool perturb {false};
};

struct ModelDeleter
{
    void operator()(mjModel* model) const
    {
        if (model != nullptr) {
            mj_deleteModel(model);
        }
    }
};

struct DataDeleter
{
    void operator()(mjData* data) const
    {
        if (data != nullptr) {
            mj_deleteData(data);
        }
    }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [model.xml]\n"
        << "  " << program << " --check [model.xml]\n\n"
        << "Default:\n"
        << "  model: " << kDefaultModelPath << "\n";
}

void print_help()
{
    std::cout << R"(
Controls:
  t      : toggle small diagonal thigh perturbation
  h      : return to home ctrl values
  r      : reset simulation to the Menagerie home keyframe
  p      : pause / resume physics
  ?      : show help
  q / Esc: quit

Viewer:
  Use the mouse to rotate / zoom the MuJoCo viewer.
)" << std::endl;
}

ModelPtr load_model(const std::string& model_path)
{
    char error[1024] {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr) {
        throw std::runtime_error(std::string("failed to load model: ") + error);
    }
    return ModelPtr(model);
}

std::vector<JointBinding> bind_joints(const mjModel* model)
{
    std::vector<JointBinding> bindings;
    bindings.reserve(kActuatorNames.size());

    for (const char* actuator_name : kActuatorNames) {
        const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name);
        if (actuator_id < 0) {
            throw std::runtime_error(std::string("actuator not found: ") + actuator_name);
        }

        const int joint_id = model->actuator_trnid[2 * actuator_id + 0];
        if (joint_id < 0) {
            throw std::runtime_error(std::string("actuator has no joint target: ") + actuator_name);
        }

        const char* joint_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
        bindings.push_back(JointBinding {
            actuator_name,
            joint_name != nullptr ? joint_name : "",
            actuator_id,
            joint_id,
            model->jnt_qposadr[joint_id],
            model->jnt_dofadr[joint_id],
        });
    }

    return bindings;
}

int find_home_key(const mjModel* model)
{
    const int key_id = mj_name2id(model, mjOBJ_KEY, kHomeKeyName);
    if (key_id < 0) {
        throw std::runtime_error(std::string("keyframe not found: ") + kHomeKeyName);
    }
    return key_id;
}

std::vector<double> read_home_ctrl(const mjModel* model, int key_id)
{
    std::vector<double> ctrl(static_cast<std::size_t>(model->nu), 0.0);
    const int offset = key_id * model->nu;
    for (int i = 0; i < model->nu; ++i) {
        ctrl[static_cast<std::size_t>(i)] = model->key_ctrl[offset + i];
    }
    return ctrl;
}

void reset_to_home(const mjModel* model, mjData* data, int key_id, const std::vector<double>& home_ctrl)
{
    mj_resetDataKeyframe(model, data, key_id);
    for (int i = 0; i < model->nu; ++i) {
        data->ctrl[i] = home_ctrl[static_cast<std::size_t>(i)];
    }
    mj_forward(model, data);
}

double clamped_ctrl(const mjModel* model, int actuator_id, double value)
{
    if (model->actuator_ctrllimited[actuator_id]) {
        const double lo = model->actuator_ctrlrange[2 * actuator_id + 0];
        const double hi = model->actuator_ctrlrange[2 * actuator_id + 1];
        return std::clamp(value, lo, hi);
    }
    return value;
}

void apply_targets(
    const mjModel* model,
    mjData* data,
    const std::vector<JointBinding>& bindings,
    const std::vector<double>& home_ctrl,
    bool perturb)
{
    for (const auto& binding : bindings) {
        double target = home_ctrl[static_cast<std::size_t>(binding.actuator_id)];
        if (perturb &&
            (std::string(binding.actuator_name) == "FR_thigh" ||
             std::string(binding.actuator_name) == "RL_thigh")) {
            target += kPerturbationRad;
        }
        data->ctrl[binding.actuator_id] = clamped_ctrl(model, binding.actuator_id, target);
    }
}

bool all_finite(const mjModel* model, const mjData* data)
{
    for (int i = 0; i < model->nq; ++i) {
        if (!std::isfinite(data->qpos[i])) {
            return false;
        }
    }
    for (int i = 0; i < model->nv; ++i) {
        if (!std::isfinite(data->qvel[i])) {
            return false;
        }
    }
    return true;
}

void print_mapping(const mjModel* model, const std::vector<JointBinding>& bindings)
{
    std::cout << "actuator mapping:" << std::endl;
    for (const auto& binding : bindings) {
        const double lo = model->actuator_ctrlrange[2 * binding.actuator_id + 0];
        const double hi = model->actuator_ctrlrange[2 * binding.actuator_id + 1];
        std::cout
            << "  [" << std::setw(2) << binding.actuator_id << "] "
            << std::setw(9) << binding.actuator_name
            << " -> " << std::setw(15) << binding.joint_name
            << " qpos=" << std::setw(2) << binding.qpos_addr
            << " qvel=" << std::setw(2) << binding.qvel_addr
            << " ctrlrange=[" << lo << ", " << hi << "]"
            << std::endl;
    }
}

void print_state(
    const mjData* data,
    const std::vector<JointBinding>& bindings,
    bool perturb)
{
    auto joint_pos = [&](const char* actuator_name) {
        const auto it = std::find_if(
            bindings.begin(),
            bindings.end(),
            [&](const JointBinding& binding) {
                return std::string(binding.actuator_name) == actuator_name;
            });
        return it != bindings.end() ? data->qpos[it->qpos_addr] : 0.0;
    };

    std::cout
        << std::fixed << std::setprecision(3)
        << "time=" << std::setw(6) << data->time
        << " base=("
        << std::setw(6) << data->qpos[0] << ", "
        << std::setw(6) << data->qpos[1] << ", "
        << std::setw(6) << data->qpos[2] << ")"
        << " mode=" << (perturb ? "perturb" : "home")
        << " FR_thigh=" << std::setw(6) << joint_pos("FR_thigh")
        << " RL_thigh=" << std::setw(6) << joint_pos("RL_thigh")
        << std::endl;
}

void handle_viewer_key(AppState& state, int key, int action)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
        state.running = false;
    } else if (key == GLFW_KEY_T) {
        state.perturb = !state.perturb;
    } else if (key == GLFW_KEY_H) {
        state.perturb = false;
    } else if (key == GLFW_KEY_R) {
        state.reset_requested = true;
    } else if (key == GLFW_KEY_P) {
        state.paused = !state.paused;
    } else if (key == GLFW_KEY_SLASH || key == GLFW_KEY_H) {
        state.print_help = true;
    }
}

int run_check(
    const mjModel* model,
    mjData* data,
    const std::vector<JointBinding>& bindings,
    int key_id,
    const std::vector<double>& home_ctrl)
{
    reset_to_home(model, data, key_id, home_ctrl);
    apply_targets(model, data, bindings, home_ctrl, false);
    for (int i = 0; i < kHomeSteps; ++i) {
        mj_step(model, data);
    }

    const double fr_before = data->qpos[bindings[1].qpos_addr];
    const double rl_before = data->qpos[bindings[10].qpos_addr];

    apply_targets(model, data, bindings, home_ctrl, true);
    for (int i = 0; i < kPerturbSteps; ++i) {
        mj_step(model, data);
    }

    const double fr_after = data->qpos[bindings[1].qpos_addr];
    const double rl_after = data->qpos[bindings[10].qpos_addr];
    print_state(data, bindings, true);

    if (!all_finite(model, data)) {
        std::cerr << "ERROR: qpos or qvel contains non-finite values" << std::endl;
        return EXIT_FAILURE;
    }
    if (data->qpos[2] < 0.20 || data->qpos[2] > 0.35) {
        std::cerr << "ERROR: base z is outside expected smoke range: "
                  << data->qpos[2] << std::endl;
        return EXIT_FAILURE;
    }
    if ((fr_after - fr_before) < 0.05 || (rl_after - rl_before) < 0.05) {
        std::cerr
            << "ERROR: perturbation did not move the expected thigh joints: "
            << "FR delta=" << (fr_after - fr_before)
            << ", RL delta=" << (rl_after - rl_before)
            << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "check ok" << std::endl;
    return EXIT_SUCCESS;
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

    ModelPtr model;
    try {
        model = load_model(model_path);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    DataPtr data(mj_makeData(model.get()));
    if (!data) {
        std::cerr << "ERROR: failed to allocate mjData" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<JointBinding> bindings;
    int key_id = -1;
    try {
        bindings = bind_joints(model.get());
        key_id = find_home_key(model.get());
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    const std::vector<double> home_ctrl = read_home_ctrl(model.get(), key_id);
    reset_to_home(model.get(), data.get(), key_id, home_ctrl);

    std::cout << "Unitree Go1 Joint I/O Example" << std::endl;
    std::cout << "model: " << model_path << std::endl;
    std::cout
        << "nq=" << model->nq
        << " nv=" << model->nv
        << " nu=" << model->nu
        << " nbody=" << model->nbody
        << " njnt=" << model->njnt
        << " ngeom=" << model->ngeom
        << " nkey=" << model->nkey
        << " timestep=" << model->opt.timestep
        << std::endl;
    print_mapping(model.get(), bindings);

    if (model->nu != static_cast<int>(kActuatorNames.size())) {
        std::cerr << "ERROR: expected 12 actuators, got " << model->nu << std::endl;
        return EXIT_FAILURE;
    }

    if (check_only) {
        return run_check(model.get(), data.get(), bindings, key_id, home_ctrl);
    }

    AppState state {};
    bool viewer_running = true;
    std::mutex mujoco_mutex;
    int step = 0;

    print_help();

    MujocoRenderRuntime render_runtime(
        model.get(),
        data.get(),
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
            reset_to_home(model.get(), data.get(), key_id, home_ctrl);
            state.perturb = false;
            state.reset_requested = false;
            std::cout << "reset to home" << std::endl;
        }

        apply_targets(model.get(), data.get(), bindings, home_ctrl, state.perturb);
        if (!state.paused) {
            mj_step(model.get(), data.get());
        }

        if (state.print_help) {
            print_help();
            state.print_help = false;
        }

        if ((step % 60) == 0) {
            print_state(data.get(), bindings, state.perturb);
        }
        ++step;
    });

    render_runtime.Run();
    std::cout << "bye" << std::endl;
    return EXIT_SUCCESS;
}
