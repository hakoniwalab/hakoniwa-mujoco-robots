#include "actuator/named_actuator_impl.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kDefaultModelPath =
    "thirdparty/mujoco_menagerie/shadow_hand/scene_right.xml";
constexpr const char* kLicensePath =
    "thirdparty/mujoco_menagerie/shadow_hand/LICENSE";
constexpr int kSmokeSteps = 250;

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

struct ActuatorRow
{
    int id {-1};
    std::string name;
    int trntype {-1};
    std::string trntype_name;
    int target_id {-1};
    std::string target_name;
    bool position_compatible {false};
    bool ctrl_limited {false};
    double ctrl_lower {0.0};
    double ctrl_upper {0.0};
    double open_target {0.0};
    double close_target {0.0};
};

const char* safe_name(const mjModel* model, int object_type, int id)
{
    const char* name = mj_id2name(model, object_type, id);
    return name != nullptr ? name : "";
}

std::string trntype_name(int trntype)
{
    switch (trntype) {
    case mjTRN_JOINT:
        return "joint";
    case mjTRN_JOINTINPARENT:
        return "joint_in_parent";
    case mjTRN_SLIDERCRANK:
        return "slider_crank";
    case mjTRN_TENDON:
        return "tendon";
    case mjTRN_SITE:
        return "site";
    case mjTRN_BODY:
        return "body";
    default:
        return "unknown";
    }
}

int trntype_object_type(int trntype)
{
    switch (trntype) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
        return mjOBJ_JOINT;
    case mjTRN_TENDON:
        return mjOBJ_TENDON;
    case mjTRN_SITE:
        return mjOBJ_SITE;
    case mjTRN_BODY:
        return mjOBJ_BODY;
    default:
        return -1;
    }
}

bool is_position_actuator(const mjModel* model, int actuator_id)
{
    constexpr double kEpsilon = 1.0e-12;
    const int bias_offset = 10 * actuator_id;
    return model->actuator_biastype[actuator_id] == mjBIAS_AFFINE &&
           std::abs(model->actuator_biasprm[bias_offset + 1]) > kEpsilon;
}

double clamp_ctrl(const ActuatorRow& row, double value)
{
    if (!row.ctrl_limited) {
        return value;
    }
    return std::clamp(value, row.ctrl_lower, row.ctrl_upper);
}

bool is_wrist_or_spread(const std::string& name)
{
    return name.find("WRJ") != std::string::npos ||
           name.find("J4") != std::string::npos ||
           name.find("LFJ5") != std::string::npos;
}

double open_candidate(const ActuatorRow& row)
{
    return clamp_ctrl(row, 0.0);
}

double close_candidate(const ActuatorRow& row)
{
    if (is_wrist_or_spread(row.name)) {
        return open_candidate(row);
    }
    if (!row.ctrl_limited) {
        return 0.5;
    }
    const double base = std::clamp(0.0, row.ctrl_lower, row.ctrl_upper);
    return clamp_ctrl(row, base + 0.55 * (row.ctrl_upper - base));
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

std::vector<ActuatorRow> collect_actuators(const mjModel* model)
{
    std::vector<ActuatorRow> rows;
    rows.reserve(static_cast<std::size_t>(model->nu));

    for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
        ActuatorRow row;
        row.id = actuator_id;
        row.name = safe_name(model, mjOBJ_ACTUATOR, actuator_id);
        row.trntype = model->actuator_trntype[actuator_id];
        row.trntype_name = trntype_name(row.trntype);
        row.target_id = model->actuator_trnid[2 * actuator_id + 0];
        const int target_type = trntype_object_type(row.trntype);
        row.target_name = target_type >= 0 ? safe_name(model, target_type, row.target_id) : "";
        row.position_compatible = is_position_actuator(model, actuator_id);
        row.ctrl_limited = model->actuator_ctrllimited[actuator_id] != 0;
        row.ctrl_lower = model->actuator_ctrlrange[2 * actuator_id + 0];
        row.ctrl_upper = model->actuator_ctrlrange[2 * actuator_id + 1];
        row.open_target = open_candidate(row);
        row.close_target = close_candidate(row);
        rows.push_back(row);
    }
    return rows;
}

void print_license_preflight()
{
    std::cout << "license preflight:" << std::endl;
    std::cout << "  model-local license: " << kLicensePath << std::endl;
    std::cout << "  present: "
              << (std::filesystem::exists(kLicensePath) ? "yes" : "no")
              << std::endl;
    std::cout << "  expected: Apache-2.0, Copyright 2022 Shadow Robot Company Ltd"
              << std::endl;
}

void print_model_summary(const mjModel* model)
{
    std::cout
        << "model summary:"
        << " nq=" << model->nq
        << " nv=" << model->nv
        << " nu=" << model->nu
        << " njnt=" << model->njnt
        << " ntendon=" << model->ntendon
        << " nbody=" << model->nbody
        << " ngeom=" << model->ngeom
        << std::endl;
}

void print_actuator_table(const std::vector<ActuatorRow>& rows)
{
    std::cout << "\nactuator command table:" << std::endl;
    std::cout
        << "  idx | actuator_name  | trn    | target      | ctrlrange           | open   | close"
        << std::endl;

    for (const auto& row : rows) {
        std::cout
            << "  " << std::setw(3) << row.id << " | "
            << std::setw(14) << row.name << " | "
            << std::setw(6) << row.trntype_name << " | "
            << std::setw(11) << row.target_name << " | "
            << "[" << std::setw(8) << row.ctrl_lower << ", "
            << std::setw(8) << row.ctrl_upper << "] | "
            << std::setw(6) << row.open_target << " | "
            << std::setw(6) << row.close_target
            << std::endl;
    }
}

void print_joint_table(const mjModel* model)
{
    std::cout << "\nobservation joint table:" << std::endl;
    std::cout
        << "  id | joint_name | qpos | qvel | range"
        << std::endl;
    for (int joint_id = 0; joint_id < model->njnt; ++joint_id) {
        std::cout
            << "  " << std::setw(2) << joint_id << " | "
            << std::setw(9) << safe_name(model, mjOBJ_JOINT, joint_id) << " | "
            << std::setw(4) << model->jnt_qposadr[joint_id] << " | "
            << std::setw(4) << model->jnt_dofadr[joint_id] << " | ";
        if (model->jnt_limited[joint_id]) {
            std::cout
                << "[" << model->jnt_range[2 * joint_id + 0]
                << ", " << model->jnt_range[2 * joint_id + 1] << "]";
        } else {
            std::cout << "unlimited";
        }
        std::cout << std::endl;
    }
}

void print_tendon_table(const mjModel* model)
{
    std::cout << "\ntendon coupling table:" << std::endl;
    for (int tendon_id = 0; tendon_id < model->ntendon; ++tendon_id) {
        std::cout
            << "  [" << tendon_id << "] "
            << safe_name(model, mjOBJ_TENDON, tendon_id)
            << " joints=";
        const int adr = model->tendon_adr[tendon_id];
        const int num = model->tendon_num[tendon_id];
        bool first = true;
        for (int i = 0; i < num; ++i) {
            const int wrap_id = adr + i;
            if (model->wrap_type[wrap_id] != mjWRAP_JOINT) {
                continue;
            }
            if (!first) {
                std::cout << ", ";
            }
            const int joint_id = model->wrap_objid[wrap_id];
            std::cout
                << safe_name(model, mjOBJ_JOINT, joint_id)
                << "(coef=" << model->wrap_prm[wrap_id] << ")";
            first = false;
        }
        if (first) {
            std::cout << "-";
        }
        std::cout << std::endl;
    }
}

void apply_targets(mjData* data, const std::vector<ActuatorRow>& rows, bool close)
{
    for (const auto& row : rows) {
        data->ctrl[row.id] = close ? row.close_target : row.open_target;
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

double joint_qpos(const mjModel* model, const mjData* data, const char* joint_name)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    if (joint_id < 0) {
        return 0.0;
    }
    return data->qpos[model->jnt_qposadr[joint_id]];
}

void print_representative_state(const mjModel* model, const mjData* data, const char* label)
{
    std::cout
        << std::fixed << std::setprecision(3)
        << label
        << " time=" << data->time
        << " FFJ2=" << joint_qpos(model, data, "rh_FFJ2")
        << " FFJ1=" << joint_qpos(model, data, "rh_FFJ1")
        << " MFJ2=" << joint_qpos(model, data, "rh_MFJ2")
        << " MFJ1=" << joint_qpos(model, data, "rh_MFJ1")
        << " THJ2=" << joint_qpos(model, data, "rh_THJ2")
        << " THJ1=" << joint_qpos(model, data, "rh_THJ1")
        << std::endl;
}

bool run_check(const mjModel* model, mjData* data, const std::vector<ActuatorRow>& rows)
{
    mj_resetData(model, data);
    apply_targets(data, rows, false);
    mj_forward(model, data);
    print_representative_state(model, data, "open ");
    for (int i = 0; i < kSmokeSteps; ++i) {
        mj_step(model, data);
    }
    if (!all_finite(model, data)) {
        std::cerr << "[ERROR] non-finite qpos/qvel during open smoke" << std::endl;
        return false;
    }

    apply_targets(data, rows, true);
    for (int i = 0; i < kSmokeSteps; ++i) {
        mj_step(model, data);
    }
    if (!all_finite(model, data)) {
        std::cerr << "[ERROR] non-finite qpos/qvel during close smoke" << std::endl;
        return false;
    }
    print_representative_state(model, data, "close");
    return true;
}

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [--check] [model.xml]\n\n"
        << "Default model:\n"
        << "  " << kDefaultModelPath << "\n";
}

}  // namespace

int main(int argc, char* argv[])
{
    bool check = false;
    std::string model_path = kDefaultModelPath;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "--check") {
            check = true;
            continue;
        }
        model_path = arg;
    }

    try {
        print_license_preflight();
        auto model = load_model(model_path);
        auto data = DataPtr(mj_makeData(model.get()));
        if (data == nullptr) {
            throw std::runtime_error("failed to allocate mjData");
        }

        mj_forward(model.get(), data.get());
        const auto actuator_rows = collect_actuators(model.get());

        print_model_summary(model.get());
        print_actuator_table(actuator_rows);
        print_joint_table(model.get());
        print_tendon_table(model.get());

        if (check) {
            const bool ok = run_check(model.get(), data.get(), actuator_rows);
            std::cout << (ok ? "check ok" : "check failed") << std::endl;
            return ok ? 0 : 1;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
}
