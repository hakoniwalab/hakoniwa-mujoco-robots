#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <mujoco/mujoco.h>

#include "physics.hpp"

class HakoniwaMujocoContext {
public:
    struct ForkliftState {
        std::array<double, 7> base_qpos {};
        std::array<double, 6> base_qvel {};
        std::array<double, 6> base_qacc {};
        double lift_qpos {0.0};
        double lift_qvel {0.0};
        double lift_qacc {0.0};
        bool minimal_only {false};
        std::vector<double> act;
        std::vector<double> ctrl;
        std::vector<double> qacc_warmstart;
        std::vector<double> qfrc_applied;
        std::vector<double> xfrc_applied;
        std::vector<double> integration_state;
        std::vector<double> context_qpos;
        std::vector<double> context_qvel;
        std::vector<double> context_qacc;
    };
    struct ControlState {
        int phase {0};
        double target_linear_velocity {0.0};
        double target_yaw_rate {0.0};
        double target_lift_z {0.0};
        std::uint64_t sim_step {0};
        double lift_pid_integral {0.0};
        double lift_pid_prev_error {0.0};
        double drive_v_pid_integral {0.0};
        double drive_v_pid_prev_error {0.0};
        double drive_w_pid_integral {0.0};
        double drive_w_pid_prev_error {0.0};
    };

private:
    std::shared_ptr<hako::robots::physics::IWorld> world_;
    std::string state_file_path_;
    int autosave_steps_;

    struct ForkliftIndex {
        int base_qpos_adr {-1};
        int base_dof_adr {-1};
        int lift_qpos_adr {-1};
        int lift_dof_adr {-1};
        bool valid {false};
    };

    struct ForkliftContextIndex {
        std::vector<int> qpos_indices;
        std::vector<int> dof_indices;
        std::vector<int> body_indices;
        std::vector<int> actuator_indices;
        bool valid {false};
    };

    static int joint_qpos_width(int joint_type)
    {
        switch (joint_type) {
        case mjJNT_FREE:
            return 7;
        case mjJNT_BALL:
            return 4;
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            return 1;
        default:
            return 0;
        }
    }

    static int joint_dof_width(int joint_type)
    {
        switch (joint_type) {
        case mjJNT_FREE:
            return 6;
        case mjJNT_BALL:
            return 3;
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            return 1;
        default:
            return 0;
        }
    }

    ForkliftContextIndex resolve_forklift_context_index(const char* base_body_name) const
    {
        ForkliftContextIndex idx;
        if (world_ == nullptr || world_->getModel() == nullptr) {
            return idx;
        }
        const mjModel* model = world_->getModel();
        int base_body_id = mj_name2id(model, mjOBJ_BODY, base_body_name);
        if (base_body_id < 0) {
            return idx;
        }

        for (int b = 0; b < model->nbody; b++) {
            int cur = b;
            while (cur >= 0) {
                if (cur == base_body_id) {
                    idx.body_indices.push_back(b);
                    break;
                }
                if (cur == 0) {
                    break;
                }
                cur = model->body_parentid[cur];
            }
        }
        if (idx.body_indices.empty()) {
            return idx;
        }
        std::sort(idx.body_indices.begin(), idx.body_indices.end());

        std::unordered_set<int> qpos_set;
        std::unordered_set<int> dof_set;
        for (int b : idx.body_indices) {
            int jadr = model->body_jntadr[b];
            int jnum = model->body_jntnum[b];
            for (int j = jadr; j < jadr + jnum; j++) {
                int qadr = model->jnt_qposadr[j];
                int dadr = model->jnt_dofadr[j];
                int qn = joint_qpos_width(model->jnt_type[j]);
                int dn = joint_dof_width(model->jnt_type[j]);
                for (int k = 0; k < qn; k++) {
                    qpos_set.insert(qadr + k);
                }
                for (int k = 0; k < dn; k++) {
                    dof_set.insert(dadr + k);
                }
            }
        }
        idx.qpos_indices.assign(qpos_set.begin(), qpos_set.end());
        idx.dof_indices.assign(dof_set.begin(), dof_set.end());
        std::sort(idx.qpos_indices.begin(), idx.qpos_indices.end());
        std::sort(idx.dof_indices.begin(), idx.dof_indices.end());

        const char* actuator_names[] = {"left_motor", "right_motor", "lift_motor"};
        for (const char* name : actuator_names) {
            int aid = mj_name2id(model, mjOBJ_ACTUATOR, name);
            if (aid >= 0) {
                idx.actuator_indices.push_back(aid);
            }
        }
        idx.valid = !idx.qpos_indices.empty() && !idx.dof_indices.empty();
        return idx;
    }

    ForkliftIndex resolve_forklift_index(
        const char* base_body_name,
        const char* lift_joint_name) const
    {
        ForkliftIndex idx;
        if (world_ == nullptr || world_->getModel() == nullptr) {
            return idx;
        }
        const mjModel* model = world_->getModel();
        int base_body_id = mj_name2id(model, mjOBJ_BODY, base_body_name);
        if (base_body_id < 0) {
            return idx;
        }
        int base_joint_id = model->body_jntadr[base_body_id];
        if (base_joint_id < 0) {
            return idx;
        }
        idx.base_qpos_adr = model->jnt_qposadr[base_joint_id];
        idx.base_dof_adr = model->jnt_dofadr[base_joint_id];
        int lift_joint_id = mj_name2id(model, mjOBJ_JOINT, lift_joint_name);
        if (lift_joint_id < 0) {
            return idx;
        }
        idx.lift_qpos_adr = model->jnt_qposadr[lift_joint_id];
        idx.lift_dof_adr = model->jnt_dofadr[lift_joint_id];
        idx.valid = (idx.base_qpos_adr >= 0 && idx.base_dof_adr >= 0 &&
            idx.lift_qpos_adr >= 0 && idx.lift_dof_adr >= 0);
        return idx;
    }

    static void write_vector_line(std::ofstream& ofs, const std::vector<double>& values)
    {
        ofs << values.size();
        for (double v : values) {
            ofs << " " << v;
        }
        ofs << "\n";
    }

    static bool read_vector_line(std::ifstream& ifs, std::vector<double>& values)
    {
        std::size_t n = 0;
        if (!(ifs >> n)) {
            return false;
        }
        values.resize(n);
        for (std::size_t i = 0; i < n; i++) {
            if (!(ifs >> values[i])) {
                return false;
            }
        }
        return true;
    }

    static int integration_state_sig()
    {
        return mjSTATE_INTEGRATION;
    }

    bool capture_integration_state(std::vector<double>& out_state) const
    {
        if (world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        const mjData* data = world_->getData();
        const int sig = integration_state_sig();
        const int nstate = mj_stateSize(model, sig);
        if (nstate <= 0) {
            return false;
        }
        out_state.resize(static_cast<std::size_t>(nstate));
        mj_getState(model, data, out_state.data(), sig);
        return true;
    }

    bool apply_integration_state(const std::vector<double>& state) const
    {
        if (world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        mjData* data = world_->getData();
        const int sig = integration_state_sig();
        const int nstate = mj_stateSize(model, sig);
        if (nstate <= 0 || state.size() != static_cast<std::size_t>(nstate)) {
            return false;
        }
        mj_setState(model, data, state.data(), sig);
        // Recompute derived quantities after setting state.
        mj_forward(model, data);
        return true;
    }

    bool capture_minimal_forklift_state(
        ForkliftState& out_state,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftIndex idx = resolve_forklift_index(base_body_name, lift_joint_name);
        if (!idx.valid || world_ == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjData* data = world_->getData();
        for (int i = 0; i < 7; i++) {
            out_state.base_qpos[static_cast<std::size_t>(i)] = data->qpos[idx.base_qpos_adr + i];
        }
        for (int i = 0; i < 6; i++) {
            out_state.base_qvel[static_cast<std::size_t>(i)] = data->qvel[idx.base_dof_adr + i];
            out_state.base_qacc[static_cast<std::size_t>(i)] = 0.0;
        }
        out_state.lift_qpos = data->qpos[idx.lift_qpos_adr];
        out_state.lift_qvel = data->qvel[idx.lift_dof_adr];
        out_state.lift_qacc = 0.0;
        out_state.minimal_only = true;
        out_state.integration_state.clear();
        out_state.act.clear();
        out_state.ctrl.clear();
        out_state.qacc_warmstart.clear();
        out_state.qfrc_applied.clear();
        out_state.xfrc_applied.clear();
        return true;
    }

    bool apply_minimal_forklift_state(
        const ForkliftState& state,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftIndex idx = resolve_forklift_index(base_body_name, lift_joint_name);
        if (!idx.valid || world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        mjData* data = world_->getData();
        for (int i = 0; i < 7; i++) {
            data->qpos[idx.base_qpos_adr + i] = state.base_qpos[static_cast<std::size_t>(i)];
        }
        for (int i = 0; i < 6; i++) {
            data->qvel[idx.base_dof_adr + i] = state.base_qvel[static_cast<std::size_t>(i)];
            data->qacc[idx.base_dof_adr + i] = 0.0;
        }
        data->qpos[idx.lift_qpos_adr] = state.lift_qpos;
        data->qvel[idx.lift_dof_adr] = state.lift_qvel;
        data->qacc[idx.lift_dof_adr] = 0.0;
        mj_forward(model, data);
        return true;
    }

    bool capture_extended_forklift_state(
        ForkliftState& out_state,
        const char* base_body_name = "forklift_base") const
    {
        if (world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        const mjData* data = world_->getData();
        ForkliftContextIndex ctx = resolve_forklift_context_index(base_body_name);
        if (!ctx.valid) {
            return false;
        }

        out_state.context_qpos.clear();
        out_state.context_qvel.clear();
        out_state.context_qacc.clear();
        out_state.qacc_warmstart.clear();
        out_state.qfrc_applied.clear();
        out_state.xfrc_applied.clear();
        out_state.ctrl.clear();
        out_state.act.clear();

        out_state.context_qpos.reserve(ctx.qpos_indices.size());
        for (int q : ctx.qpos_indices) {
            out_state.context_qpos.push_back(data->qpos[q]);
        }
        out_state.context_qvel.reserve(ctx.dof_indices.size());
        out_state.context_qacc.reserve(ctx.dof_indices.size());
        out_state.qacc_warmstart.reserve(ctx.dof_indices.size());
        out_state.qfrc_applied.reserve(ctx.dof_indices.size());
        for (int d : ctx.dof_indices) {
            out_state.context_qvel.push_back(data->qvel[d]);
            out_state.context_qacc.push_back(data->qacc[d]);
            out_state.qacc_warmstart.push_back(data->qacc_warmstart[d]);
            out_state.qfrc_applied.push_back(data->qfrc_applied[d]);
        }
        out_state.xfrc_applied.reserve(ctx.body_indices.size() * 6);
        for (int b : ctx.body_indices) {
            for (int k = 0; k < 6; k++) {
                out_state.xfrc_applied.push_back(data->xfrc_applied[6 * b + k]);
            }
        }
        out_state.ctrl.reserve(ctx.actuator_indices.size());
        for (int aid : ctx.actuator_indices) {
            out_state.ctrl.push_back(data->ctrl[aid]);
        }
        out_state.act.assign(data->act, data->act + model->na);

        // Keep base/lift snapshots for debug and backward helper behavior.
        ForkliftIndex idx = resolve_forklift_index(base_body_name, "lift_joint");
        if (idx.valid) {
            for (int i = 0; i < 7; i++) {
                out_state.base_qpos[static_cast<std::size_t>(i)] = data->qpos[idx.base_qpos_adr + i];
            }
            for (int i = 0; i < 6; i++) {
                out_state.base_qvel[static_cast<std::size_t>(i)] = data->qvel[idx.base_dof_adr + i];
                out_state.base_qacc[static_cast<std::size_t>(i)] = data->qacc[idx.base_dof_adr + i];
            }
            out_state.lift_qpos = data->qpos[idx.lift_qpos_adr];
            out_state.lift_qvel = data->qvel[idx.lift_dof_adr];
            out_state.lift_qacc = data->qacc[idx.lift_dof_adr];
        }

        out_state.minimal_only = true;
        out_state.integration_state.clear();
        return true;
    }

    bool apply_extended_forklift_state(
        const ForkliftState& state,
        const char* base_body_name = "forklift_base") const
    {
        if (world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        mjData* data = world_->getData();
        ForkliftContextIndex ctx = resolve_forklift_context_index(base_body_name);
        if (!ctx.valid) {
            return false;
        }
        if (state.context_qpos.size() != ctx.qpos_indices.size() ||
            state.context_qvel.size() != ctx.dof_indices.size() ||
            state.context_qacc.size() != ctx.dof_indices.size() ||
            state.qacc_warmstart.size() != ctx.dof_indices.size() ||
            state.qfrc_applied.size() != ctx.dof_indices.size() ||
            state.xfrc_applied.size() != ctx.body_indices.size() * 6 ||
            state.ctrl.size() != ctx.actuator_indices.size()) {
            return false;
        }

        for (std::size_t i = 0; i < ctx.qpos_indices.size(); i++) {
            data->qpos[ctx.qpos_indices[i]] = state.context_qpos[i];
        }
        for (std::size_t i = 0; i < ctx.dof_indices.size(); i++) {
            data->qvel[ctx.dof_indices[i]] = state.context_qvel[i];
        }
        for (std::size_t i = 0; i < ctx.actuator_indices.size(); i++) {
            data->ctrl[ctx.actuator_indices[i]] = state.ctrl[i];
        }
        if (state.act.size() == static_cast<std::size_t>(model->na)) {
            for (int i = 0; i < model->na; i++) {
                data->act[i] = state.act[static_cast<std::size_t>(i)];
            }
        }
        for (std::size_t i = 0; i < ctx.dof_indices.size(); i++) {
            data->qfrc_applied[ctx.dof_indices[i]] = state.qfrc_applied[i];
        }
        for (std::size_t i = 0; i < ctx.body_indices.size(); i++) {
            int b = ctx.body_indices[i];
            for (int k = 0; k < 6; k++) {
                data->xfrc_applied[6 * b + k] = state.xfrc_applied[i * 6 + static_cast<std::size_t>(k)];
            }
        }

        mj_forward(model, data);

        for (std::size_t i = 0; i < ctx.dof_indices.size(); i++) {
            data->qacc[ctx.dof_indices[i]] = state.context_qacc[i];
            data->qacc_warmstart[ctx.dof_indices[i]] = state.qacc_warmstart[i];
        }
        return true;
    }

    bool decode_integration_state_forklift_snapshot(
        const std::vector<double>& integration_state,
        ForkliftState& out_state,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        if (world_ == nullptr || world_->getModel() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        ForkliftIndex idx = resolve_forklift_index(base_body_name, lift_joint_name);
        if (!idx.valid) {
            return false;
        }
        const int sig = integration_state_sig();
        const int total = mj_stateSize(model, sig);
        if (total <= 0 || integration_state.size() != static_cast<std::size_t>(total)) {
            return false;
        }
        const int size_time = mj_stateSize(model, mjSTATE_TIME);
        const int size_qpos = mj_stateSize(model, mjSTATE_QPOS);
        const int size_qvel = mj_stateSize(model, mjSTATE_QVEL);
        if (size_qpos <= 0 || size_qvel <= 0) {
            return false;
        }
        int off_qpos = size_time;
        int off_qvel = off_qpos + size_qpos;
        if (off_qvel + size_qvel > total) {
            return false;
        }
        for (int i = 0; i < 7; i++) {
            out_state.base_qpos[static_cast<std::size_t>(i)] =
                integration_state[static_cast<std::size_t>(off_qpos + idx.base_qpos_adr + i)];
        }
        for (int i = 0; i < 6; i++) {
            out_state.base_qvel[static_cast<std::size_t>(i)] =
                integration_state[static_cast<std::size_t>(off_qvel + idx.base_dof_adr + i)];
        }
        out_state.lift_qpos = integration_state[static_cast<std::size_t>(off_qpos + idx.lift_qpos_adr)];
        out_state.lift_qvel = integration_state[static_cast<std::size_t>(off_qvel + idx.lift_dof_adr)];
        // qacc is not part of mjSTATE_INTEGRATION.
        for (int i = 0; i < 6; i++) {
            out_state.base_qacc[static_cast<std::size_t>(i)] = 0.0;
        }
        out_state.lift_qacc = 0.0;
        out_state.minimal_only = false;
        return true;
    }

public:
    HakoniwaMujocoContext(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        const std::string& default_state_file_path,
        int default_autosave_steps = 1000)
        : world_(std::move(world))
        , state_file_path_(default_state_file_path)
        , autosave_steps_(default_autosave_steps)
    {
        const char* path_env = std::getenv("HAKO_FORKLIFT_STATE_FILE");
        if (path_env != nullptr && path_env[0] != '\0') {
            state_file_path_ = path_env;
        }
        const char* step_env = std::getenv("HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS");
        if (step_env != nullptr) {
            try {
                int v = std::stoi(step_env);
                if (v > 0) {
                    autosave_steps_ = v;
                }
            } catch (...) {
            }
        }
    }

    const std::string& state_file_path() const
    {
        return state_file_path_;
    }

    int autosave_steps() const
    {
        return autosave_steps_;
    }

    bool should_autosave(int step_count) const
    {
        return (autosave_steps_ > 0) && (step_count > 0) && ((step_count % autosave_steps_) == 0);
    }

    bool capture_forklift_state(
        ForkliftState& out_state,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftIndex idx = resolve_forklift_index(base_body_name, lift_joint_name);
        if (!idx.valid || world_ == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        if (model == nullptr) {
            return false;
        }
        const mjData* data = world_->getData();
        for (int i = 0; i < 7; i++) {
            out_state.base_qpos[static_cast<size_t>(i)] = data->qpos[idx.base_qpos_adr + i];
        }
        for (int i = 0; i < 6; i++) {
            out_state.base_qvel[static_cast<size_t>(i)] = data->qvel[idx.base_dof_adr + i];
            out_state.base_qacc[static_cast<size_t>(i)] = data->qacc[idx.base_dof_adr + i];
        }
        out_state.lift_qpos = data->qpos[idx.lift_qpos_adr];
        out_state.lift_qvel = data->qvel[idx.lift_dof_adr];
        out_state.lift_qacc = data->qacc[idx.lift_dof_adr];
        out_state.act.assign(data->act, data->act + model->na);
        out_state.ctrl.assign(data->ctrl, data->ctrl + model->nu);
        out_state.qacc_warmstart.assign(data->qacc_warmstart, data->qacc_warmstart + model->nv);
        out_state.qfrc_applied.assign(data->qfrc_applied, data->qfrc_applied + model->nv);
        out_state.xfrc_applied.assign(data->xfrc_applied, data->xfrc_applied + (model->nbody * 6));
        out_state.minimal_only = false;
        return true;
    }

    bool apply_forklift_state(
        const ForkliftState& state,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftIndex idx = resolve_forklift_index(base_body_name, lift_joint_name);
        if (!idx.valid || world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            return false;
        }
        const mjModel* model = world_->getModel();
        mjData* data = world_->getData();
        for (int i = 0; i < 7; i++) {
            data->qpos[idx.base_qpos_adr + i] = state.base_qpos[static_cast<size_t>(i)];
        }
        for (int i = 0; i < 6; i++) {
            data->qvel[idx.base_dof_adr + i] = state.base_qvel[static_cast<size_t>(i)];
            data->qacc[idx.base_dof_adr + i] = state.base_qacc[static_cast<size_t>(i)];
        }
        data->qpos[idx.lift_qpos_adr] = state.lift_qpos;
        data->qvel[idx.lift_dof_adr] = state.lift_qvel;
        if (state.act.size() == static_cast<std::size_t>(model->na)) {
            for (int i = 0; i < model->na; i++) {
                data->act[i] = state.act[static_cast<std::size_t>(i)];
            }
        }
        if (state.ctrl.size() == static_cast<std::size_t>(model->nu)) {
            for (int i = 0; i < model->nu; i++) {
                data->ctrl[i] = state.ctrl[static_cast<std::size_t>(i)];
            }
        }
        if (state.qfrc_applied.size() == static_cast<std::size_t>(model->nv)) {
            for (int i = 0; i < model->nv; i++) {
                data->qfrc_applied[i] = state.qfrc_applied[static_cast<std::size_t>(i)];
            }
        }
        if (state.xfrc_applied.size() == static_cast<std::size_t>(model->nbody * 6)) {
            for (int i = 0; i < model->nbody * 6; i++) {
                data->xfrc_applied[i] = state.xfrc_applied[static_cast<std::size_t>(i)];
            }
        }

        mj_forward(model, data);

        for (int i = 0; i < 6; i++) {
            data->qacc[idx.base_dof_adr + i] = state.base_qacc[static_cast<std::size_t>(i)];
        }
        data->qacc[idx.lift_dof_adr] = state.lift_qacc;
        if (state.qacc_warmstart.size() == static_cast<std::size_t>(model->nv)) {
            for (int i = 0; i < model->nv; i++) {
                data->qacc_warmstart[i] = state.qacc_warmstart[static_cast<std::size_t>(i)];
            }
        }
        if (state.qfrc_applied.size() == static_cast<std::size_t>(model->nv)) {
            for (int i = 0; i < model->nv; i++) {
                data->qfrc_applied[i] = state.qfrc_applied[static_cast<std::size_t>(i)];
            }
        }
        if (state.xfrc_applied.size() == static_cast<std::size_t>(model->nbody * 6)) {
            for (int i = 0; i < model->nbody * 6; i++) {
                data->xfrc_applied[i] = state.xfrc_applied[static_cast<std::size_t>(i)];
            }
        }
        return true;
    }

    bool save_forklift_state_with_control(
        const ControlState* control_state = nullptr,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        (void)lift_joint_name;
        ForkliftState partial_state;
        if (!capture_extended_forklift_state(partial_state, base_body_name)) {
            return false;
        }
        std::ofstream ofs(state_file_path_, std::ios::trunc);
        if (!ofs.is_open()) {
            return false;
        }
        ofs << std::setprecision(17);
        ofs << "v8\n";
        write_vector_line(ofs, partial_state.context_qpos);
        write_vector_line(ofs, partial_state.context_qvel);
        write_vector_line(ofs, partial_state.context_qacc);
        write_vector_line(ofs, partial_state.qacc_warmstart);
        write_vector_line(ofs, partial_state.qfrc_applied);
        write_vector_line(ofs, partial_state.xfrc_applied);
        write_vector_line(ofs, partial_state.ctrl);
        write_vector_line(ofs, partial_state.act);
        if (control_state != nullptr) {
            ofs << control_state->phase << " "
                << control_state->target_linear_velocity << " "
                << control_state->target_yaw_rate << " "
                << control_state->target_lift_z << " "
                << control_state->sim_step << " "
                << control_state->lift_pid_integral << " "
                << control_state->lift_pid_prev_error << " "
                << control_state->drive_v_pid_integral << " "
                << control_state->drive_v_pid_prev_error << " "
                << control_state->drive_w_pid_integral << " "
                << control_state->drive_w_pid_prev_error << "\n";
        } else {
            ofs << 0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0
                << " " << 0.0 << " " << 0.0
                << " " << 0.0 << " " << 0.0
                << " " << 0.0 << " " << 0.0 << "\n";
        }
        return true;
    }

    bool save_forklift_state(
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        return save_forklift_state_with_control(nullptr, base_body_name, lift_joint_name);
    }

    bool load_forklift_state(ForkliftState& out_state, ControlState* out_control_state = nullptr) const
    {
        std::ifstream ifs(state_file_path_);
        if (!ifs.is_open()) {
            return false;
        }
        std::string version;
        if (!(ifs >> version)) {
            return false;
        }
        out_state.act.clear();
        out_state.ctrl.clear();
        out_state.qacc_warmstart.clear();
        out_state.qfrc_applied.clear();
        out_state.xfrc_applied.clear();
        out_state.integration_state.clear();
        out_state.context_qpos.clear();
        out_state.context_qvel.clear();
        out_state.context_qacc.clear();
        out_state.minimal_only = false;
        ControlState loaded_control {};
        auto* control = (out_control_state != nullptr) ? out_control_state : &loaded_control;
        if (version == "v8") {
            if (!read_vector_line(ifs, out_state.context_qpos)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.context_qvel)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.context_qacc)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qacc_warmstart)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qfrc_applied)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.xfrc_applied)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.ctrl)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.act)) {
                return false;
            }
            out_state.minimal_only = true;
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step
                >> control->lift_pid_integral
                >> control->lift_pid_prev_error
                >> control->drive_v_pid_integral
                >> control->drive_v_pid_prev_error
                >> control->drive_w_pid_integral
                >> control->drive_w_pid_prev_error)) {
                return false;
            }
            return true;
        }
        if (version == "v7") {
            std::vector<double> base_qpos;
            std::vector<double> base_qvel;
            std::vector<double> lift_state;
            if (!read_vector_line(ifs, base_qpos) || base_qpos.size() != 7) {
                return false;
            }
            if (!read_vector_line(ifs, base_qvel) || base_qvel.size() != 6) {
                return false;
            }
            if (!read_vector_line(ifs, lift_state) || lift_state.size() != 2) {
                return false;
            }
            for (int i = 0; i < 7; i++) {
                out_state.base_qpos[static_cast<std::size_t>(i)] = base_qpos[static_cast<std::size_t>(i)];
            }
            for (int i = 0; i < 6; i++) {
                out_state.base_qvel[static_cast<std::size_t>(i)] = base_qvel[static_cast<std::size_t>(i)];
                out_state.base_qacc[static_cast<std::size_t>(i)] = 0.0;
            }
            out_state.lift_qpos = lift_state[0];
            out_state.lift_qvel = lift_state[1];
            out_state.lift_qacc = 0.0;
            out_state.minimal_only = true;
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step
                >> control->lift_pid_integral
                >> control->lift_pid_prev_error
                >> control->drive_v_pid_integral
                >> control->drive_v_pid_prev_error
                >> control->drive_w_pid_integral
                >> control->drive_w_pid_prev_error)) {
                return false;
            }
            return true;
        }
        if (version == "v6") {
            if (!read_vector_line(ifs, out_state.integration_state)) {
                return false;
            }
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step
                >> control->lift_pid_integral
                >> control->lift_pid_prev_error
                >> control->drive_v_pid_integral
                >> control->drive_v_pid_prev_error
                >> control->drive_w_pid_integral
                >> control->drive_w_pid_prev_error)) {
                return false;
            }
            (void)decode_integration_state_forklift_snapshot(out_state.integration_state, out_state);
            return true;
        }
        if (version == "v5") {
            for (int i = 0; i < 7; i++) {
                if (!(ifs >> out_state.base_qpos[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qvel[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qacc[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            if (!(ifs >> out_state.lift_qpos)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qvel)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qacc)) {
                return false;
            }
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step
                >> control->lift_pid_integral
                >> control->lift_pid_prev_error
                >> control->drive_v_pid_integral
                >> control->drive_v_pid_prev_error
                >> control->drive_w_pid_integral
                >> control->drive_w_pid_prev_error)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.act)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.ctrl)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qacc_warmstart)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qfrc_applied)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.xfrc_applied)) {
                return false;
            }
            return true;
        }
        if (version == "v4") {
            for (int i = 0; i < 7; i++) {
                if (!(ifs >> out_state.base_qpos[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qvel[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qacc[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            if (!(ifs >> out_state.lift_qpos)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qvel)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qacc)) {
                return false;
            }
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step)) {
                return false;
            }
            control->lift_pid_integral = 0.0;
            control->lift_pid_prev_error = 0.0;
            control->drive_v_pid_integral = 0.0;
            control->drive_v_pid_prev_error = 0.0;
            control->drive_w_pid_integral = 0.0;
            control->drive_w_pid_prev_error = 0.0;
            if (!read_vector_line(ifs, out_state.act)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.ctrl)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qacc_warmstart)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.qfrc_applied)) {
                return false;
            }
            if (!read_vector_line(ifs, out_state.xfrc_applied)) {
                return false;
            }
            return true;
        }
        if (version == "v3") {
            for (int i = 0; i < 7; i++) {
                if (!(ifs >> out_state.base_qpos[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qvel[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qacc[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            if (!(ifs >> out_state.lift_qpos)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qvel)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qacc)) {
                return false;
            }
            if (!(ifs >> control->phase
                >> control->target_linear_velocity
                >> control->target_yaw_rate
                >> control->target_lift_z
                >> control->sim_step)) {
                return false;
            }
            control->lift_pid_integral = 0.0;
            control->lift_pid_prev_error = 0.0;
            control->drive_v_pid_integral = 0.0;
            control->drive_v_pid_prev_error = 0.0;
            control->drive_w_pid_integral = 0.0;
            control->drive_w_pid_prev_error = 0.0;
            return true;
        }
        if (version == "v2") {
            for (int i = 0; i < 7; i++) {
                if (!(ifs >> out_state.base_qpos[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qvel[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            for (int i = 0; i < 6; i++) {
                if (!(ifs >> out_state.base_qacc[static_cast<size_t>(i)])) {
                    return false;
                }
            }
            if (!(ifs >> out_state.lift_qpos)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qvel)) {
                return false;
            }
            if (!(ifs >> out_state.lift_qacc)) {
                return false;
            }
            control->phase = 0;
            control->target_linear_velocity = 0.0;
            control->target_yaw_rate = 0.0;
            control->target_lift_z = 0.0;
            control->sim_step = 0;
            control->lift_pid_integral = 0.0;
            control->lift_pid_prev_error = 0.0;
            control->drive_v_pid_integral = 0.0;
            control->drive_v_pid_prev_error = 0.0;
            control->drive_w_pid_integral = 0.0;
            control->drive_w_pid_prev_error = 0.0;
            return true;
        }
        if (version == "v1") {
            double px, py, pz;
            double roll, pitch, yaw;
            double lift;
            if (!(ifs >> px >> py >> pz)) {
                return false;
            }
            if (!(ifs >> roll >> pitch >> yaw)) {
                return false;
            }
            if (!(ifs >> lift)) {
                return false;
            }
            out_state.base_qpos[0] = px;
            out_state.base_qpos[1] = py;
            out_state.base_qpos[2] = pz;
            mjtNum euler[3] = {roll, pitch, yaw};
            mjtNum quat[4];
            mju_euler2Quat(quat, euler, "XYZ");
            out_state.base_qpos[3] = quat[0];
            out_state.base_qpos[4] = quat[1];
            out_state.base_qpos[5] = quat[2];
            out_state.base_qpos[6] = quat[3];
            out_state.lift_qpos = lift;
            control->phase = 0;
            control->target_linear_velocity = 0.0;
            control->target_yaw_rate = 0.0;
            control->target_lift_z = 0.0;
            control->sim_step = 0;
            control->lift_pid_integral = 0.0;
            control->lift_pid_prev_error = 0.0;
            control->drive_v_pid_integral = 0.0;
            control->drive_v_pid_prev_error = 0.0;
            control->drive_w_pid_integral = 0.0;
            control->drive_w_pid_prev_error = 0.0;
            return true;
        }
        return false;
    }

    bool restore_forklift_state(
        ForkliftState* restored_state = nullptr,
        ControlState* restored_control_state = nullptr,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftState state;
        ControlState control;
        if (!load_forklift_state(state, &control)) {
            return false;
        }
        if (!state.integration_state.empty()) {
            if (!apply_integration_state(state.integration_state)) {
                return false;
            }
        } else if (state.minimal_only) {
            if (!state.context_qpos.empty()) {
                if (!apply_extended_forklift_state(state, base_body_name)) {
                    return false;
                }
            } else {
                if (!apply_minimal_forklift_state(state, base_body_name, lift_joint_name)) {
                    return false;
                }
            }
        } else {
            if (!apply_forklift_state(state, base_body_name, lift_joint_name)) {
                return false;
            }
        }
        if (restored_state != nullptr) {
            *restored_state = state;
        }
        if (restored_control_state != nullptr) {
            *restored_control_state = control;
        }
        return true;
    }
};
