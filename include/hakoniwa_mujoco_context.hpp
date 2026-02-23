#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

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
    };
    struct ControlState {
        int phase {0};
        double target_linear_velocity {0.0};
        double target_yaw_rate {0.0};
        double target_lift_z {0.0};
        std::uint64_t sim_step {0};
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
        data->qacc[idx.lift_dof_adr] = state.lift_qacc;
        mj_forward(world_->getModel(), data);
        return true;
    }

    bool save_forklift_state_with_control(
        const ControlState* control_state = nullptr,
        const char* base_body_name = "forklift_base",
        const char* lift_joint_name = "lift_joint") const
    {
        ForkliftState state;
        if (!capture_forklift_state(state, base_body_name, lift_joint_name)) {
            return false;
        }
        std::ofstream ofs(state_file_path_, std::ios::trunc);
        if (!ofs.is_open()) {
            return false;
        }
        ofs << std::setprecision(17);
        ofs << "v3\n";
        for (int i = 0; i < 7; i++) {
            if (i != 0) {
                ofs << " ";
            }
            ofs << state.base_qpos[static_cast<size_t>(i)];
        }
        ofs << "\n";
        for (int i = 0; i < 6; i++) {
            if (i != 0) {
                ofs << " ";
            }
            ofs << state.base_qvel[static_cast<size_t>(i)];
        }
        ofs << "\n";
        for (int i = 0; i < 6; i++) {
            if (i != 0) {
                ofs << " ";
            }
            ofs << state.base_qacc[static_cast<size_t>(i)];
        }
        ofs << "\n";
        ofs << state.lift_qpos << "\n";
        ofs << state.lift_qvel << "\n";
        ofs << state.lift_qacc << "\n";
        if (control_state != nullptr) {
            ofs << control_state->phase << " "
                << control_state->target_linear_velocity << " "
                << control_state->target_yaw_rate << " "
                << control_state->target_lift_z << " "
                << control_state->sim_step << "\n";
        } else {
            ofs << 0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0 << "\n";
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
            if (out_control_state != nullptr) {
                if (!(ifs >> out_control_state->phase
                    >> out_control_state->target_linear_velocity
                    >> out_control_state->target_yaw_rate
                    >> out_control_state->target_lift_z
                    >> out_control_state->sim_step)) {
                    return false;
                }
            }
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
        if (!apply_forklift_state(state, base_body_name, lift_joint_name)) {
            return false;
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
