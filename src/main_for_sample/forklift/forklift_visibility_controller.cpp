#include "forklift_visibility_controller.hpp"

#include <algorithm>
#include <unordered_set>

#include <mujoco/mujoco.h>

bool ForkliftVisibilityController::initialize(mjModel* model, mjData* data, const char* base_body_name)
{
    if (initialized_ || model == nullptr || base_body_name == nullptr) {
        return false;
    }
    model_ = model;
    data_ = data;
    const int base_body_id = mj_name2id(model_, mjOBJ_BODY, base_body_name);
    if (base_body_id < 0) {
        return false;
    }
    std::unordered_set<int> subtree_body_ids;
    for (int b = 0; b < model_->nbody; b++) {
        int cur = b;
        while (cur >= 0) {
            if (cur == base_body_id) {
                subtree_body_ids.insert(b);
                break;
            }
            if (cur == 0) {
                break;
            }
            cur = model_->body_parentid[cur];
        }
    }
    for (int g = 0; g < model_->ngeom; g++) {
        if (subtree_body_ids.find(model_->geom_bodyid[g]) == subtree_body_ids.end()) {
            continue;
        }
        geom_ids_.push_back(g);
        std::array<float, 4> rgba {};
        rgba[0] = model_->geom_rgba[(g * 4) + 0];
        rgba[1] = model_->geom_rgba[(g * 4) + 1];
        rgba[2] = model_->geom_rgba[(g * 4) + 2];
        rgba[3] = model_->geom_rgba[(g * 4) + 3];
        original_rgba_.push_back(rgba);
        original_contype_.push_back(model_->geom_contype[g]);
        original_conaffinity_.push_back(model_->geom_conaffinity[g]);
    }
    std::unordered_set<int> dof_set;
    for (int b : subtree_body_ids) {
        const int jadr = model_->body_jntadr[b];
        const int jnum = model_->body_jntnum[b];
        for (int j = jadr; j < (jadr + jnum); j++) {
            const int dadr = model_->jnt_dofadr[j];
            int dnum = 0;
            switch (model_->jnt_type[j]) {
            case mjJNT_FREE: dnum = 6; break;
            case mjJNT_BALL: dnum = 3; break;
            case mjJNT_SLIDE:
            case mjJNT_HINGE: dnum = 1; break;
            default: dnum = 0; break;
            }
            for (int k = 0; k < dnum; k++) {
                dof_set.insert(dadr + k);
            }
        }
    }
    subtree_dof_ids_.assign(dof_set.begin(), dof_set.end());
    std::sort(subtree_dof_ids_.begin(), subtree_dof_ids_.end());
    ground_geom_id_ = mj_name2id(model_, mjOBJ_GEOM, "ground");
    if (ground_geom_id_ >= 0) {
        original_ground_contype_ = model_->geom_contype[ground_geom_id_];
        original_ground_conaffinity_ = model_->geom_conaffinity[ground_geom_id_];
    }
    standby_weld_eq_id_ = mj_name2id(model_, mjOBJ_EQUALITY, "forklift_standby_weld");
    initialized_ = !geom_ids_.empty();
    return initialized_;
}

void ForkliftVisibilityController::set_standby_alpha(float alpha)
{
    standby_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
}

void ForkliftVisibilityController::set_standby_tint(float tint)
{
    standby_tint_ = std::clamp(tint, 0.0f, 1.0f);
}

void ForkliftVisibilityController::set_standby_weld_enabled(bool enabled)
{
    standby_weld_enabled_ = enabled;
}

void ForkliftVisibilityController::set_standby_zero_dynamics(bool enabled)
{
    standby_zero_dynamics_ = enabled;
}

void ForkliftVisibilityController::set_owner_active(bool owner_active)
{
    if (!initialized_ || model_ == nullptr || owner_active_ == owner_active) {
        return;
    }
    for (std::size_t i = 0; i < geom_ids_.size(); i++) {
        const int g = geom_ids_[i];
        if (owner_active) {
            model_->geom_rgba[(g * 4) + 0] = original_rgba_[i][0];
            model_->geom_rgba[(g * 4) + 1] = original_rgba_[i][1];
            model_->geom_rgba[(g * 4) + 2] = original_rgba_[i][2];
            model_->geom_rgba[(g * 4) + 3] = original_rgba_[i][3];
        } else {
            model_->geom_rgba[(g * 4) + 0] = original_rgba_[i][0] * standby_tint_;
            model_->geom_rgba[(g * 4) + 1] = original_rgba_[i][1] * standby_tint_;
            model_->geom_rgba[(g * 4) + 2] = original_rgba_[i][2] * standby_tint_;
            model_->geom_rgba[(g * 4) + 3] = standby_alpha_;
        }
    }
    if (owner_active) {
        for (std::size_t i = 0; i < geom_ids_.size(); i++) {
            const int g = geom_ids_[i];
            model_->geom_contype[g] = original_contype_[i];
            model_->geom_conaffinity[g] = original_conaffinity_[i];
        }
        if (ground_geom_id_ >= 0) {
            model_->geom_contype[ground_geom_id_] = original_ground_contype_;
            model_->geom_conaffinity[ground_geom_id_] = original_ground_conaffinity_;
        }
        if (data_ != nullptr && standby_weld_eq_id_ >= 0 && standby_weld_enabled_) {
            data_->eq_active[standby_weld_eq_id_] = 0;
        }
    } else {
        constexpr int kStandbyType = 2;
        for (int g : geom_ids_) {
            model_->geom_contype[g] = 0;
            model_->geom_conaffinity[g] = kStandbyType;
        }
        if (ground_geom_id_ >= 0) {
            model_->geom_contype[ground_geom_id_] = (original_ground_contype_ | kStandbyType);
            model_->geom_conaffinity[ground_geom_id_] = original_ground_conaffinity_;
        }
        if (data_ != nullptr && standby_weld_eq_id_ >= 0 && standby_weld_enabled_) {
            data_->eq_active[standby_weld_eq_id_] = 1;
        }
        if (data_ != nullptr && standby_zero_dynamics_) {
            for (int d : subtree_dof_ids_) {
                data_->qvel[d] = 0.0;
                data_->qacc[d] = 0.0;
                data_->qacc_warmstart[d] = 0.0;
            }
            for (int i = 0; i < model_->nu; i++) {
                data_->ctrl[i] = 0.0;
            }
            for (int i = 0; i < model_->na; i++) {
                data_->act[i] = 0.0;
            }
        }
    }
    if (model_ != nullptr && data_ != nullptr) {
        mj_forward(model_, data_);
    }
    owner_active_ = owner_active;
}
