#pragma once

#include <array>
#include <vector>

#include <mujoco/mujoco.h>

class ForkliftVisibilityController {
public:
    bool initialize(mjModel* model, mjData* data, const char* base_body_name);

    void set_standby_alpha(float alpha);
    void set_standby_tint(float tint);
    void set_standby_weld_enabled(bool enabled);
    void set_standby_zero_dynamics(bool enabled);
    void set_owner_active(bool owner_active);

private:
    mjModel* model_ {nullptr};
    mjData* data_ {nullptr};
    bool initialized_ {false};
    bool owner_active_ {true};
    std::vector<int> geom_ids_ {};
    std::vector<std::array<float, 4>> original_rgba_ {};
    std::vector<int> subtree_dof_ids_ {};
    std::vector<int> original_contype_ {};
    std::vector<int> original_conaffinity_ {};
    int ground_geom_id_ {-1};
    int original_ground_contype_ {1};
    int original_ground_conaffinity_ {1};
    int standby_weld_eq_id_ {-1};
    float standby_alpha_ {0.0f};
    float standby_tint_ {0.35f};
    bool standby_weld_enabled_ {false};
    bool standby_zero_dynamics_ {true};
};
