#pragma once

// MuJoCo ray caster for range sensors: mj_ray from a pose mounted on a body,
// skipping the geoms of the excluded subtree (the robot that carries the
// sensor). Requires the hakoniwa-sensor-adapter headers (header-only, C++17).

#include "physics.hpp"

#include "hakoniwa/sensor_adapter/range_image_sensor_adapter.hpp"

#include <mujoco/mujoco.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace hako::robots::physics::impl
{

/**
 * Rays start at the mounted pose of the last mj_forward / mj_step, in the
 * sensor axes (x forward, y left, z up). Geoms of the mount body (the sensor
 * housing) and of the exclusion root, and of the bodies under them, are
 * never hit. object_id is the MuJoCo geom id and
 * the normal is in the sensor axes. The caster reads the model and the data
 * and never changes them.
 */
class MujocoRayCaster final : public hakoniwa::sensor_adapter::IRayCaster
{
public:
    MujocoRayCaster(const mjModel* model, const mjData* data, const RayCasterSpec& spec)
        : model_(model)
        , data_(data)
    {
        if (model_ == nullptr || data_ == nullptr) {
            throw std::invalid_argument("ray caster requires a loaded MuJoCo model");
        }
        body_id_ = mj_name2id(model_, mjOBJ_BODY, spec.mount_body.c_str());
        if (body_id_ <= 0) {
            throw std::invalid_argument("ray caster mount body is not a MuJoCo body: " + spec.mount_body);
        }
        // The mount body is always excluded; an exclusion root is added to it.
        excluded_roots_.push_back(body_id_);
        if (!spec.exclude_root_body.empty()) {
            const int root = mj_name2id(model_, mjOBJ_BODY, spec.exclude_root_body.c_str());
            if (root <= 0) {
                throw std::invalid_argument(
                    "ray caster exclusion root is not a MuJoCo body: " + spec.exclude_root_body);
            }
            if (root != body_id_) {
                excluded_roots_.push_back(root);
            }
        }
        mjtNum q[4] = {spec.orientation[0], spec.orientation[1], spec.orientation[2], spec.orientation[3]};
        if (mju_normalize4(q) < mjMINVAL) {
            throw std::invalid_argument("ray caster orientation is not a quaternion");
        }
        mju_quat2Mat(mount_rotation_, q);
        for (int i = 0; i < 3; ++i) {
            mount_position_[i] = spec.position[i];
        }
    }

    bool cast(const hakoniwa::sensor_adapter::Ray* rays, std::size_t count, double max_distance,
              hakoniwa::sensor_adapter::RayHit* hits, std::string& error) override
    {
        if (count > 0 && (rays == nullptr || hits == nullptr)) {
            error = "rays and hits are required";
            return false;
        }
        // Rays from the mounted pose of the last mj_forward / mj_step (read only)
        mjtNum p[3], R[9];  // sensor position and axes in the world, row-major
        pose(p, R);
        for (std::size_t i = 0; i < count; ++i) {
            const auto& o = rays[i].origin;
            const auto& d = rays[i].direction;
            mjtNum start[3], dir[3];
            for (int r = 0; r < 3; ++r) {
                start[r] = p[r] + R[3 * r] * o.x + R[3 * r + 1] * o.y + R[3 * r + 2] * o.z;
                dir[r] = R[3 * r] * d.x + R[3 * r + 1] * d.y + R[3 * r + 2] * d.z;
            }
            hits[i] = {};
            // Walk through the robot's own geoms: restart just past each one it hits.
            mjtNum travelled = 0.0;
            for (int attempt = 0; attempt < 32; ++attempt) {
                int geom = -1;
                mjtNum normal[3] = {0, 0, 0};
                const mjtNum dist = mj_ray(model_, data_, start, dir, nullptr, 1, -1, &geom, normal);
                // Hits beyond the cast distance are no hit
                if (dist < 0 || travelled + dist > max_distance) break;
                if (!is_excluded(geom)) {
                    hits[i].hit = true;
                    hits[i].distance = travelled + dist;
                    hits[i].object_id = geom;
                    // World normal into the sensor axes (R transposed).
                    hits[i].normal = {R[0] * normal[0] + R[3] * normal[1] + R[6] * normal[2],
                                      R[1] * normal[0] + R[4] * normal[1] + R[7] * normal[2],
                                      R[2] * normal[0] + R[5] * normal[1] + R[8] * normal[2]};
                    break;
                }
                // Restart 1e-6 m past an excluded geom (up to 32 times)
                constexpr mjtNum kStep = 1e-6;
                for (int r = 0; r < 3; ++r) start[r] += dir[r] * (dist + kStep);
                travelled += dist + kStep;
            }
        }
        return true;
    }

private:
    // World position and rotation (row-major, columns = sensor axes in the world).
    void pose(mjtNum position[3], mjtNum rotation[9]) const
    {
        const mjtNum* p = data_->xpos + 3 * body_id_;
        const mjtNum* R = data_->xmat + 9 * body_id_;
        mjtNum offset[3];
        mju_mulMatVec(offset, R, mount_position_, 3, 3);
        mju_add3(position, p, offset);
        mju_mulMatMat(rotation, R, mount_rotation_, 3, 3, 3);
    }

    // A geom is excluded when its body is one of the excluded roots or under one.
    bool is_excluded(int geom_id) const
    {
        if (geom_id < 0) return false;
        for (int b = model_->geom_bodyid[geom_id]; b > 0; b = model_->body_parentid[b])
            for (const int root : excluded_roots_)
                if (b == root) return true;
        return false;
    }

    const mjModel* model_ {nullptr};
    const mjData* data_ {nullptr};
    int body_id_ {-1};
    std::vector<int> excluded_roots_;
    mjtNum mount_position_[3] {0, 0, 0};
    mjtNum mount_rotation_[9] {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

}  // namespace hako::robots::physics::impl
