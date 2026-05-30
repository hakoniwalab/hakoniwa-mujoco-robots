#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hako_msgs/pdu_cpptype_conv_ImpulseCollision.hpp"
#include "hakoniwa/mirrored_rigid_body.hpp"
#include "physics.hpp"

namespace hakoniwa
{
class ImpulseDisturbanceSender
{
public:
    struct Config {
        double restitution_coefficient {0.3};
        double relative_normal_speed_threshold {0.2};
        int cooldown_steps {100};
    };

    ImpulseDisturbanceSender(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        std::vector<MirroredRigidBody*> mirrored_bodies,
        std::string target_body_name)
        : ImpulseDisturbanceSender(
              std::move(world),
              std::move(mirrored_bodies),
              Config{},
              std::move(target_body_name))
    {
    }

    ImpulseDisturbanceSender(
        std::shared_ptr<hako::robots::physics::IWorld> world,
        std::vector<MirroredRigidBody*> mirrored_bodies,
        Config config,
        std::string target_body_name)
        : world_(std::move(world))
        , config_(config)
        , target_body_name_(std::move(target_body_name))
    {
        if (world_ == nullptr || world_->getModel() == nullptr || world_->getData() == nullptr) {
            throw std::runtime_error("ImpulseDisturbanceSender requires valid MuJoCo world/model/data");
        }
        controllable_target_body_id_ = mj_name2id(world_->getModel(), mjOBJ_BODY, target_body_name_.c_str());
        if (controllable_target_body_id_ < 0) {
            throw std::runtime_error("Target body not found for impulse sender: " + target_body_name_);
        }

        for (MirroredRigidBody* mirrored_body : mirrored_bodies) {
            if (mirrored_body == nullptr) {
                continue;
            }
            const int body_id = mj_name2id(world_->getModel(), mjOBJ_BODY, mirrored_body->body_name().c_str());
            if (body_id < 0) {
                throw std::runtime_error("Drone body not found for impulse sender: " + mirrored_body->body_name());
            }
            targets_.push_back(DroneTarget{
                mirrored_body,
                mirrored_body->robot_name(),
                mirrored_body->body_name(),
                body_id,
                false,
            });
        }
        if (targets_.empty()) {
            throw std::runtime_error("ImpulseDisturbanceSender requires at least one mirrored drone target");
        }
    }

    void emit_collision_impulses()
    {
        const mjModel* model = world_->getModel();
        const mjData* data = world_->getData();
        std::vector<std::optional<ContactCandidate>> candidates(targets_.size());
        std::vector<bool> has_contact(targets_.size(), false);

        for (int i = 0; i < data->ncon; ++i) {
            const mjContact& contact = data->contact[i];
            if (contact.geom1 < 0 || contact.geom2 < 0) {
                continue;
            }

            const int body1 = model->geom_bodyid[contact.geom1];
            const int body2 = model->geom_bodyid[contact.geom2];
            const bool geom1_is_controllable_body = body_is_descendant_(body1, controllable_target_body_id_);
            const bool geom2_is_controllable_body = body_is_descendant_(body2, controllable_target_body_id_);
            if (geom1_is_controllable_body == geom2_is_controllable_body) {
                continue;
            }

            const int drone_body = geom1_is_controllable_body ? body2 : body1;
            const int target_index = find_target_index_for_body_(drone_body);
            if (target_index < 0) {
                continue;
            }
            has_contact[static_cast<std::size_t>(target_index)] = true;

            ContactCandidate candidate {};
            candidate.drone_target_index = target_index;
            candidate.dist = contact.dist;
            candidate.contact_point = {contact.pos[0], contact.pos[1], contact.pos[2]};
            candidate.normal = {contact.frame[0], contact.frame[1], contact.frame[2]};
            if (!geom1_is_controllable_body) {
                candidate.normal[0] = -candidate.normal[0];
                candidate.normal[1] = -candidate.normal[1];
                candidate.normal[2] = -candidate.normal[2];
            }
            candidate.relative_normal_speed = std::abs(relative_normal_speed_(targets_[target_index].root_body_id, controllable_target_body_id_, candidate.normal));

            auto& slot = candidates[static_cast<std::size_t>(target_index)];
            if (!slot.has_value() || candidate.dist < slot->dist) {
                slot = candidate;
            }
        }

        for (std::size_t i = 0; i < targets_.size(); ++i) {
            auto& target = targets_[i];
            if (target.cooldown_remaining_steps > 0) {
                target.cooldown_remaining_steps--;
            }
            const bool was_active = target.contact_active;
            target.contact_active = has_contact[i];
            if (!candidates[i].has_value()) {
                continue;
            }
            const auto& candidate = *candidates[i];
            if (was_active) {
                continue;
            }
            if (candidate.relative_normal_speed < config_.relative_normal_speed_threshold) {
                std::cout
                    << "[Impulse] suppressed robot=" << target.robot_name
                    << " body=" << target.root_body_name
                    << " reason=low_relative_speed speed=" << candidate.relative_normal_speed
                    << std::endl;
                continue;
            }
            if (target.cooldown_remaining_steps > 0) {
                std::cout
                    << "[Impulse] suppressed robot=" << target.robot_name
                    << " body=" << target.root_body_name
                    << " reason=cooldown remaining=" << target.cooldown_remaining_steps
                    << std::endl;
                continue;
            }
            send_impulse_(target, candidate);
            target.cooldown_remaining_steps = config_.cooldown_steps;
        }
    }

private:
    struct DroneTarget {
        MirroredRigidBody* mirrored_body {nullptr};
        std::string robot_name {};
        std::string root_body_name {};
        int root_body_id {-1};
        bool contact_active {false};
        int cooldown_remaining_steps {0};
    };

    struct ContactCandidate {
        int drone_target_index {-1};
        double dist {0.0};
        std::array<double, 3> contact_point {};
        std::array<double, 3> normal {};
        double relative_normal_speed {0.0};
    };

    static std::array<double, 3> body_com_world_(const mjData* data, int body_id)
    {
        return {
            data->xipos[3 * body_id + 0],
            data->xipos[3 * body_id + 1],
            data->xipos[3 * body_id + 2],
        };
    }

    static std::array<double, 3> body_velocity_world_(const mjModel* model, const mjData* data, int body_id)
    {
        mjtNum object_velocity[6] = {};
        mj_objectVelocity(model, data, mjOBJ_BODY, body_id, object_velocity, 0);
        return {
            object_velocity[3],
            object_velocity[4],
            object_velocity[5],
        };
    }

    static std::array<double, 3> body_angular_velocity_world_(const mjModel* model, const mjData* data, int body_id)
    {
        mjtNum object_velocity[6] = {};
        mj_objectVelocity(model, data, mjOBJ_BODY, body_id, object_velocity, 0);
        return {
            object_velocity[0],
            object_velocity[1],
            object_velocity[2],
        };
    }

    static std::array<double, 3> body_euler_world_(const mjData* data, int body_id)
    {
        const mjtNum* R = &data->xmat[9 * body_id];
        std::array<double, 3> euler {};
        euler[1] = std::asin(R[6]);
        const double cos_pitch = std::cos(euler[1]);
        if (std::fabs(cos_pitch) > 1e-6) {
            euler[0] = std::atan2(-R[7], R[8]);
            euler[2] = std::atan2(R[3], R[0]);
        } else {
            euler[0] = 0.0;
            euler[2] = std::atan2(-R[1], R[4]);
        }
        return euler;
    }

    static double dot_(const std::array<double, 3>& lhs, const std::array<double, 3>& rhs)
    {
        return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
    }

    static std::array<double, 3> sub_(
        const std::array<double, 3>& lhs,
        const std::array<double, 3>& rhs)
    {
        return {lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]};
    }

    bool body_is_descendant_(int body_id, int ancestor_body_id) const
    {
        const mjModel* model = world_->getModel();
        int cursor = body_id;
        while (cursor >= 0) {
            if (cursor == ancestor_body_id) {
                return true;
            }
            const int parent = model->body_parentid[cursor];
            if (parent == cursor) {
                break;
            }
            cursor = parent;
        }
        return false;
    }

    int find_target_index_for_body_(int body_id) const
    {
        for (std::size_t i = 0; i < targets_.size(); ++i) {
            if (body_is_descendant_(body_id, targets_[i].root_body_id)) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    double relative_normal_speed_(
        int self_body_id,
        int target_body_id,
        const std::array<double, 3>& normal) const
    {
        const mjModel* model = world_->getModel();
        const mjData* data = world_->getData();
        const auto self_vel = body_velocity_world_(model, data, self_body_id);
        const auto target_vel = body_velocity_world_(model, data, target_body_id);
        return dot_(sub_(self_vel, target_vel), normal);
    }

    void send_impulse_(const DroneTarget& target, const ContactCandidate& candidate)
    {
        const mjModel* model = world_->getModel();
        const mjData* data = world_->getData();

        HakoCpp_ImpulseCollision impulse {};
        impulse.collision = true;
        impulse.is_target_static = false;
        impulse.restitution_coefficient = config_.restitution_coefficient;

        const auto self_com = body_com_world_(data, target.root_body_id);
        const auto controllable_body_com = body_com_world_(data, controllable_target_body_id_);
        const auto self_contact = sub_(candidate.contact_point, self_com);
        const auto target_contact = sub_(candidate.contact_point, controllable_body_com);
        const auto target_velocity = body_velocity_world_(model, data, controllable_target_body_id_);
        const auto target_angular_velocity = body_angular_velocity_world_(model, data, controllable_target_body_id_);
        const auto target_euler = body_euler_world_(data, controllable_target_body_id_);

        impulse.self_contact_vector.x = self_contact[0];
        impulse.self_contact_vector.y = self_contact[1];
        impulse.self_contact_vector.z = self_contact[2];

        impulse.normal.x = candidate.normal[0];
        impulse.normal.y = candidate.normal[1];
        impulse.normal.z = candidate.normal[2];

        impulse.target_contact_vector.x = target_contact[0];
        impulse.target_contact_vector.y = target_contact[1];
        impulse.target_contact_vector.z = target_contact[2];

        impulse.target_velocity.x = target_velocity[0];
        impulse.target_velocity.y = target_velocity[1];
        impulse.target_velocity.z = target_velocity[2];

        impulse.target_angular_velocity.x = target_angular_velocity[0];
        impulse.target_angular_velocity.y = target_angular_velocity[1];
        impulse.target_angular_velocity.z = target_angular_velocity[2];

        impulse.target_euler.x = target_euler[0];
        impulse.target_euler.y = target_euler[1];
        impulse.target_euler.z = target_euler[2];

        impulse.target_inertia.x = model->body_inertia[3 * controllable_target_body_id_ + 0];
        impulse.target_inertia.y = model->body_inertia[3 * controllable_target_body_id_ + 1];
        impulse.target_inertia.z = model->body_inertia[3 * controllable_target_body_id_ + 2];
        impulse.target_mass = model->body_mass[controllable_target_body_id_];

        const bool result = target.mirrored_body != nullptr && target.mirrored_body->publish_impulse(impulse);
        std::cout
            << "[Impulse] " << (result ? "emitted" : "send_failed")
            << " robot=" << target.robot_name
            << " self_body=" << target.root_body_name
            << " target_body=" << target_body_name_
            << " contact=(" << candidate.contact_point[0] << ", " << candidate.contact_point[1] << ", " << candidate.contact_point[2] << ")"
            << " normal=(" << candidate.normal[0] << ", " << candidate.normal[1] << ", " << candidate.normal[2] << ")"
            << " self_contact=(" << self_contact[0] << ", " << self_contact[1] << ", " << self_contact[2] << ")"
            << " target_contact=(" << target_contact[0] << ", " << target_contact[1] << ", " << target_contact[2] << ")"
            << " target_velocity=(" << target_velocity[0] << ", " << target_velocity[1] << ", " << target_velocity[2] << ")"
            << " target_angular_velocity=(" << target_angular_velocity[0] << ", " << target_angular_velocity[1] << ", " << target_angular_velocity[2] << ")"
            << " target_mass=" << impulse.target_mass
            << " target_inertia=(" << impulse.target_inertia.x << ", " << impulse.target_inertia.y << ", " << impulse.target_inertia.z << ")"
            << " restitution=" << impulse.restitution_coefficient
            << " relative_normal_speed=" << candidate.relative_normal_speed
            << " cooldown_steps=" << config_.cooldown_steps
            << std::endl;
    }

    std::shared_ptr<hako::robots::physics::IWorld> world_;
    Config config_ {};
    std::string target_body_name_ {};
    int controllable_target_body_id_ {-1};
    std::vector<DroneTarget> targets_ {};
};
}  // namespace hakoniwa
