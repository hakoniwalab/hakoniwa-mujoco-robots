#pragma once

#include "actuator.hpp"
#include "config/json_config_utils.hpp"
#include "sensors/common/update_scheduler.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hako::robots::actuator::impl
{
    class JointTrajectoryActuatorImpl : public IJointTrajectoryActuator
    {
    private:
        struct RuntimeJoint {
            JointTrajectoryJointConfig config;
            int actuator_id {-1};
            std::size_t trajectory_index {0};
        };

        mjModel* model_ {nullptr};
        mjData* data_ {nullptr};
        JointTrajectoryActuatorConfig config_;
        std::vector<RuntimeJoint> runtime_joints_;
        JointTrajectoryTarget trajectory_;
        hako::robots::sensor::common::UpdateScheduler scheduler_;
        double trajectory_start_time_sec_ {0.0};
        bool trajectory_active_ {false};

        static const char* ActuatorTypeName(ActuatorType type)
        {
            switch (type) {
            case ActuatorType::Position:
                return "position";
            case ActuatorType::Velocity:
                return "velocity";
            case ActuatorType::Torque:
                return "torque";
            default:
                return "unknown";
            }
        }

        static bool ParseActuatorType(const std::string& value, ActuatorType& out)
        {
            if (value == "position") {
                out = ActuatorType::Position;
                return true;
            }
            if (value == "velocity") {
                out = ActuatorType::Velocity;
                return true;
            }
            if (value == "torque") {
                out = ActuatorType::Torque;
                return true;
            }
            return false;
        }

        bool IsMjcfActuatorCompatible(int actuator_id, ActuatorType type) const
        {
            constexpr double kEpsilon = 1.0e-12;
            const int bias_type = model_->actuator_biastype[actuator_id];
            const int bias_offset = 10 * actuator_id;

            switch (type) {
            case ActuatorType::Torque:
                return bias_type == mjBIAS_NONE;
            case ActuatorType::Position:
                return bias_type == mjBIAS_AFFINE &&
                       std::abs(model_->actuator_biasprm[bias_offset + 1]) > kEpsilon;
            case ActuatorType::Velocity:
                return bias_type == mjBIAS_AFFINE &&
                       std::abs(model_->actuator_biasprm[bias_offset + 2]) > kEpsilon;
            default:
                return false;
            }
        }

        static void ReadLimit(const nlohmann::json& source, JointTrajectoryJointConfig& config)
        {
            if (!source.contains("limit") || !source.at("limit").is_object()) {
                return;
            }
            const auto& limit = source.at("limit");
            config.limit.has_limits = true;
            if (limit.contains("lower") && limit.at("lower").is_number()) {
                config.limit.lower = limit.at("lower").get<double>();
            }
            if (limit.contains("upper") && limit.at("upper").is_number()) {
                config.limit.upper = limit.at("upper").get<double>();
            }
            if (limit.contains("effort") && limit.at("effort").is_number()) {
                config.limit.effort = limit.at("effort").get<double>();
            }
            if (limit.contains("velocity") && limit.at("velocity").is_number()) {
                config.limit.velocity = limit.at("velocity").get<double>();
            }
        }

        static void ReadDynamics(const nlohmann::json& source, JointTrajectoryJointConfig& config)
        {
            if (!source.contains("dynamics") || !source.at("dynamics").is_object()) {
                return;
            }
            const auto& dynamics = source.at("dynamics");
            if (dynamics.contains("damping") && dynamics.at("damping").is_number()) {
                config.dynamics.damping = dynamics.at("damping").get<double>();
            }
            if (dynamics.contains("friction") && dynamics.at("friction").is_number()) {
                config.dynamics.friction = dynamics.at("friction").get<double>();
            }
        }

        static const std::vector<double>* ValuesForType(
            const JointTrajectoryPointTarget& point,
            ActuatorType type)
        {
            switch (type) {
            case ActuatorType::Position:
                return &point.positions;
            case ActuatorType::Velocity:
                return &point.velocities;
            case ActuatorType::Torque:
                return &point.effort;
            default:
                return nullptr;
            }
        }

        static double ClampTarget(double target, const JointTrajectoryJointConfig& config)
        {
            if (!config.limit.has_limits) {
                return target;
            }
            if (config.type == ActuatorType::Torque && config.limit.effort > 0.0) {
                return std::clamp(target, -config.limit.effort, config.limit.effort);
            }
            if (config.type == ActuatorType::Velocity && config.limit.velocity > 0.0) {
                return std::clamp(target, -config.limit.velocity, config.limit.velocity);
            }
            if (config.type == ActuatorType::Position &&
                (config.limit.lower != 0.0 || config.limit.upper != 0.0))
            {
                return std::clamp(target, config.limit.lower, config.limit.upper);
            }
            return target;
        }

        bool ValidateTrajectory(const JointTrajectoryTarget& trajectory)
        {
            if (trajectory.joint_names.empty()) {
                std::cerr << "[ERROR] JointTrajectory has no joint_names." << std::endl;
                return false;
            }
            if (trajectory.points.empty()) {
                std::cerr << "[ERROR] JointTrajectory has no points." << std::endl;
                return false;
            }

            std::unordered_map<std::string, std::size_t> indices;
            for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
                if (!indices.emplace(trajectory.joint_names[index], index).second) {
                    std::cerr << "[ERROR] JointTrajectory contains duplicate joint name: "
                              << trajectory.joint_names[index] << std::endl;
                    return false;
                }
            }

            double previous_time = -1.0;
            for (const auto& point : trajectory.points) {
                if (point.time_from_start_sec < 0.0 || point.time_from_start_sec < previous_time) {
                    std::cerr << "[ERROR] JointTrajectory point times must be non-negative and non-decreasing."
                              << std::endl;
                    return false;
                }
                previous_time = point.time_from_start_sec;
            }

            for (auto& runtime_joint : runtime_joints_) {
                const auto it = indices.find(runtime_joint.config.name);
                if (it == indices.end()) {
                    std::cerr << "[ERROR] JointTrajectory is missing configured joint: "
                              << runtime_joint.config.name << std::endl;
                    return false;
                }
                runtime_joint.trajectory_index = it->second;
                for (const auto& point : trajectory.points) {
                    const auto* values = ValuesForType(point, runtime_joint.config.type);
                    if (values == nullptr || runtime_joint.trajectory_index >= values->size()) {
                        std::cerr << "[ERROR] JointTrajectory point does not provide "
                                  << ActuatorTypeName(runtime_joint.config.type)
                                  << " data for joint " << runtime_joint.config.name << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }

        double InterpolateJointTarget(
            const RuntimeJoint& joint,
            const JointTrajectoryPointTarget& from,
            const JointTrajectoryPointTarget& to,
            double alpha) const
        {
            const auto* from_values = ValuesForType(from, joint.config.type);
            const auto* to_values = ValuesForType(to, joint.config.type);
            const double from_value = (*from_values)[joint.trajectory_index];
            const double to_value = (*to_values)[joint.trajectory_index];
            return from_value + (to_value - from_value) * alpha;
        }

        void ApplyPoint(const JointTrajectoryPointTarget& point)
        {
            for (const auto& joint : runtime_joints_) {
                const auto* values = ValuesForType(point, joint.config.type);
                const double target = ClampTarget((*values)[joint.trajectory_index], joint.config);
                data_->ctrl[joint.actuator_id] = target;
            }
        }

        void ApplyInterpolated(
            const JointTrajectoryPointTarget& from,
            const JointTrajectoryPointTarget& to,
            double alpha)
        {
            for (const auto& joint : runtime_joints_) {
                const double target = ClampTarget(
                    InterpolateJointTarget(joint, from, to, alpha),
                    joint.config);
                data_->ctrl[joint.actuator_id] = target;
            }
        }

        double GetUpdatePeriodSec() const
        {
            return (config_.pdu_config.update_rate_hz > 0.0)
                ? (1.0 / config_.pdu_config.update_rate_hz)
                : 0.0;
        }

    public:
        JointTrajectoryActuatorImpl(mjModel* model, mjData* data)
            : model_(model)
            , data_(data)
        {}

        bool LoadConfig(const std::string& config_path) override
        {
            std::ifstream ifs(config_path);
            if (!ifs.is_open()) {
                std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
                return false;
            }

            nlohmann::json root;
            try {
                ifs >> root;
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] JSON parse error: " << e.what() << std::endl;
                return false;
            }

            if (!root.contains("spec") || !root.at("spec").is_object() ||
                !root.at("spec").contains("joints") || !root.at("spec").at("joints").is_array())
            {
                std::cerr << "[ERROR] config missing required spec.joints array" << std::endl;
                return false;
            }

            const nlohmann::json* binding = hako::robots::config::FindMjcfBinding(root);
            if (binding == nullptr || !binding->contains("actuators") ||
                !binding->at("actuators").is_array())
            {
                std::cerr << "[ERROR] config missing required mjcf_binding.actuators array" << std::endl;
                return false;
            }

            std::unordered_map<std::string, std::string> actuator_bindings;
            for (const auto& item : binding->at("actuators")) {
                if (!item.is_object() || !item.contains("name") || !item.at("name").is_string() ||
                    !item.contains("actuator_name") || !item.at("actuator_name").is_string())
                {
                    std::cerr << "[ERROR] invalid mjcf_binding.actuators entry" << std::endl;
                    return false;
                }
                actuator_bindings[item.at("name").get<std::string>()] =
                    item.at("actuator_name").get<std::string>();
            }

            config_ = JointTrajectoryActuatorConfig {};
            runtime_joints_.clear();
            std::unordered_set<std::string> configured_names;

            for (const auto& item : root.at("spec").at("joints")) {
                if (!item.is_object() || !item.contains("name") || !item.at("name").is_string() ||
                    !item.contains("type") || !item.at("type").is_string())
                {
                    std::cerr << "[ERROR] invalid spec.joints entry" << std::endl;
                    return false;
                }

                JointTrajectoryJointConfig joint_config {};
                joint_config.name = item.at("name").get<std::string>();
                if (joint_config.name.empty() || !configured_names.insert(joint_config.name).second) {
                    std::cerr << "[ERROR] duplicate or empty joint name in config: "
                              << joint_config.name << std::endl;
                    return false;
                }
                if (!ParseActuatorType(item.at("type").get<std::string>(), joint_config.type)) {
                    std::cerr << "[ERROR] invalid actuator type for joint "
                              << joint_config.name << std::endl;
                    return false;
                }
                ReadLimit(item, joint_config);
                ReadDynamics(item, joint_config);

                const auto binding_it = actuator_bindings.find(joint_config.name);
                if (binding_it == actuator_bindings.end()) {
                    std::cerr << "[ERROR] missing MJCF actuator binding for joint: "
                              << joint_config.name << std::endl;
                    return false;
                }
                joint_config.actuator_name = binding_it->second;

                const int actuator_id = mj_name2id(
                    model_, mjOBJ_ACTUATOR, joint_config.actuator_name.c_str());
                if (actuator_id < 0) {
                    std::cerr << "[ERROR] Actuator not found in MuJoCo model: "
                              << joint_config.actuator_name << std::endl;
                    return false;
                }
                if (!IsMjcfActuatorCompatible(actuator_id, joint_config.type)) {
                    std::cerr << "[ERROR] Actuator type mismatch: config type="
                              << ActuatorTypeName(joint_config.type)
                              << " actuator=" << joint_config.actuator_name << std::endl;
                    return false;
                }

                config_.joints.push_back(joint_config);
                runtime_joints_.push_back(RuntimeJoint {joint_config, actuator_id, 0});
            }

            if (config_.joints.empty()) {
                std::cerr << "[ERROR] config must contain at least one joint" << std::endl;
                return false;
            }

            hako::robots::config::ReadPduConfig(
                root,
                config_.pdu_config.pdu_name,
                config_.pdu_config.update_rate_hz,
                &config_.pdu_config.message_type);

            scheduler_.StartReady(GetUpdatePeriodSec());
            std::cout << "[INFO] Joint trajectory actuator loaded: joints="
                      << config_.joints.size()
                      << " pdu=" << config_.pdu_config.pdu_name << std::endl;
            return true;
        }

        const JointTrajectoryActuatorConfig& GetConfig() const override
        {
            return config_;
        }

        bool SetTrajectory(const JointTrajectoryTarget& trajectory) override
        {
            if (!ValidateTrajectory(trajectory)) {
                return false;
            }
            trajectory_ = trajectory;
            trajectory_start_time_sec_ = data_ != nullptr ? data_->time : 0.0;
            trajectory_active_ = true;
            return true;
        }

        void Update() override
        {
            // No trajectory is available, or the MuJoCo runtime is not ready.
            if (!trajectory_active_ || data_ == nullptr || trajectory_.points.empty()) {
                return;
            }

            // Compute the elapsed simulation time since this trajectory was accepted.
            // JointTrajectoryPoint::time_from_start is interpreted relative to
            // trajectory_start_time_sec_.
            const double elapsed =
                std::max(0.0, data_->time - trajectory_start_time_sec_);

            const auto& points = trajectory_.points;

            // A single trajectory point cannot be interpolated.
            // Also, before reaching the first scheduled point, hold its target values.
            if (points.size() == 1 ||
                elapsed <= points.front().time_from_start_sec)
            {
                ApplyPoint(points.front());
                return;
            }

            // Once the trajectory reaches its final scheduled time,
            // stop interpolating and keep applying the final target values.
            if (elapsed >= points.back().time_from_start_sec) {
                ApplyPoint(points.back());
                return;
            }

            // Find the first trajectory point scheduled after the current elapsed time.
            // The previous point and this point define the interpolation interval.
            const auto upper = std::upper_bound(
                points.begin(),
                points.end(),
                elapsed,
                [](double time, const JointTrajectoryPointTarget& point) {
                    return time < point.time_from_start_sec;
                });

            const auto lower = upper - 1;

            // Compute the normalized position within the current time interval.
            //
            // alpha = 0.0 means the lower point.
            // alpha = 1.0 means the upper point.
            const double span =
                upper->time_from_start_sec - lower->time_from_start_sec;

            const double alpha = span > 0.0
                ? (elapsed - lower->time_from_start_sec) / span
                : 1.0;

            // Linearly interpolate the target value of every configured joint
            // and apply the resulting values to the corresponding MJCF actuators.
            ApplyInterpolated(
                *lower,
                *upper,
                std::clamp(alpha, 0.0, 1.0));
        }

        void ClearTrajectory() override
        {
            trajectory_active_ = false;
            trajectory_ = JointTrajectoryTarget {};
        }

        bool ShouldUpdate(double delta_sec) override
        {
            return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
        }
    };
}