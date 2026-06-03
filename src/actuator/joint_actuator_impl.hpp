#pragma once

#include "actuator.hpp"
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace hako::robots::actuator::impl
{
    class JointActuatorImpl : public IJointActuator
    {
    private:
        mjModel* model_;
        mjData* data_;
        int actuator_id_ {-1};
        JointActuatorConfig config_;

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

    public:
        JointActuatorImpl(mjModel* m, mjData* d)
            : model_(m)
            , data_(d)
        {}

        virtual ~JointActuatorImpl() {}

        bool LoadConfig(const std::string& config_path) override
        {
            std::ifstream ifs(config_path);
            if (!ifs.is_open()) {
                std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
                return false;
            }

            nlohmann::json j;
            try {
                ifs >> j;
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] JSON parse error: " << e.what() << std::endl;
                return false;
            }

            config_ = JointActuatorConfig {};
            if (!j.contains("joint_name") || !j.at("joint_name").is_string()) {
                std::cerr << "[ERROR] config missing required joint_name" << std::endl;
                return false;
            }
            config_.joint_name = j.at("joint_name").get<std::string>();

            if (!j.contains("type") || !j.at("type").is_string()) {
                std::cerr << "[ERROR] config missing required type" << std::endl;
                return false;
            }
            std::string type_str = j.at("type").get<std::string>();
            if (type_str == "position") {
                config_.type = ActuatorType::Position;
            } else if (type_str == "velocity") {
                config_.type = ActuatorType::Velocity;
            } else if (type_str == "torque") {
                config_.type = ActuatorType::Torque;
            } else {
                std::cerr << "[ERROR] Invalid actuator type: " << type_str << std::endl;
                return false;
            }

            if (j.contains("limit") && j.at("limit").is_object()) {
                const auto& limit_obj = j.at("limit");
                config_.limit.has_limits = true;
                if (limit_obj.contains("lower") && limit_obj.at("lower").is_number()) {
                    config_.limit.lower = limit_obj.at("lower").get<double>();
                }
                if (limit_obj.contains("upper") && limit_obj.at("upper").is_number()) {
                    config_.limit.upper = limit_obj.at("upper").get<double>();
                }
                if (limit_obj.contains("effort") && limit_obj.at("effort").is_number()) {
                    config_.limit.effort = limit_obj.at("effort").get<double>();
                }
                if (limit_obj.contains("velocity") && limit_obj.at("velocity").is_number()) {
                    config_.limit.velocity = limit_obj.at("velocity").get<double>();
                }
            }

            if (j.contains("dynamics") && j.at("dynamics").is_object()) {
                const auto& dyn_obj = j.at("dynamics");
                if (dyn_obj.contains("damping") && dyn_obj.at("damping").is_number()) {
                    config_.dynamics.damping = dyn_obj.at("damping").get<double>();
                }
                if (dyn_obj.contains("friction") && dyn_obj.at("friction").is_number()) {
                    config_.dynamics.friction = dyn_obj.at("friction").get<double>();
                }
            }

            if (j.contains("RuntimeBinding") && j.at("RuntimeBinding").is_object()) {
                const auto& binding_obj = j.at("RuntimeBinding");
                if (binding_obj.contains("actuator_name") && binding_obj.at("actuator_name").is_string()) {
                    config_.actuator_name = binding_obj.at("actuator_name").get<std::string>();
                }
            }

            // Resolve MuJoCo Actuator ID
            std::string resolved_name = config_.actuator_name.empty() ? config_.joint_name : config_.actuator_name;
            actuator_id_ = mj_name2id(model_, mjOBJ_ACTUATOR, resolved_name.c_str());
            if (actuator_id_ < 0) {
                // Try resolving by joint name directly as a fallback if the actuator name wasn't found
                actuator_id_ = mj_name2id(model_, mjOBJ_ACTUATOR, config_.joint_name.c_str());
            }

            if (actuator_id_ < 0) {
                std::cerr << "[ERROR] Actuator/Joint not found in MuJoCo model: " << resolved_name << std::endl;
                return false;
            }

            const int joint_id = mj_name2id(model_, mjOBJ_JOINT, config_.joint_name.c_str());
            if (joint_id < 0) {
                std::cerr << "[ERROR] Joint not found in MuJoCo model: "
                          << config_.joint_name << std::endl;
                return false;
            }
            if (model_->actuator_trntype[actuator_id_] != mjTRN_JOINT ||
                model_->actuator_trnid[2 * actuator_id_] != joint_id) {
                std::cerr << "[ERROR] Actuator is not bound to joint:"
                          << " actuator=" << resolved_name
                          << " joint=" << config_.joint_name
                          << std::endl;
                return false;
            }
            if (!IsMjcfActuatorCompatible(actuator_id_, config_.type)) {
                std::cerr << "[ERROR] Actuator type mismatch:"
                          << " config type=" << ActuatorTypeName(config_.type)
                          << " actuator=" << resolved_name
                          << ". Use matching MJCF <motor>, <position>, or <velocity> actuator."
                          << std::endl;
                return false;
            }

            std::cout << "[INFO] Joint actuator loaded:"
                      << " joint=" << config_.joint_name
                      << " actuator=" << resolved_name
                      << " type=" << ActuatorTypeName(config_.type)
                      << " id=" << actuator_id_
                      << std::endl;
            return true;
        }

        const JointActuatorConfig& GetConfig() const override
        {
            return config_;
        }

        void SetTarget(double target) override
        {
            if (actuator_id_ >= 0 && data_ != nullptr) {
                // Apply limit constraints if configured
                if (config_.limit.has_limits) {
                    if (config_.type == ActuatorType::Torque && config_.limit.effort > 0.0) {
                        if (target > config_.limit.effort) target = config_.limit.effort;
                        else if (target < -config_.limit.effort) target = -config_.limit.effort;
                    }
                    else if (config_.type == ActuatorType::Velocity && config_.limit.velocity > 0.0) {
                        if (target > config_.limit.velocity) target = config_.limit.velocity;
                        else if (target < -config_.limit.velocity) target = -config_.limit.velocity;
                    }
                    else if (config_.type == ActuatorType::Position) {
                        if (config_.limit.lower != 0.0 || config_.limit.upper != 0.0) {
                            if (target < config_.limit.lower) target = config_.limit.lower;
                            else if (target > config_.limit.upper) target = config_.limit.upper;
                        }
                    }
                }
                data_->ctrl[actuator_id_] = target;
            }
        }
    };
}
