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

namespace hako::robots::actuator::impl
{
    class NamedActuatorImpl : public INamedActuator
    {
    private:
        mjModel* model_ {nullptr};
        mjData* data_ {nullptr};
        NamedActuatorConfig config_;
        NamedActuatorBinding binding_;
        hako::robots::sensor::common::UpdateScheduler scheduler_;

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

        static ActuatorTransmissionType ConvertTransmissionType(int trntype)
        {
            switch (trntype) {
            case mjTRN_JOINT:
                return ActuatorTransmissionType::Joint;
            case mjTRN_TENDON:
                return ActuatorTransmissionType::Tendon;
            case mjTRN_SITE:
                return ActuatorTransmissionType::Site;
            case mjTRN_BODY:
                return ActuatorTransmissionType::Body;
            default:
                return ActuatorTransmissionType::Unknown;
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

        std::string ResolveTargetName(ActuatorTransmissionType type, int target_id) const
        {
            int object_type = -1;
            switch (type) {
            case ActuatorTransmissionType::Joint:
                object_type = mjOBJ_JOINT;
                break;
            case ActuatorTransmissionType::Tendon:
                object_type = mjOBJ_TENDON;
                break;
            case ActuatorTransmissionType::Site:
                object_type = mjOBJ_SITE;
                break;
            case ActuatorTransmissionType::Body:
                object_type = mjOBJ_BODY;
                break;
            default:
                return "";
            }

            const char* name = mj_id2name(model_, object_type, target_id);
            return name != nullptr ? name : "";
        }

        bool BindByName()
        {
            binding_ = NamedActuatorBinding {};
            binding_.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, config_.actuator_name.c_str());
            if (binding_.actuator_id < 0) {
                std::cerr << "[ERROR] Actuator not found in MuJoCo model: "
                          << config_.actuator_name << std::endl;
                return false;
            }
            if (!IsMjcfActuatorCompatible(binding_.actuator_id, config_.type)) {
                std::cerr << "[ERROR] Actuator type mismatch:"
                          << " config type=" << ActuatorTypeName(config_.type)
                          << " actuator=" << config_.actuator_name
                          << ". Use matching MJCF <motor>, <position>, or <velocity> actuator."
                          << std::endl;
                return false;
            }

            binding_.transmission_type =
                ConvertTransmissionType(model_->actuator_trntype[binding_.actuator_id]);
            binding_.target_id = model_->actuator_trnid[2 * binding_.actuator_id + 0];
            binding_.target_name = ResolveTargetName(binding_.transmission_type, binding_.target_id);
            binding_.ctrl_limited = model_->actuator_ctrllimited[binding_.actuator_id] != 0;
            binding_.ctrl_lower = model_->actuator_ctrlrange[2 * binding_.actuator_id + 0];
            binding_.ctrl_upper = model_->actuator_ctrlrange[2 * binding_.actuator_id + 1];

            if (binding_.target_id < 0 || binding_.target_name.empty()) {
                std::cerr << "[ERROR] Unsupported or unnamed actuator transmission:"
                          << " actuator=" << config_.actuator_name << std::endl;
                return false;
            }

            std::cout << "[INFO] Named actuator loaded:"
                      << " actuator=" << config_.actuator_name
                      << " type=" << ActuatorTypeName(config_.type)
                      << " id=" << binding_.actuator_id
                      << " target=" << binding_.target_name
                      << std::endl;
            scheduler_.StartReady(GetUpdatePeriodSec());
            return true;
        }

    public:
        NamedActuatorImpl(mjModel* m, mjData* d)
            : model_(m)
            , data_(d)
        {}

        virtual ~NamedActuatorImpl() {}

        bool BindConfig(const NamedActuatorConfig& config) override
        {
            config_ = config;
            return BindByName();
        }

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

            config_ = NamedActuatorConfig {};
            const nlohmann::json* spec = &j;
            if (j.contains("spec") && j.at("spec").is_object()) {
                spec = &j.at("spec");
            }

            if (!spec->contains("actuator_name") || !spec->at("actuator_name").is_string()) {
                std::cerr << "[ERROR] config missing required actuator_name" << std::endl;
                return false;
            }
            config_.actuator_name = spec->at("actuator_name").get<std::string>();

            if (!spec->contains("type") || !spec->at("type").is_string()) {
                std::cerr << "[ERROR] config missing required type" << std::endl;
                return false;
            }
            const std::string type_str = spec->at("type").get<std::string>();
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

            if (spec->contains("limit") && spec->at("limit").is_object()) {
                const auto& limit_obj = spec->at("limit");
                config_.limit.has_limits = true;
                if (limit_obj.contains("lower") && limit_obj.at("lower").is_number()) {
                    config_.limit.lower = limit_obj.at("lower").get<double>();
                }
                if (limit_obj.contains("upper") && limit_obj.at("upper").is_number()) {
                    config_.limit.upper = limit_obj.at("upper").get<double>();
                }
            }

            hako::robots::config::ReadPduConfig(
                j,
                config_.pdu_config.pdu_name,
                config_.pdu_config.update_rate_hz,
                &config_.pdu_config.message_type);

            return BindByName();
        }

        const NamedActuatorConfig& GetConfig() const override
        {
            return config_;
        }

        const NamedActuatorBinding& GetBinding() const override
        {
            return binding_;
        }

        void SetTarget(double target) override
        {
            if (binding_.actuator_id < 0 || data_ == nullptr) {
                return;
            }
            if (config_.limit.has_limits) {
                target = std::clamp(target, config_.limit.lower, config_.limit.upper);
            } else if (binding_.ctrl_limited) {
                target = std::clamp(target, binding_.ctrl_lower, binding_.ctrl_upper);
            }
            data_->ctrl[binding_.actuator_id] = target;
        }

        bool ShouldUpdate(double delta_sec) override
        {
            return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
        }

    private:
        double GetUpdatePeriodSec() const
        {
            return (config_.pdu_config.update_rate_hz > 0.0)
                ? (1.0 / config_.pdu_config.update_rate_hz)
                : 0.0;
        }
    };
}
