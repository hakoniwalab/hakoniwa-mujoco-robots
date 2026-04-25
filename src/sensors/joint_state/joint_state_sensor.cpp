#include "sensors/joint_state/joint_state_sensor.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor
{
namespace
{
using json = nlohmann::json;

double get_json_number(const json& j, const char* key, double default_value)
{
    if (!j.contains(key) || !j.at(key).is_number()) {
        return default_value;
    }
    return j.at(key).get<double>();
}

std::string get_json_string(const json& j, const char* key, const std::string& default_value)
{
    if (!j.contains(key) || !j.at(key).is_string()) {
        return default_value;
    }
    return j.at(key).get<std::string>();
}
}

JointStateSensor::JointStateSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
    : world_(std::move(world))
{
}

bool JointStateSensor::LoadConfig(const std::string& config_path)
{
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        return false;
    }

    json root;
    ifs >> root;

    config_ = JointStateConfig {};
    config_.output.name = get_json_string(root, "name", "joint_states");
    config_.output.pdu_name = get_json_string(root, "pdu_name", "joint_states");
    config_.output.update_rate_hz = get_json_number(root, "update_rate_hz", 50.0);

    if (root.contains("joints") && root.at("joints").is_array()) {
        for (const auto& entry : root.at("joints")) {
            JointBinding binding {};
            binding.name = get_json_string(entry, "name", "");
            binding.mjcf_joint = get_json_string(entry, "mjcf_joint", binding.name);
            config_.joints.push_back(std::move(binding));
        }
    }

    ResolveJointIds();
    elapsed_sec_ = GetUpdatePeriodSec();
    return !joint_ids_.empty();
}

const JointStateConfig& JointStateSensor::GetConfig() const
{
    return config_;
}

void JointStateSensor::Build(JointStateFrame& out)
{
    auto* model = world_->getModel();
    auto* data = world_->getData();

    out.names.clear();
    out.position.clear();
    out.velocity.clear();
    out.effort.clear();

    for (size_t i = 0; i < config_.joints.size(); ++i) {
        const int joint_id = joint_ids_.at(i);
        out.names.push_back(config_.joints[i].name);
        out.position.push_back(data->qpos[model->jnt_qposadr[joint_id]]);
        out.velocity.push_back(data->qvel[model->jnt_dofadr[joint_id]]);
        out.effort.push_back(0.0);
    }
}

void JointStateSensor::Reset()
{
    elapsed_sec_ = 0.0;
}

double JointStateSensor::GetUpdatePeriodSec() const
{
    return (config_.output.update_rate_hz > 0.0) ? (1.0 / config_.output.update_rate_hz) : 0.02;
}

bool JointStateSensor::ShouldUpdate(double delta_sec)
{
    elapsed_sec_ += delta_sec;
    const double period = GetUpdatePeriodSec();
    if (elapsed_sec_ + 1.0e-9 < period) {
        return false;
    }
    elapsed_sec_ -= period;
    if (elapsed_sec_ < 0.0) {
        elapsed_sec_ = 0.0;
    }
    return true;
}

void JointStateSensor::ResolveJointIds()
{
    joint_ids_.clear();
    auto* model = world_->getModel();
    for (const auto& joint : config_.joints) {
        const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint.mjcf_joint.c_str());
        if (joint_id < 0) {
            throw std::runtime_error("Joint not found: " + joint.mjcf_joint);
        }
        joint_ids_.push_back(joint_id);
    }
}
}
