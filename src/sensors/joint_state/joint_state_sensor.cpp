#include "sensors/joint_state/joint_state_sensor.hpp"

#include <stdexcept>
#include <utility>
#include "sensors/common/json_utils.hpp"

namespace hako::robots::sensor
{
JointStateSensor::JointStateSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
    : world_(std::move(world))
{
}

bool JointStateSensor::LoadConfig(const std::string& config_path)
{
    common::json root;
    if (!common::load_json_file(config_path, root)) {
        return false;
    }

    config_ = JointStateConfig {};
    config_.output.name = common::get_json_string(root, "name", "joint_states");
    config_.output.pdu_name = common::get_json_string(root, "pdu_name", "joint_states");
    config_.output.update_rate_hz = common::get_json_number(root, "update_rate_hz", 50.0);

    if (root.contains("joints") && root.at("joints").is_array()) {
        for (const auto& entry : root.at("joints")) {
            JointBinding binding {};
            binding.name = common::get_json_string(entry, "name", "");
            binding.mjcf_joint = common::get_json_string(entry, "mjcf_joint", binding.name);
            config_.joints.push_back(std::move(binding));
        }
    }

    ResolveJointIds();
    scheduler_.StartReady(GetUpdatePeriodSec());
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
    scheduler_.Reset();
}

double JointStateSensor::GetUpdatePeriodSec() const
{
    return (config_.output.update_rate_hz > 0.0) ? (1.0 / config_.output.update_rate_hz) : 0.02;
}

bool JointStateSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
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
