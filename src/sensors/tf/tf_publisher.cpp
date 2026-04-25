#include "sensors/tf/tf_publisher.hpp"

#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor
{
namespace
{
using json = nlohmann::json;

struct WorldPose
{
    hako::robots::types::Position position {};
    Quaternion orientation {};
};

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

Quaternion quat_from_mj(const mjtNum* q)
{
    Quaternion out {};
    out.w = q[0];
    out.x = q[1];
    out.y = q[2];
    out.z = q[3];
    return out;
}

Quaternion quat_conjugate(const Quaternion& q)
{
    Quaternion out {};
    out.w = q.w;
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;
    return out;
}

Quaternion quat_multiply(const Quaternion& a, const Quaternion& b)
{
    Quaternion out {};
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
}

hako::robots::types::Vector3 rotate_vector(const Quaternion& q, const hako::robots::types::Vector3& v)
{
    Quaternion vq {};
    vq.w = 0.0;
    vq.x = v.x;
    vq.y = v.y;
    vq.z = v.z;
    const Quaternion qr = quat_multiply(quat_multiply(q, vq), quat_conjugate(q));
    return {qr.x, qr.y, qr.z};
}

WorldPose get_world_pose(
    hako::robots::physics::IWorld* world,
    const std::shared_ptr<hako::robots::physics::IRigidBody>& body,
    const std::string& body_name)
{
    WorldPose pose {};
    pose.position = body->GetPosition();
    const int body_id = mj_name2id(world->getModel(), mjOBJ_BODY, body_name.c_str());
    if (body_id >= 0) {
        pose.orientation = quat_from_mj(&world->getData()->xquat[4 * body_id]);
    }
    return pose;
}
}

TfPublisher::TfPublisher(std::shared_ptr<hako::robots::physics::IWorld> world)
    : world_(std::move(world))
{
}

bool TfPublisher::LoadConfig(const std::string& config_path)
{
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        return false;
    }

    json root;
    ifs >> root;

    config_ = TfConfig {};
    config_.output.name = get_json_string(root, "name", "tf");
    config_.output.pdu_name = get_json_string(root, "pdu_name", "tf");
    config_.output.update_rate_hz = get_json_number(root, "update_rate_hz", 50.0);

    body_cache_.clear();
    child_to_body_.clear();
    if (root.contains("transforms") && root.at("transforms").is_array()) {
        for (const auto& entry : root.at("transforms")) {
            binding::TransformBinding binding {};
            binding.parent_frame_id = get_json_string(entry, "parent_frame_id", "");
            binding.child_frame_id = get_json_string(entry, "child_frame_id", "");
            binding.source_body = get_json_string(entry, "source_body", "");
            config_.transforms.push_back(binding);
            child_to_body_[binding.child_frame_id] = binding.source_body;
            if (!binding.source_body.empty()) {
                body_cache_[binding.source_body] = world_->getRigidBody(binding.source_body);
            }
        }
    }

    elapsed_sec_ = GetUpdatePeriodSec();
    return true;
}

const TfConfig& TfPublisher::GetConfig() const
{
    return config_;
}

void TfPublisher::Build(TfFrame& out)
{
    out.transforms.clear();

    for (const auto& binding : config_.transforms) {
        TransformFrame frame {};
        frame.header.frame_id = binding.parent_frame_id;
        frame.child_frame_id = binding.child_frame_id;

        auto child_it = body_cache_.find(binding.source_body);
        if (child_it == body_cache_.end()) {
            continue;
        }
        const auto child_pose = get_world_pose(world_.get(), child_it->second, binding.source_body);

        if (binding.parent_frame_id == "odom" || binding.parent_frame_id.empty()) {
            frame.transform.position = child_pose.position;
            frame.transform.orientation = child_pose.orientation;
            out.transforms.push_back(std::move(frame));
            continue;
        }

        auto parent_body_it = child_to_body_.find(binding.parent_frame_id);
        if (parent_body_it == child_to_body_.end()) {
            frame.transform.position = child_pose.position;
            frame.transform.orientation = child_pose.orientation;
            out.transforms.push_back(std::move(frame));
            continue;
        }

        const auto parent_cache_it = body_cache_.find(parent_body_it->second);
        if (parent_cache_it == body_cache_.end()) {
            continue;
        }
        const auto parent_pose = get_world_pose(world_.get(), parent_cache_it->second, parent_body_it->second);

        const Quaternion parent_inv = quat_conjugate(parent_pose.orientation);
        hako::robots::types::Vector3 delta_world {
            child_pose.position.x - parent_pose.position.x,
            child_pose.position.y - parent_pose.position.y,
            child_pose.position.z - parent_pose.position.z
        };
        const auto delta_local = rotate_vector(parent_inv, delta_world);
        frame.transform.position = {delta_local.x, delta_local.y, delta_local.z};
        frame.transform.orientation = quat_multiply(parent_inv, child_pose.orientation);
        out.transforms.push_back(std::move(frame));
    }
}

void TfPublisher::Reset()
{
    elapsed_sec_ = 0.0;
}

double TfPublisher::GetUpdatePeriodSec() const
{
    return (config_.output.update_rate_hz > 0.0) ? (1.0 / config_.output.update_rate_hz) : 0.02;
}

bool TfPublisher::ShouldUpdate(double delta_sec)
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
}
