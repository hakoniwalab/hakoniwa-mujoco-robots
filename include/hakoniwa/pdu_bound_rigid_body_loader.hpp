#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hakoniwa/pdu_bound_rigid_body.hpp"

namespace hakoniwa
{
namespace detail
{
inline std::filesystem::path resolve_relative_path(
    const std::filesystem::path& base_dir,
    const std::string& relative_or_absolute)
{
    const std::filesystem::path path(relative_or_absolute);
    if (path.is_absolute()) {
        return path.lexically_normal();
    }
    return (base_dir / path).lexically_normal();
}

inline nlohmann::json load_json_file(const std::filesystem::path& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("failed to open JSON file: " + path.string());
    }
    nlohmann::json json;
    ifs >> json;
    return json;
}

inline PduBoundRigidBodyType parse_rigid_body_type(const std::string& value)
{
    if (value == "mirrored") {
        return PduBoundRigidBodyType::Mirrored;
    }
    if (value == "controllable") {
        return PduBoundRigidBodyType::Controllable;
    }
    throw std::runtime_error("unknown rigid body binding type: " + value);
}

inline bool is_publish_channel(
    PduBoundRigidBodyType type,
    const std::string& logical_name)
{
    if (type == PduBoundRigidBodyType::Mirrored) {
        return false;
    }
    return logical_name == "pos" || logical_name == "velocity";
}

inline PduChannelKind resolve_channel_kind(
    PduBoundRigidBodyType type,
    const std::string& logical_name,
    bool notify_on_recv)
{
    if (is_publish_channel(type, logical_name)) {
        return PduChannelKind::Publish;
    }
    return notify_on_recv ? PduChannelKind::SubscribeEvent : PduChannelKind::SubscribeLatest;
}
}  // namespace detail

class PduBoundRigidBodyBindingsLoader
{
public:
    static std::vector<PduBoundRigidBodyConfig> load(
        const std::filesystem::path& binding_json_path,
        const std::filesystem::path& endpoint_json_path)
    {
        const auto bindings_json = detail::load_json_file(binding_json_path);
        const auto endpoint_json = detail::load_json_file(endpoint_json_path);
        const auto endpoint_dir = endpoint_json_path.parent_path();
        const auto comm_path = detail::resolve_relative_path(
            endpoint_dir,
            endpoint_json.at("comm").get<std::string>());
        const auto comm_json = detail::load_json_file(comm_path);

        std::unordered_map<std::string, std::unordered_map<std::string, bool>> notify_map;
        for (const auto& robot_entry : comm_json.at("io").at("robots")) {
            const auto robot_name = robot_entry.at("name").get<std::string>();
            auto& robot_map = notify_map[robot_name];
            for (const auto& pdu_entry : robot_entry.at("pdu")) {
                robot_map[pdu_entry.at("name").get<std::string>()] =
                    pdu_entry.at("notify_on_recv").get<bool>();
            }
        }

        std::vector<PduBoundRigidBodyConfig> configs;
        for (const auto& robot_entry : bindings_json.at("robots")) {
            PduBoundRigidBodyConfig config {};
            config.robot_name = robot_entry.at("name").get<std::string>();
            config.body_name = robot_entry.at("bodyName").get<std::string>();
            config.type = detail::parse_rigid_body_type(robot_entry.at("type").get<std::string>());

            const auto notify_it = notify_map.find(config.robot_name);
            if (notify_it == notify_map.end()) {
                throw std::runtime_error(
                    "robot not found in endpoint comm definition: " + config.robot_name);
            }

            for (auto it = robot_entry.at("channels").begin(); it != robot_entry.at("channels").end(); ++it) {
                PduChannelConfig channel {};
                channel.logical_name = it.key();
                channel.pdu_name = it.value().get<std::string>();

                const auto pdu_notify_it = notify_it->second.find(channel.pdu_name);
                if (pdu_notify_it == notify_it->second.end()) {
                    throw std::runtime_error(
                        "channel '" + channel.pdu_name + "' for robot '" + config.robot_name +
                        "' not found in endpoint comm definition");
                }
                channel.notify_on_recv = pdu_notify_it->second;
                channel.kind = detail::resolve_channel_kind(
                    config.type,
                    channel.logical_name,
                    channel.notify_on_recv);
                config.channels.push_back(std::move(channel));
            }
            configs.push_back(std::move(config));
        }
        return configs;
    }
};
}  // namespace hakoniwa
