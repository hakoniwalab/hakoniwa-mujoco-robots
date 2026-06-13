#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace hako::robots::config
{
    using json = nlohmann::json;

    inline const json* FindObject(const json& root, const char* key)
    {
        if (!root.contains(key) || !root.at(key).is_object()) {
            return nullptr;
        }
        return &root.at(key);
    }

    inline const json* FindObjectAlias(
        const json& root,
        const char* primary_key,
        const char* legacy_key)
    {
        if (const json* primary = FindObject(root, primary_key)) {
            return primary;
        }
        return FindObject(root, legacy_key);
    }

    inline const json* FindMjcfBinding(const json& root)
    {
        return FindObjectAlias(root, "mjcf_binding", "RuntimeBinding");
    }

    inline void ReadPduConfig(
        const json& root,
        std::string& pdu_name,
        double& update_rate_hz,
        std::string* message_type = nullptr)
    {
        const json* pdu_config = FindObject(root, "pdu_config");
        if (pdu_config == nullptr) {
            return;
        }
        if (pdu_config->contains("pdu_name") && pdu_config->at("pdu_name").is_string()) {
            pdu_name = pdu_config->at("pdu_name").get<std::string>();
        }
        if (pdu_config->contains("update_rate_hz") &&
            pdu_config->at("update_rate_hz").is_number())
        {
            update_rate_hz = pdu_config->at("update_rate_hz").get<double>();
        }
        if (message_type != nullptr &&
            pdu_config->contains("message_type") &&
            pdu_config->at("message_type").is_string())
        {
            *message_type = pdu_config->at("message_type").get<std::string>();
        }
    }
}
