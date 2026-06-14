#include "config/asset_manifest.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

#include "config/json_config_utils.hpp"

namespace hako::robots::config
{
namespace
{
    std::string resolve_path(const std::filesystem::path& base_dir, const std::string& value)
    {
        const std::filesystem::path path(value);
        if (path.is_absolute()) {
            return path.lexically_normal().string();
        }
        return (base_dir / path).lexically_normal().string();
    }

    bool load_json_file(const std::string& path, json& out, std::string* error_message)
    {
        std::ifstream ifs(path);
        if (!ifs) {
            if (error_message != nullptr) {
                *error_message = "failed to open JSON file: " + path;
            }
            return false;
        }
        try {
            ifs >> out;
        } catch (const std::exception& e) {
            if (error_message != nullptr) {
                *error_message = "failed to parse JSON file: " + path + ": " + e.what();
            }
            return false;
        }
        return true;
    }

    bool read_required_string(
        const json& root,
        const char* key,
        std::string& out,
        std::string* error_message)
    {
        if (!root.contains(key) || !root.at(key).is_string() || root.at(key).get<std::string>().empty()) {
            if (error_message != nullptr) {
                *error_message = std::string("manifest field is missing or invalid: ") + key;
            }
            return false;
        }
        out = root.at(key).get<std::string>();
        return true;
    }
}

const AssetManifestComponent* AssetManifest::FindComponent(const std::string& id) const
{
    for (const auto& component : components) {
        if (component.id == id) {
            return &component;
        }
    }
    return nullptr;
}

std::string AssetManifest::ComponentConfig(const std::string& id) const
{
    const AssetManifestComponent* component = FindComponent(id);
    return component != nullptr ? component->config : std::string {};
}

std::string AssetManifest::ComponentPduRobot(const std::string& id) const
{
    const AssetManifestComponent* component = FindComponent(id);
    return component != nullptr ? component->pdu_robot : std::string {};
}

bool LoadAssetManifestFromJson(
    const std::string& manifest_path,
    AssetManifest& out,
    std::string* error_message)
{
    json root;
    if (!load_json_file(manifest_path, root, error_message)) {
        return false;
    }
    if (!root.is_object()) {
        if (error_message != nullptr) {
            *error_message = "manifest root must be an object: " + manifest_path;
        }
        return false;
    }

    AssetManifest manifest {};
    manifest.path = std::filesystem::path(manifest_path).lexically_normal().string();
    const auto base_dir = std::filesystem::path(manifest.path).parent_path();

    if (!read_required_string(root, "name", manifest.name, error_message) ||
        !read_required_string(root, "model", manifest.model, error_message) ||
        !read_required_string(root, "pdu_def", manifest.pdu_def, error_message) ||
        !read_required_string(root, "endpoint", manifest.endpoint, error_message))
    {
        return false;
    }
    manifest.model = resolve_path(base_dir, manifest.model);
    manifest.pdu_def = resolve_path(base_dir, manifest.pdu_def);
    manifest.endpoint = resolve_path(base_dir, manifest.endpoint);

    if (!root.contains("components") || !root.at("components").is_array()) {
        if (error_message != nullptr) {
            *error_message = "manifest field is missing or invalid: components";
        }
        return false;
    }

    for (const auto& item : root.at("components")) {
        if (!item.is_object()) {
            if (error_message != nullptr) {
                *error_message = "manifest component must be an object";
            }
            return false;
        }
        AssetManifestComponent component {};
        if (!read_required_string(item, "id", component.id, error_message) ||
            !read_required_string(item, "kind", component.kind, error_message) ||
            !read_required_string(item, "type", component.type, error_message) ||
            !read_required_string(item, "config", component.config, error_message))
        {
            return false;
        }
        component.config = resolve_path(base_dir, component.config);
        if (item.contains("pdu_robot") && item.at("pdu_robot").is_string()) {
            component.pdu_robot = item.at("pdu_robot").get<std::string>();
        }
        manifest.components.push_back(std::move(component));
    }

    out = std::move(manifest);
    return true;
}

bool ReadPduNameFromConfig(
    const std::string& config_path,
    std::string& out,
    std::string* error_message)
{
    json root;
    if (!load_json_file(config_path, root, error_message)) {
        return false;
    }
    double update_rate_hz = 0.0;
    ReadPduConfig(root, out, update_rate_hz);
    if (out.empty()) {
        if (error_message != nullptr) {
            *error_message = "pdu_config.pdu_name is missing: " + config_path;
        }
        return false;
    }
    return true;
}
}
