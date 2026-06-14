#pragma once

#include <string>
#include <vector>

namespace hako::robots::config
{
    struct AssetManifestComponent
    {
        std::string id {};
        std::string kind {};
        std::string type {};
        std::string config {};
        std::string pdu_robot {};
    };

    struct AssetManifest
    {
        std::string path {};
        std::string name {};
        std::string model {};
        std::string pdu_def {};
        std::string endpoint {};
        std::vector<AssetManifestComponent> components {};

        const AssetManifestComponent* FindComponent(const std::string& id) const;
        std::string ComponentConfig(const std::string& id) const;
        std::string ComponentPduRobot(const std::string& id) const;
    };

    bool LoadAssetManifestFromJson(
        const std::string& manifest_path,
        AssetManifest& out,
        std::string* error_message = nullptr);

    bool ReadPduNameFromConfig(
        const std::string& config_path,
        std::string& out,
        std::string* error_message = nullptr);
}
