#pragma once

#include <string>

#include "config/asset_manifest.hpp"
#include "robots/tb3/tb3_robot.hpp"

namespace hako::robots::tb3
{
    struct Tb3RuntimeConfigOverrides
    {
        std::string lidar_config {};
    };

    std::string GetTb3ManifestPathFromEnvironment();

    Tb3RuntimeConfig LoadTb3RuntimeConfig(
        const hako::robots::config::AssetManifest& manifest,
        const Tb3RuntimeConfigOverrides& overrides = {});
}
