#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hako::rd_lite {
class HakoPduRuntimeStatusStore;
class HakoPduRuntimeContextStore;
class RdLiteCoordinator;
}

class RdLightIntegration {
public:
    using SaveContextCallback = std::function<bool(std::vector<std::uint8_t>&)>;
    using RestoreContextCallback = std::function<bool(const std::vector<std::uint8_t>&)>;

    RdLightIntegration();
    ~RdLightIntegration();

    static bool is_enabled_from_env();
    static bool is_initial_owner_from_env();

    bool initialize(
        const std::string& asset_name,
        SaveContextCallback save_callback,
        RestoreContextCallback restore_callback);

    bool is_enabled() const;
    bool is_local_owner() const;
    bool tick(double pos_x);

    int status_log_every_steps() const;
    double release_x() const;
    double home_x() const;
    bool standby_physics_enabled() const;

private:
    bool enabled_ {false};
    bool standby_physics_enabled_ {true};
    int status_log_every_steps_ {1000};
    double release_x_ {5.0};
    double home_x_ {0.0};

    SaveContextCallback save_callback_ {};
    RestoreContextCallback restore_callback_ {};

    std::unique_ptr<hako::rd_lite::HakoPduRuntimeStatusStore> status_store_ {};
    std::unique_ptr<hako::rd_lite::HakoPduRuntimeContextStore> context_store_ {};
    std::unique_ptr<hako::rd_lite::RdLiteCoordinator> coordinator_ {};
};
