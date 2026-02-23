#include "rd_light_integration.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include "rd_lite/rd_lite.hpp"

namespace {
int get_env_int(const char* name, int default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stoi(env);
    } catch (...) {
        return default_value;
    }
}

double get_env_double(const char* name, double default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stod(env);
    } catch (...) {
        return default_value;
    }
}

std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}
} // namespace

RdLightIntegration::RdLightIntegration() = default;
RdLightIntegration::~RdLightIntegration() = default;

bool RdLightIntegration::is_enabled_from_env()
{
    return get_env_int("HAKO_RD_LITE_ENABLE", 0) != 0;
}

bool RdLightIntegration::is_initial_owner_from_env()
{
    return get_env_int("HAKO_RD_LITE_INITIAL_OWNER", 1) != 0;
}

bool RdLightIntegration::initialize(
    const std::string& asset_name,
    SaveContextCallback save_callback,
    RestoreContextCallback restore_callback)
{
    save_callback_ = std::move(save_callback);
    restore_callback_ = std::move(restore_callback);

    status_log_every_steps_ = std::max(1, get_env_int("HAKO_RD_LITE_STATUS_LOG_EVERY_STEPS", 1000));
    release_x_ = get_env_double("HAKO_RD_LITE_RELEASE_X", get_env_double("FORWARD_GOAL_X", 5.0));
    home_x_ = get_env_double("HAKO_RD_LITE_HOME_X", get_env_double("HOME_GOAL_X", 0.0));
    standby_physics_enabled_ = (get_env_int("HAKO_RD_LITE_STANDBY_PHYSICS", 1) != 0);

    enabled_ = is_enabled_from_env();
    if (!enabled_) {
        return true;
    }

    hako::rd_lite::RdLiteConfig cfg {};
    cfg.asset_name = asset_name;
    cfg.node_id = static_cast<std::uint8_t>(std::clamp(get_env_int("HAKO_RD_LITE_NODE_ID", 1), 0, 255));
    cfg.peer_node_id = static_cast<std::uint8_t>(std::clamp(get_env_int("HAKO_RD_LITE_PEER_NODE_ID", 2), 0, 255));
    cfg.initial_owner = is_initial_owner_from_env();
    cfg.config_hash = static_cast<std::uint32_t>(get_env_int("HAKO_RD_LITE_CONFIG_HASH", 0));
    cfg.release_x = release_x_;
    cfg.home_x = home_x_;
    cfg.goal_tolerance = get_env_double("GOAL_TOLERANCE", 0.03);
    cfg.switch_timeout_sec = get_env_double("HAKO_RD_LITE_SWITCH_TIMEOUT_SEC", 2.0);
    cfg.max_context_bytes = static_cast<std::size_t>(std::max(0, get_env_int("HAKO_RD_LITE_MAX_CONTEXT_BYTES", 4096)));
    cfg.runtime_status_org_name = get_env_string("HAKO_RD_LITE_RUNTIME_STATUS_NAME", "runtime_status");
    cfg.runtime_context_org_name = get_env_string("HAKO_RD_LITE_RUNTIME_CONTEXT_NAME", "runtime_context");

    status_store_ = std::make_unique<hako::rd_lite::HakoPduRuntimeStatusStore>(
        cfg.asset_name, cfg.runtime_status_org_name);
    context_store_ = std::make_unique<hako::rd_lite::HakoPduRuntimeContextStore>(
        cfg.asset_name, cfg.runtime_context_org_name);
    if (!status_store_->initialize() || !context_store_->initialize()) {
        std::cerr << "[WARN] RD-lite disabled: runtime_status/runtime_context PDU not found for asset="
                  << cfg.asset_name << std::endl;
        enabled_ = false;
        status_store_.reset();
        context_store_.reset();
        return false;
    }

    coordinator_ = std::make_unique<hako::rd_lite::RdLiteCoordinator>(
        cfg, *status_store_, *context_store_);
    coordinator_->set_logger([](const std::string& msg) {
        std::cout << "[RD-LITE] " << msg << std::endl;
    });
    bool init_ok = true;
    if (cfg.initial_owner) {
        init_ok = coordinator_->initialize();
    } else {
        std::cout << "[INFO] RD-lite standby mode: skip runtime_status initialization" << std::endl;
    }
    if (!init_ok) {
        std::cerr << "[WARN] RD-lite disabled: coordinator initialize failed on initial owner" << std::endl;
        enabled_ = false;
        coordinator_.reset();
        status_store_.reset();
        context_store_.reset();
        return false;
    }

    std::cout << "[INFO] RD-lite enabled. node_id="
              << static_cast<int>(cfg.node_id)
              << " peer_id=" << static_cast<int>(cfg.peer_node_id)
              << " owner=" << (cfg.initial_owner ? "yes" : "no")
              << " release_x=" << cfg.release_x
              << " home_x=" << cfg.home_x
              << " tol=" << cfg.goal_tolerance
              << std::endl;
    return true;
}

bool RdLightIntegration::is_enabled() const
{
    return enabled_;
}

bool RdLightIntegration::is_local_owner() const
{
    return (!enabled_) || (!coordinator_) || coordinator_->is_local_owner();
}

bool RdLightIntegration::tick(double pos_x)
{
    if (!enabled_ || !coordinator_) {
        return true;
    }
    if (!save_callback_ || !restore_callback_) {
        return false;
    }
    return coordinator_->tick(pos_x, save_callback_, restore_callback_);
}

int RdLightIntegration::status_log_every_steps() const
{
    return status_log_every_steps_;
}

double RdLightIntegration::release_x() const
{
    return release_x_;
}

double RdLightIntegration::home_x() const
{
    return home_x_;
}

bool RdLightIntegration::standby_physics_enabled() const
{
    return standby_physics_enabled_;
}
