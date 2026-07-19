#include "runtime/hakoniwa_asset_lifecycle.hpp"

#include <iostream>
#include <utility>

#include "hako_conductor.h"

namespace hako::robots::runtime
{
HakoniwaAssetLifecycle* HakoniwaAssetLifecycle::active_instance_ = nullptr;

HakoniwaAssetLifecycle::HakoniwaAssetLifecycle(HakoniwaAssetLifecycleConfig config)
    : config_(std::move(config))
    , endpoint_(config_.endpoint_name, HAKO_PDU_ENDPOINT_DIRECTION_INOUT)
{
}

HakoniwaAssetLifecycle::~HakoniwaAssetLifecycle()
{
    StopAndClose();
    if (active_instance_ == this) {
        active_instance_ = nullptr;
    }
}

bool HakoniwaAssetLifecycle::OpenEndpoint(std::string* error_message)
{
    if (endpoint_opened_.load()) {
        return true;
    }
    if (endpoint_.open(config_.endpoint_config_path) != HAKO_PDU_ERR_OK) {
        if (error_message != nullptr) {
            *error_message = "endpoint.open() failed: " + config_.endpoint_config_path;
        }
        return false;
    }
    if (endpoint_.start() != HAKO_PDU_ERR_OK) {
        if (error_message != nullptr) {
            *error_message = "endpoint.start() failed: " + config_.endpoint_name;
        }
        endpoint_.close();
        return false;
    }
    endpoint_opened_.store(true);
    return true;
}

bool HakoniwaAssetLifecycle::RegisterAndRunAsset(
    ManualTimingCallback manual_timing,
    ResetCallback reset,
    std::string* error_message)
{
    return RegisterAndRunAssetInternal(
        std::move(manual_timing),
        std::move(reset),
        {},
        false,
        error_message);
}

bool HakoniwaAssetLifecycle::RegisterAndRunAssetNoWait(
    ManualTimingCallback manual_timing,
    ForceStopCallback force_stop,
    ResetCallback reset,
    std::string* error_message)
{
    return RegisterAndRunAssetInternal(
        std::move(manual_timing),
        std::move(reset),
        std::move(force_stop),
        true,
        error_message);
}

bool HakoniwaAssetLifecycle::RegisterAndRunAssetInternal(
    ManualTimingCallback manual_timing,
    ResetCallback reset,
    ForceStopCallback force_stop,
    bool no_wait,
    std::string* error_message)
{
    if (active_instance_ != nullptr && active_instance_ != this) {
        if (error_message != nullptr) {
            *error_message = "another HakoniwaAssetLifecycle instance is already active";
        }
        return false;
    }
    if (!endpoint_opened_.load()) {
        if (!OpenEndpoint(error_message)) {
            return false;
        }
    }

    manual_timing_ = std::move(manual_timing);
    reset_ = std::move(reset);
    force_stop_ = std::move(force_stop);
    callbacks_ = {};
    callbacks_.on_initialize = StaticOnInitialize;
    callbacks_.on_simulation_step = nullptr;
    callbacks_.on_manual_timing_control = StaticOnManualTiming;
    callbacks_.on_reset = StaticOnReset;
    active_instance_ = this;

    if (config_.start_conductor) {
        hako_conductor_start(config_.delta_time_usec, config_.conductor_cycle_usec);
        conductor_started_.store(true);
    } else {
        std::cout << "[INFO] Conductor start is disabled for asset: "
                  << config_.asset_name << std::endl;
    }
    const int register_result = hako_asset_register(
        config_.asset_name.c_str(),
        config_.pdu_def_path.c_str(),
        &callbacks_,
        config_.delta_time_usec,
        config_.model_type);
    if (register_result != 0) {
        if (error_message != nullptr) {
            *error_message = "hako_asset_register() returns " + std::to_string(register_result);
        }
        return false;
    }

    const int start_result = no_wait
        ? hako_asset_start_no_wait(StaticShouldStop)
        : hako_asset_start();
    if (start_result != 0) {
        if (error_message != nullptr) {
            *error_message = std::string(no_wait ? "hako_asset_start_no_wait() returns " : "hako_asset_start() returns ")
                + std::to_string(start_result);
        }
        return false;
    }
    return true;
}

bool HakoniwaAssetLifecycle::IsReady() const
{
    return ready_.load();
}

hakoniwa::pdu::Endpoint& HakoniwaAssetLifecycle::Endpoint()
{
    return endpoint_;
}

void HakoniwaAssetLifecycle::StopAndClose()
{
    ready_.store(false);
    if (endpoint_opened_.exchange(false)) {
        (void)endpoint_.stop();
        endpoint_.close();
    }
    if (conductor_started_.exchange(false)) {
        hako_conductor_stop();
    }
}

int HakoniwaAssetLifecycle::OnInitialize()
{
    if (!endpoint_opened_.load()) {
        std::cerr << "[ERROR] Endpoint is not initialized." << std::endl;
        return -1;
    }
    if (endpoint_.post_start() != HAKO_PDU_ERR_OK) {
        std::cerr << "[ERROR] Failed to complete endpoint post_start." << std::endl;
        return -1;
    }
    ready_.store(true);
    return 0;
}

int HakoniwaAssetLifecycle::OnManualTiming()
{
    if (!manual_timing_) {
        std::cerr << "[ERROR] Manual timing callback is not configured." << std::endl;
        return -1;
    }
    return manual_timing_(endpoint_);
}

int HakoniwaAssetLifecycle::OnReset()
{
    if (reset_) {
        return reset_();
    }
    return 0;
}

int HakoniwaAssetLifecycle::ShouldStop() const
{
    if (force_stop_) {
        return force_stop_();
    }
    return 0;
}

int HakoniwaAssetLifecycle::StaticOnInitialize(hako_asset_context_t* context)
{
    (void)context;
    return active_instance_ != nullptr ? active_instance_->OnInitialize() : -1;
}

int HakoniwaAssetLifecycle::StaticOnManualTiming(hako_asset_context_t* context)
{
    (void)context;
    return active_instance_ != nullptr ? active_instance_->OnManualTiming() : -1;
}

int HakoniwaAssetLifecycle::StaticOnReset(hako_asset_context_t* context)
{
    (void)context;
    return active_instance_ != nullptr ? active_instance_->OnReset() : -1;
}

int HakoniwaAssetLifecycle::StaticShouldStop()
{
    return active_instance_ != nullptr ? active_instance_->ShouldStop() : 1;
}
}
