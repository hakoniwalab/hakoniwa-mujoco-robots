#pragma once

#include <cstdint>
#include <cstring>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "hako_asset.h"
#include "hako_asset_pdu.hpp"
#include "hako_msgs/pdu_cpptype_conv_ExecutionUnitRuntimeContext.hpp"
#include "hako_msgs/pdu_cpptype_conv_ExecutionUnitRuntimeStatus.hpp"

namespace hako::rd_lite {

enum class RuntimeStatusCode : std::uint8_t {
    OwnerStable = 0,
    OwnerReleasing = 1,
    OwnerActivating = 2,
};

struct RdLiteConfig {
    std::string asset_name {"forklift"};
    std::uint8_t node_id {1};
    std::uint8_t peer_node_id {2};
    bool initial_owner {true};
    std::uint32_t config_hash {0};
    double release_x {5.0};
    double home_x {0.0};
    double goal_tolerance {0.03};
    double switch_timeout_sec {2.0};
    std::size_t max_context_bytes {4096};
    std::string runtime_status_org_name {"runtime_status"};
    std::string runtime_context_org_name {"runtime_context"};
};

struct RuntimeStatusFrame {
    std::uint32_t config_hash {0};
    std::uint16_t unit_count {1};
    RuntimeStatusCode status {RuntimeStatusCode::OwnerStable};
    std::uint8_t epoch {0};
    std::uint8_t curr_owner_node_id {0};
    std::uint8_t next_owner_node_id {0};
};

struct RuntimeContextFrame {
    std::uint32_t config_hash {0};
    std::uint8_t epoch {0};
    std::uint8_t owner_id {0};
    std::vector<std::uint8_t> context;
};

class IRuntimeStatusStore {
public:
    virtual ~IRuntimeStatusStore() = default;
    virtual bool read(RuntimeStatusFrame& out) const = 0;
    virtual bool write(const RuntimeStatusFrame& in) = 0;
};

class IRuntimeContextStore {
public:
    virtual ~IRuntimeContextStore() = default;
    virtual bool read(RuntimeContextFrame& out) const = 0;
    virtual bool write(const RuntimeContextFrame& in) = 0;
};

namespace detail {
struct PduEndpoint {
    int channel_id {-1};
    int pdu_size {0};
    bool valid() const
    {
        return channel_id >= 0 && pdu_size > 0;
    }
};

inline bool resolve_pdu_endpoint(const std::string& robot_name, const std::string& org_name, PduEndpoint& out)
{
    std::vector<hako::asset::Robot> robots;
    if (!hako::asset::hako_asset_get_pdus(robots)) {
        return false;
    }
    for (const auto& robot : robots) {
        if (robot.name != robot_name) {
            continue;
        }
        for (const auto& writer : robot.pdu_writers) {
            if (writer.org_name == org_name) {
                out.channel_id = writer.channel_id;
                out.pdu_size = writer.pdu_size;
                return true;
            }
        }
        for (const auto& reader : robot.pdu_readers) {
            if (reader.org_name == org_name) {
                out.channel_id = reader.channel_id;
                out.pdu_size = reader.pdu_size;
                return true;
            }
        }
        return false;
    }
    return false;
}
} // namespace detail

class HakoPduRuntimeStatusStore : public IRuntimeStatusStore {
public:
    explicit HakoPduRuntimeStatusStore(std::string asset_name, std::string org_name = "runtime_status")
        : asset_name_(std::move(asset_name))
        , org_name_(std::move(org_name))
    {
    }

    bool initialize()
    {
        if (!detail::resolve_pdu_endpoint(asset_name_, org_name_, endpoint_)) {
            return false;
        }
        buffer_.assign(static_cast<std::size_t>(endpoint_.pdu_size), '\0');
        return endpoint_.valid();
    }

    bool read(RuntimeStatusFrame& out) const override
    {
        if (!endpoint_.valid() || buffer_.empty()) {
            return false;
        }
        if (hako_asset_pdu_read(asset_name_.c_str(), endpoint_.channel_id, const_cast<char*>(buffer_.data()), buffer_.size()) != 0) {
            return false;
        }
        auto* meta = reinterpret_cast<const HakoPduMetaDataType*>(buffer_.data());
        if (HAKO_PDU_METADATA_IS_INVALID(meta)) {
            return false;
        }

        HakoCpp_ExecutionUnitRuntimeStatus cpp {};
        hako::pdu::msgs::hako_msgs::ExecutionUnitRuntimeStatus conv;
        if (!conv.pdu2cpp(const_cast<char*>(buffer_.data()), cpp)) {
            return false;
        }
        if (cpp.unit_count == 0 || cpp.status.empty() || cpp.epoch.empty() ||
            cpp.curr_owner_node_id.empty() || cpp.next_owner_node_id.empty()) {
            return false;
        }
        out.config_hash = cpp.config_hash;
        out.unit_count = cpp.unit_count;
        out.status = static_cast<RuntimeStatusCode>(cpp.status[0]);
        out.epoch = cpp.epoch[0];
        out.curr_owner_node_id = cpp.curr_owner_node_id[0];
        out.next_owner_node_id = cpp.next_owner_node_id[0];
        return true;
    }

    bool write(const RuntimeStatusFrame& in) override
    {
        if (!endpoint_.valid() || buffer_.empty()) {
            return false;
        }
        HakoCpp_ExecutionUnitRuntimeStatus cpp {};
        cpp.config_hash = in.config_hash;
        cpp.unit_count = in.unit_count;
        cpp.status = {static_cast<std::uint8_t>(in.status)};
        cpp.epoch = {in.epoch};
        cpp.curr_owner_node_id = {in.curr_owner_node_id};
        cpp.next_owner_node_id = {in.next_owner_node_id};

        hako::pdu::msgs::hako_msgs::ExecutionUnitRuntimeStatus conv;
        const int actual_size = conv.cpp2pdu(cpp, buffer_.data(), static_cast<int>(buffer_.size()));
        if (actual_size <= 0 || actual_size > static_cast<int>(buffer_.size())) {
            return false;
        }
        return hako_asset_pdu_write(asset_name_.c_str(), endpoint_.channel_id, buffer_.data(), static_cast<std::size_t>(actual_size)) == 0;
    }

private:
    std::string asset_name_;
    std::string org_name_;
    detail::PduEndpoint endpoint_ {};
    mutable std::vector<char> buffer_ {};
};

class HakoPduRuntimeContextStore : public IRuntimeContextStore {
public:
    explicit HakoPduRuntimeContextStore(std::string asset_name, std::string org_name = "runtime_context")
        : asset_name_(std::move(asset_name))
        , org_name_(std::move(org_name))
    {
    }

    bool initialize()
    {
        if (!detail::resolve_pdu_endpoint(asset_name_, org_name_, endpoint_)) {
            return false;
        }
        buffer_.assign(static_cast<std::size_t>(endpoint_.pdu_size), '\0');
        return endpoint_.valid();
    }

    bool read(RuntimeContextFrame& out) const override
    {
        if (!endpoint_.valid() || buffer_.empty()) {
            return false;
        }
        if (hako_asset_pdu_read(asset_name_.c_str(), endpoint_.channel_id, const_cast<char*>(buffer_.data()), buffer_.size()) != 0) {
            return false;
        }
        auto* meta = reinterpret_cast<const HakoPduMetaDataType*>(buffer_.data());
        if (HAKO_PDU_METADATA_IS_INVALID(meta)) {
            return false;
        }

        HakoCpp_ExecutionUnitRuntimeContext cpp {};
        hako::pdu::msgs::hako_msgs::ExecutionUnitRuntimeContext conv;
        if (!conv.pdu2cpp(const_cast<char*>(buffer_.data()), cpp)) {
            return false;
        }
        out.config_hash = cpp.config_hash;
        out.epoch = cpp.epoch;
        out.owner_id = cpp.owner_id;
        out.context = cpp.context;
        return true;
    }

    bool write(const RuntimeContextFrame& in) override
    {
        if (!endpoint_.valid() || buffer_.empty()) {
            return false;
        }
        HakoCpp_ExecutionUnitRuntimeContext cpp {};
        cpp.config_hash = in.config_hash;
        cpp.epoch = in.epoch;
        cpp.owner_id = in.owner_id;
        cpp.context = in.context;

        hako::pdu::msgs::hako_msgs::ExecutionUnitRuntimeContext conv;
        const int actual_size = conv.cpp2pdu(cpp, buffer_.data(), static_cast<int>(buffer_.size()));
        if (actual_size <= 0 || actual_size > static_cast<int>(buffer_.size())) {
            return false;
        }
        return hako_asset_pdu_write(asset_name_.c_str(), endpoint_.channel_id, buffer_.data(), static_cast<std::size_t>(actual_size)) == 0;
    }

private:
    std::string asset_name_;
    std::string org_name_;
    detail::PduEndpoint endpoint_ {};
    mutable std::vector<char> buffer_ {};
};

class RdLiteOwnershipFSM {
public:
    explicit RdLiteOwnershipFSM(const RdLiteConfig& config)
        : config_(config)
    {
    }

    bool should_release(double pos_x) const
    {
        return pos_x >= (config_.release_x - config_.goal_tolerance);
    }

    bool should_takeback(double pos_x) const
    {
        return pos_x <= (config_.home_x + config_.goal_tolerance);
    }

private:
    RdLiteConfig config_;
};

class RdLiteCoordinator {
public:
    using SaveContextFn = std::function<bool(std::vector<std::uint8_t>& out_bytes)>;
    using RestoreContextFn = std::function<bool(const std::vector<std::uint8_t>& in_bytes)>;
    using LogFn = std::function<void(const std::string&)>;

    RdLiteCoordinator(
        RdLiteConfig config,
        IRuntimeStatusStore& status_store,
        IRuntimeContextStore& context_store)
        : config_(std::move(config))
        , status_store_(status_store)
        , context_store_(context_store)
        , fsm_(config_)
        , primary_owner_node_id_(config_.initial_owner ? config_.node_id : config_.peer_node_id)
    {
    }

    void set_logger(LogFn logger)
    {
        logger_ = std::move(logger);
    }

    bool initialize()
    {
        RuntimeStatusFrame status {};
        if (status_store_.read(status)) {
            return true;
        }
        status.config_hash = config_.config_hash;
        status.unit_count = 1;
        status.status = RuntimeStatusCode::OwnerStable;
        status.epoch = 0;
        status.curr_owner_node_id = config_.initial_owner ? config_.node_id : config_.peer_node_id;
        status.next_owner_node_id = status.curr_owner_node_id;
        const bool ok = status_store_.write(status);
        if (ok) {
            log("rd-lite: initialize default runtime_status");
        }
        return ok;
    }

    bool tick(double pos_x, const SaveContextFn& save_fn, const RestoreContextFn& restore_fn)
    {
        const auto now = std::chrono::steady_clock::now();
        RuntimeStatusFrame status {};
        if (!status_store_.read(status)) {
            // Runtime status can be temporarily unreadable during startup/handover.
            // Keep last known owner state instead of failing hard every tick.
            log_unreadable_status_once();
            return true;
        }
        unreadable_status_log_suppressed_ = false;
        local_owner_active_ = is_owner(status);
        has_observed_status_ = true;
        if (status.config_hash != 0 && config_.config_hash != 0 && status.config_hash != config_.config_hash) {
            log("rd-lite: config_hash mismatch");
            return false;
        }
        if (is_owner(status)) {
            if (status.status == RuntimeStatusCode::OwnerStable &&
                !is_in_switch_cooldown(now) &&
                should_release_for_role(pos_x)) {
                return release_ownership(status, save_fn);
            }
            return true;
        }
        if (status.status == RuntimeStatusCode::OwnerReleasing &&
            status.next_owner_node_id == config_.node_id &&
            status.curr_owner_node_id != config_.node_id) {
            return activate_from_context(status, restore_fn);
        }
        return true;
    }

    bool is_local_owner() const
    {
        RuntimeStatusFrame status {};
        if (!status_store_.read(status)) {
            // Fallback to last known state while status is temporarily unreadable.
            return has_observed_status_ ? local_owner_active_ : config_.initial_owner;
        }
        unreadable_status_log_suppressed_ = false;
        local_owner_active_ = is_owner(status);
        has_observed_status_ = true;
        if (status.curr_owner_node_id != config_.node_id) {
            return false;
        }
        return status.status != RuntimeStatusCode::OwnerReleasing;
    }

private:
    bool is_owner(const RuntimeStatusFrame& status) const
    {
        return status.curr_owner_node_id == config_.node_id;
    }

    bool is_primary_owner_role() const
    {
        return config_.node_id == primary_owner_node_id_;
    }

    bool should_release_for_role(double pos_x) const
    {
        // Primary owner releases at release_x, secondary owner returns at home_x.
        if (is_primary_owner_role()) {
            return fsm_.should_release(pos_x);
        }
        return fsm_.should_takeback(pos_x);
    }

    bool release_ownership(const RuntimeStatusFrame& current, const SaveContextFn& save_fn)
    {
        std::vector<std::uint8_t> bytes;
        if (!save_fn(bytes)) {
            log("rd-lite: save context failed");
            return false;
        }
        if (bytes.size() > config_.max_context_bytes) {
            log("rd-lite: context too large");
            return false;
        }

        RuntimeContextFrame ctx {};
        ctx.config_hash = config_.config_hash;
        ctx.epoch = static_cast<std::uint8_t>(current.epoch + 1);
        ctx.owner_id = config_.node_id;
        ctx.context = std::move(bytes);
        if (!context_store_.write(ctx)) {
            log("rd-lite: write runtime_context failed");
            return false;
        }

        RuntimeStatusFrame next = current;
        next.config_hash = config_.config_hash;
        next.status = RuntimeStatusCode::OwnerReleasing;
        next.epoch = ctx.epoch;
        next.curr_owner_node_id = config_.node_id;
        next.next_owner_node_id = config_.peer_node_id;
        if (!status_store_.write(next)) {
            log("rd-lite: write runtime_status(releasing) failed");
            return false;
        }
        mark_switch_now();
        log("rd-lite: ownership release requested");
        return true;
    }

    bool activate_from_context(const RuntimeStatusFrame& requested, const RestoreContextFn& restore_fn)
    {
        RuntimeContextFrame ctx {};
        if (!context_store_.read(ctx)) {
            log("rd-lite: read runtime_context failed");
            return false;
        }
        if (ctx.epoch != requested.epoch) {
            log("rd-lite: runtime_context epoch mismatch");
            return false;
        }
        if (!restore_fn(ctx.context)) {
            log("rd-lite: restore context failed");
            return false;
        }

        RuntimeStatusFrame activating = requested;
        activating.status = RuntimeStatusCode::OwnerActivating;
        activating.curr_owner_node_id = config_.node_id;
        activating.next_owner_node_id = config_.node_id;
        if (!status_store_.write(activating)) {
            log("rd-lite: write runtime_status(activating) failed");
            return false;
        }

        RuntimeStatusFrame stable = activating;
        stable.status = RuntimeStatusCode::OwnerStable;
        if (!status_store_.write(stable)) {
            log("rd-lite: write runtime_status(stable) failed");
            return false;
        }
        mark_switch_now();
        log("rd-lite: ownership activated");
        return true;
    }

    void log(const std::string& msg) const
    {
        if (logger_) {
            logger_(msg);
        }
    }

    void log_unreadable_status_once() const
    {
        if (!unreadable_status_log_suppressed_) {
            log("rd-lite: runtime_status unreadable, keep last ownership state");
            unreadable_status_log_suppressed_ = true;
        }
    }

    bool is_in_switch_cooldown(const std::chrono::steady_clock::time_point& now) const
    {
        if (!has_last_switch_tp_) {
            return false;
        }
        if (config_.switch_timeout_sec <= 0.0) {
            return false;
        }
        const auto elapsed_sec =
            std::chrono::duration<double>(now - last_switch_tp_).count();
        return elapsed_sec < config_.switch_timeout_sec;
    }

    void mark_switch_now()
    {
        last_switch_tp_ = std::chrono::steady_clock::now();
        has_last_switch_tp_ = true;
    }

    RdLiteConfig config_;
    IRuntimeStatusStore& status_store_;
    IRuntimeContextStore& context_store_;
    RdLiteOwnershipFSM fsm_;
    std::uint8_t primary_owner_node_id_ {0};
    LogFn logger_ {};
    mutable bool has_observed_status_ {false};
    mutable bool local_owner_active_ {false};
    mutable bool unreadable_status_log_suppressed_ {false};
    bool has_last_switch_tp_ {false};
    std::chrono::steady_clock::time_point last_switch_tp_ {};
};

} // namespace hako::rd_lite
