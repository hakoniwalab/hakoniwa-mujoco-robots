#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "hako_asset.h"
#include "hakoniwa/pdu/endpoint.hpp"

namespace hako::robots::runtime
{
    struct HakoniwaAssetLifecycleConfig
    {
        std::string endpoint_name {};
        std::string endpoint_config_path {};
        std::string asset_name {};
        std::string pdu_def_path {};
        hako_time_t delta_time_usec {0};
        HakoAssetModelType model_type {HAKO_ASSET_MODEL_PLANT};
        hako_time_t conductor_cycle_usec {100000};
        bool start_conductor {true};
    };

    class HakoniwaAssetLifecycle
    {
    public:
        using ManualTimingCallback = std::function<int(hakoniwa::pdu::Endpoint&)>;
        using ResetCallback = std::function<int()>;
        using ForceStopCallback = std::function<int()>;

        explicit HakoniwaAssetLifecycle(HakoniwaAssetLifecycleConfig config);
        ~HakoniwaAssetLifecycle();

        HakoniwaAssetLifecycle(const HakoniwaAssetLifecycle&) = delete;
        HakoniwaAssetLifecycle& operator=(const HakoniwaAssetLifecycle&) = delete;

        bool OpenEndpoint(std::string* error_message = nullptr);
        bool RegisterAndRunAsset(
            ManualTimingCallback manual_timing,
            ResetCallback reset = {},
            std::string* error_message = nullptr);
        bool RegisterAndRunAssetNoWait(
            ManualTimingCallback manual_timing,
            ForceStopCallback force_stop,
            ResetCallback reset = {},
            std::string* error_message = nullptr);

        bool IsReady() const;
        hakoniwa::pdu::Endpoint& Endpoint();

        void StopAndClose();

    private:
        int OnInitialize();
        int OnManualTiming();
        int OnReset();
        int ShouldStop() const;

        static int StaticOnInitialize(hako_asset_context_t* context);
        static int StaticOnManualTiming(hako_asset_context_t* context);
        static int StaticOnReset(hako_asset_context_t* context);
        static int StaticShouldStop();

        bool RegisterAndRunAssetInternal(
            ManualTimingCallback manual_timing,
            ResetCallback reset,
            ForceStopCallback force_stop,
            bool no_wait,
            std::string* error_message);

        static HakoniwaAssetLifecycle* active_instance_;

        HakoniwaAssetLifecycleConfig config_;
        hakoniwa::pdu::Endpoint endpoint_;
        hako_asset_callbacks_t callbacks_ {};
        ManualTimingCallback manual_timing_ {};
        ResetCallback reset_ {};
        ForceStopCallback force_stop_ {};
        std::atomic_bool ready_ {false};
        std::atomic_bool endpoint_opened_ {false};
        std::atomic_bool conductor_started_ {false};
    };
}
