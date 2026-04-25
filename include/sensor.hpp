#pragma once

#include <string>
#include <vector>

namespace hako::robots::sensor
{
    struct DetectionDistance
    {
        double min {0.0};
        double max {0.0};
    };

    struct BlindPaddingRange
    {
        int size {0};
        double value {0.0};
        bool enabled {false};
    };

    struct AngleRange
    {
        double min_deg {0.0};
        double max_deg {0.0};
        bool ascending_order_of_data {true};
        double resolution_deg {1.0};
        int scan_frequency_hz {10};
        BlindPaddingRange blind_padding {};
    };

    struct DistanceAccuracy
    {
        DetectionDistance range {};
        bool distance_dependent {false};
        double percentage {0.0};
        double stddev {0.0};
        std::string noise_distribution {"Gaussian"};
    };

    struct LiDAR2DConfig
    {
        std::string frame_id {"laser"};
        DetectionDistance detection_distance {};
        AngleRange angle_range {};
        std::vector<DistanceAccuracy> distance_accuracy {};
        double yaw_bias_deg {0.0};
        double origin_offset_m {0.0};
    };

    struct LaserScanFrame
    {
        std::string frame_id {"laser"};
        float angle_min {0.0F};
        float angle_max {0.0F};
        float angle_increment {0.0F};
        float time_increment {0.0F};
        float scan_time {0.0F};
        float range_min {0.0F};
        float range_max {0.0F};
        std::vector<float> ranges {};
        std::vector<float> intensities {};
    };

    class ISensor
    {
    public:
        virtual ~ISensor() = default;
        virtual void Reset() = 0;
        virtual double GetUpdatePeriodSec() const = 0;
        virtual bool ShouldUpdate(double delta_sec) = 0;
    };

    class ILidarSensor2D : public ISensor
    {
    public:
        virtual ~ILidarSensor2D() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual void SetRuntimeOptions(double yaw_bias_deg, double origin_offset_m) = 0;
        virtual const LiDAR2DConfig& GetConfig() const = 0;

        // Capture one full scan frame according to the current config.
        virtual void Scan(LaserScanFrame& out) = 0;
    };
}
