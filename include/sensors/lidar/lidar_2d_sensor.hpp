#pragma once

#include <memory>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::lidar
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
        double precision {0.0};
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

    class ILidar2DSensor : public ISensor
    {
    public:
        virtual ~ILidar2DSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual void SetRuntimeOptions(double yaw_bias_deg, double origin_offset_m) = 0;
        virtual const LiDAR2DConfig& GetConfig() const = 0;

        // Capture one full scan frame according to the current config.
        virtual void Scan(LaserScanFrame& out) = 0;
    };

    class LiDAR2DSensor : public ILidar2DSensor
    {
    public:
        LiDAR2DSensor(
            std::shared_ptr<hako::robots::physics::IWorld> world,
            std::string sensor_body_name = "base_scan",
            std::string exclude_body_name = "base_footprint");

        bool LoadConfig(const std::string& config_path) override;
        void SetRuntimeOptions(double yaw_bias_deg, double origin_offset_m) override;
        const LiDAR2DConfig& GetConfig() const override;
        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;
        void Scan(LaserScanFrame& out) override;

    private:
        float CastRay(
            const mjModel* model,
            mjData* data,
            const mjtNum* sensor_pos,
            int body_exclude,
            double base_yaw_rad,
            double degree_yaw) const;
        bool IsSelfGeom(const mjModel* model, int body_exclude, int geom_id) const;
        void ApplyBlindPadding(std::vector<float>& ranges) const;
        void RebuildNoisePipeline();

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::shared_ptr<hako::robots::physics::IRigidBody> sensor_body_;
        std::string sensor_body_name_;
        std::string exclude_body_name_;
        LiDAR2DConfig config_ {};
        double elapsed_sec_ {0.0};
        noise::RangeNoisePipeline noise_pipeline_;
    };
}
