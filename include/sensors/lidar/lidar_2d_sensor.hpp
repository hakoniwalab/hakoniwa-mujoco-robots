#pragma once

#include <memory>
#include <string>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::lidar
{
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
