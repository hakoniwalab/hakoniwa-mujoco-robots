#pragma once

#include <memory>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/common/update_scheduler.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::camera
{
    // Forward declaration
    class MujocoCameraRenderer;

    // Common Configuration structures
    struct ImageConfig
    {
        int width = 640;
        int height = 480;
        std::string format = "R8G8B8";
    };

    struct ClipConfig
    {
        double near = 0.1;
        double far = 10.0;
    };

    struct CameraNoiseConfig
    {
        std::string type = "gaussian";
        double mean = 0.0;
        double stddev = 0.0;
    };

    // Standard Camera Config
    struct CameraConfig
    {
        std::string frame_id = "camera";
        double update_rate = 30.0;
        double horizontal_fov = 1.39626;
        ImageConfig image;
        ClipConfig clip;
        CameraNoiseConfig noise;
    };

    // Depth Camera Config
    struct DepthCameraConfig
    {
        std::string frame_id = "camera_depth";
        double update_rate = 30.0;
        double horizontal_fov = 1.047;
        ImageConfig image{640, 480, "DEPTH_F32_M"};
        ClipConfig clip;
        CameraNoiseConfig noise;
    };

    // RGBD Camera Config
    struct RgbdCameraConfig
    {
        CameraConfig rgb;
        DepthCameraConfig depth;
    };

    struct StereoCameraConfig
    {
        CameraConfig left;
        CameraConfig right;
        double baseline = 0.0;
    };


    // Output frame structures
    struct ImageFrame
    {
        int width = 0;
        int height = 0;
        int channels = 0;
        std::string format;
        std::string frame_id;
        std::vector<uint8_t> data;
        double timestamp = 0.0;
    };

    struct DepthFrame
    {
        int width = 0;
        int height = 0;
        std::string format = "DEPTH_F32_M";
        std::string frame_id;
        // Encoded depth samples are exposed as float meters.
        // DEPTH_U16_MM remains a downstream serialization/storage hint.
        std::vector<float> data;
        double timestamp = 0.0;
    };

    // --- Base Class for common logic ---
    class CameraSensorBase : public ISensor
    {
    public:
        void Reset() override
        {
            scheduler_.Reset();
        }
        bool ShouldUpdate(double delta_sec) override
        {
            return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
        }

    protected:
        void StartScheduler(double update_rate)
        {
            update_rate_ = update_rate;
            scheduler_.StartReady(GetUpdatePeriodSec());
        }
        double GetUpdatePeriodSec() const override
        {
            if (update_rate_ <= 0) {
                return 0.1; // Default to 10Hz if rate is invalid
            }
            return 1.0 / update_rate_;
        }

    private:
        common::UpdateScheduler scheduler_;
        double update_rate_ = 30.0;
    };


    // --- Interfaces ---

    // Interface for a standard camera sensor
    class ICameraSensor : public CameraSensorBase
    {
    public:
        virtual ~ICameraSensor() = default;
        virtual bool LoadConfig(const CameraConfig& config) = 0;
        virtual const CameraConfig& GetConfig() const = 0;
        virtual void Capture(ImageFrame& out) = 0;
    };

    // Interface for a depth camera sensor
    class IDepthCameraSensor : public CameraSensorBase
    {
    public:
        virtual ~IDepthCameraSensor() = default;
        virtual bool LoadConfig(const DepthCameraConfig& config) = 0;
        virtual const DepthCameraConfig& GetConfig() const = 0;
        virtual void Capture(DepthFrame& out) = 0;
    };

    // Interface for an RGBD camera sensor
    class IRgbdCameraSensor : public CameraSensorBase
    {
    public:
        virtual ~IRgbdCameraSensor() = default;
        virtual bool LoadConfig(const RgbdCameraConfig& config) = 0;
        virtual const RgbdCameraConfig& GetConfig() const = 0;
        virtual void Capture(ImageFrame& rgb_out, DepthFrame& depth_out) = 0;
    };

    class IStereoCameraSensor : public CameraSensorBase
    {
    public:
        virtual ~IStereoCameraSensor() = default;
        virtual bool LoadConfig(const StereoCameraConfig& config) = 0;
        virtual const StereoCameraConfig& GetConfig() const = 0;
        virtual void Capture(ImageFrame& left_out, ImageFrame& right_out) = 0;
    };


    // --- Concrete Implementation ---
    class CameraSensor : public ICameraSensor
    {
    public:
        CameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name);
        ~CameraSensor() override;

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const CameraConfig& config) override;
        const CameraConfig& GetConfig() const override;
        void Capture(ImageFrame& out) override;
    private:
        std::shared_ptr<MujocoCameraRenderer> renderer_;
        std::string camera_name_;
        CameraConfig config_;
    };

    class DepthCameraSensor : public IDepthCameraSensor
    {
    public:
        DepthCameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name);
        ~DepthCameraSensor() override;

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const DepthCameraConfig& config) override;
        const DepthCameraConfig& GetConfig() const override;
        void Capture(DepthFrame& out) override;
    private:
        std::shared_ptr<MujocoCameraRenderer> renderer_;
        std::string camera_name_;
        DepthCameraConfig config_;
    };

    class RgbdCameraSensor : public IRgbdCameraSensor
    {
    public:
        RgbdCameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name);
        ~RgbdCameraSensor() override;

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const RgbdCameraConfig& config) override;
        const RgbdCameraConfig& GetConfig() const override;
        void Capture(ImageFrame& rgb_out, DepthFrame& depth_out) override;
    private:
        std::shared_ptr<MujocoCameraRenderer> renderer_;
        std::string camera_name_;
        RgbdCameraConfig config_;
    };

    class StereoCameraSensor : public IStereoCameraSensor
    {
    public:
        StereoCameraSensor(
            std::shared_ptr<MujocoCameraRenderer> renderer,
            const std::string& left_camera_name,
            const std::string& right_camera_name);
        ~StereoCameraSensor() override;

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const StereoCameraConfig& config) override;
        const StereoCameraConfig& GetConfig() const override;
        void Capture(ImageFrame& left_out, ImageFrame& right_out) override;

    private:
        std::shared_ptr<MujocoCameraRenderer> renderer_;
        std::string left_camera_name_;
        std::string right_camera_name_;
        StereoCameraConfig config_;
    };
}
