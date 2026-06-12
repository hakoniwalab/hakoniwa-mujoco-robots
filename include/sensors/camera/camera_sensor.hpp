#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
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

    struct RGBAColor
    {
        float r = 0.0F;
        float g = 0.0F;
        float b = 0.0F;
        float a = 0.0F;
    };

    inline bool TryExtractRGBAColor(
        const ImageFrame& frame,
        RGBAColor& out,
        int x = -1,
        int y = -1)
    {
        out = {};
        if (frame.width <= 0 || frame.height <= 0) {
            return false;
        }

        int channels = 0;
        if (frame.format == "R8G8B8" || frame.format == "B8G8R8") {
            channels = 3;
        } else if (frame.format == "L8") {
            channels = 1;
        } else {
            return false;
        }

        const std::size_t expected_size =
            static_cast<std::size_t>(frame.width) *
            static_cast<std::size_t>(frame.height) *
            static_cast<std::size_t>(channels);
        if (frame.data.size() != expected_size) {
            return false;
        }

        const int px = (x < 0) ? (frame.width / 2) : std::clamp(x, 0, frame.width - 1);
        const int py = (y < 0) ? (frame.height / 2) : std::clamp(y, 0, frame.height - 1);
        const std::size_t idx =
            (static_cast<std::size_t>(py) * static_cast<std::size_t>(frame.width) +
             static_cast<std::size_t>(px)) *
            static_cast<std::size_t>(channels);

        constexpr float kInv255 = 1.0F / 255.0F;
        if (frame.format == "R8G8B8") {
            out = {
                static_cast<float>(frame.data[idx + 0]) * kInv255,
                static_cast<float>(frame.data[idx + 1]) * kInv255,
                static_cast<float>(frame.data[idx + 2]) * kInv255,
                1.0F
            };
            return true;
        }
        if (frame.format == "B8G8R8") {
            out = {
                static_cast<float>(frame.data[idx + 2]) * kInv255,
                static_cast<float>(frame.data[idx + 1]) * kInv255,
                static_cast<float>(frame.data[idx + 0]) * kInv255,
                1.0F
            };
            return true;
        }

        const float luminance = static_cast<float>(frame.data[idx]) * kInv255;
        out = {luminance, luminance, luminance, 1.0F};
        return true;
    }

    inline bool TryExtractAverageRGBAColor(
        const ImageFrame& frame,
        RGBAColor& out,
        int x,
        int y,
        int width,
        int height)
    {
        out = {};
        if (frame.width <= 0 || frame.height <= 0 || width <= 0 || height <= 0) {
            return false;
        }

        int channels = 0;
        if (frame.format == "R8G8B8" || frame.format == "B8G8R8") {
            channels = 3;
        } else if (frame.format == "L8") {
            channels = 1;
        } else {
            return false;
        }

        const std::size_t expected_size =
            static_cast<std::size_t>(frame.width) *
            static_cast<std::size_t>(frame.height) *
            static_cast<std::size_t>(channels);
        if (frame.data.size() != expected_size) {
            return false;
        }

        const int x0 = std::clamp(x, 0, frame.width);
        const int y0 = std::clamp(y, 0, frame.height);
        const int x1 = std::clamp(x + width, 0, frame.width);
        const int y1 = std::clamp(y + height, 0, frame.height);
        if (x0 >= x1 || y0 >= y1) {
            return false;
        }

        double r = 0.0;
        double g = 0.0;
        double b = 0.0;
        std::size_t count = 0;
        for (int py = y0; py < y1; ++py) {
            for (int px = x0; px < x1; ++px) {
                const std::size_t idx =
                    (static_cast<std::size_t>(py) * static_cast<std::size_t>(frame.width) +
                     static_cast<std::size_t>(px)) *
                    static_cast<std::size_t>(channels);
                if (frame.format == "R8G8B8") {
                    r += frame.data[idx + 0];
                    g += frame.data[idx + 1];
                    b += frame.data[idx + 2];
                } else if (frame.format == "B8G8R8") {
                    r += frame.data[idx + 2];
                    g += frame.data[idx + 1];
                    b += frame.data[idx + 0];
                } else {
                    r += frame.data[idx];
                    g += frame.data[idx];
                    b += frame.data[idx];
                }
                ++count;
            }
        }

        if (count == 0) {
            return false;
        }

        constexpr float kInv255 = 1.0F / 255.0F;
        out = {
            static_cast<float>(r / static_cast<double>(count)) * kInv255,
            static_cast<float>(g / static_cast<double>(count)) * kInv255,
            static_cast<float>(b / static_cast<double>(count)) * kInv255,
            1.0F
        };
        return true;
    }

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

        virtual RGBAColor CaptureAsRGBA(int x = -1, int y = -1)
        {
            ImageFrame frame {};
            Capture(frame);
            RGBAColor color {};
            TryExtractRGBAColor(frame, color, x, y);
            return color;
        }

        virtual RGBAColor CaptureRegionAverageRGBA(int x, int y, int width, int height)
        {
            ImageFrame frame {};
            Capture(frame);
            RGBAColor color {};
            TryExtractAverageRGBAColor(frame, color, x, y, width, height);
            return color;
        }
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
