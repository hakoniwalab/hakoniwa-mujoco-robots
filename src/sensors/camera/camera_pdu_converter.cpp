#include "sensors/camera/camera_pdu_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

namespace hako::robots::sensor::camera
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

HakoCpp_Time ToHakoTime(double stamp_sec)
{
    HakoCpp_Time out {};
    const double sec_floor = std::floor(stamp_sec);
    out.sec = static_cast<Hako_int32>(sec_floor);
    long long nanosec = std::llround((stamp_sec - sec_floor) * 1.0e9);
    if (nanosec >= 1000000000LL) {
        out.sec += 1;
        nanosec -= 1000000000LL;
    }
    out.nanosec = static_cast<Hako_uint32>(std::max<long long>(0, nanosec));
    return out;
}

void FillHeader(const std::string& frame_id, double timestamp, HakoCpp_Header& out)
{
    out.stamp = ToHakoTime(timestamp);
    out.frame_id = frame_id;
}

bool ValidateImageFrameCommon(
    const std::string& format,
    int width,
    int height,
    std::size_t data_size,
    int channels)
{
    if (width <= 0 || height <= 0) {
        std::cerr << "Failed to convert ImageFrame: invalid image size "
                  << width << "x" << height << std::endl;
        return false;
    }
    if (channels <= 0) {
        std::cerr << "Failed to convert ImageFrame: invalid channels for format '"
                  << format << "'" << std::endl;
        return false;
    }
    const std::size_t expected_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * static_cast<std::size_t>(channels);
    if (data_size != expected_size) {
        std::cerr << "Failed to convert ImageFrame: data size mismatch for format '"
                  << format << "': expected " << expected_size
                  << ", actual " << data_size << std::endl;
        return false;
    }
    return true;
}

bool FillCameraInfoCommon(
    const std::string& frame_id,
    int width,
    int height,
    double horizontal_fov,
    double timestamp,
    HakoCpp_CameraInfo& out)
{
    if (width <= 0 || height <= 0) {
        std::cerr << "Failed to convert CameraInfo: invalid image size "
                  << width << "x" << height << std::endl;
        return false;
    }
    if (horizontal_fov <= 0.0 || horizontal_fov > kPi) {
        std::cerr << "Failed to convert CameraInfo: invalid horizontal_fov "
                  << horizontal_fov << std::endl;
        return false;
    }

    FillHeader(frame_id, timestamp, out.header);
    out.width = static_cast<Hako_uint32>(width);
    out.height = static_cast<Hako_uint32>(height);
    out.distortion_model = "plumb_bob";
    out.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double half_horizontal = horizontal_fov * 0.5;
    const double vertical_fov =
        2.0 * std::atan(std::tan(half_horizontal) * static_cast<double>(height) / static_cast<double>(width));
    const double fx = static_cast<double>(width) / (2.0 * std::tan(half_horizontal));
    const double fy = static_cast<double>(height) / (2.0 * std::tan(vertical_fov * 0.5));
    const double cx = static_cast<double>(width) * 0.5;
    const double cy = static_cast<double>(height) * 0.5;

    out.k = {fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0};
    out.r = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};
    out.p = {fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0};
    out.binning_x = 0;
    out.binning_y = 0;
    out.roi.x_offset = 0;
    out.roi.y_offset = 0;
    out.roi.height = 0;
    out.roi.width = 0;
    out.roi.do_rectify = 0;
    return true;
}
}

bool ConvertImageFrameToSensorMsgsImage(
    const ImageFrame& frame,
    HakoCpp_Image& out)
{
    std::string encoding;
    Hako_uint32 step = 0;
    int channels = 0;

    if (frame.format == "R8G8B8") {
        encoding = "rgb8";
        step = static_cast<Hako_uint32>(frame.width * 3);
        channels = 3;
    } else if (frame.format == "B8G8R8") {
        encoding = "bgr8";
        step = static_cast<Hako_uint32>(frame.width * 3);
        channels = 3;
    } else if (frame.format == "L8") {
        encoding = "mono8";
        step = static_cast<Hako_uint32>(frame.width);
        channels = 1;
    } else {
        std::cerr << "Failed to convert ImageFrame: unsupported format '"
                  << frame.format << "'" << std::endl;
        return false;
    }

    if (!ValidateImageFrameCommon(frame.format, frame.width, frame.height, frame.data.size(), channels)) {
        return false;
    }

    FillHeader(frame.frame_id, frame.timestamp, out.header);
    out.height = static_cast<Hako_uint32>(frame.height);
    out.width = static_cast<Hako_uint32>(frame.width);
    out.encoding = std::move(encoding);
    out.is_bigendian = 0;
    out.step = step;
    out.data = frame.data;
    return true;
}

bool ConvertDepthFrameToSensorMsgsImage(
    const DepthFrame& frame,
    HakoCpp_Image& out)
{
    if (frame.width <= 0 || frame.height <= 0) {
        std::cerr << "Failed to convert DepthFrame: invalid image size "
                  << frame.width << "x" << frame.height << std::endl;
        return false;
    }
    const std::size_t expected_size =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.data.size() != expected_size) {
        std::cerr << "Failed to convert DepthFrame: data size mismatch for format '"
                  << frame.format << "': expected " << expected_size
                  << ", actual " << frame.data.size() << std::endl;
        return false;
    }

    FillHeader(frame.frame_id, frame.timestamp, out.header);
    out.height = static_cast<Hako_uint32>(frame.height);
    out.width = static_cast<Hako_uint32>(frame.width);
    out.is_bigendian = 0;

    if (frame.format == "DEPTH_F32_M") {
        out.encoding = "32FC1";
        out.step = static_cast<Hako_uint32>(frame.width * static_cast<int>(sizeof(float)));
        out.data.resize(expected_size * sizeof(float));
        std::memcpy(out.data.data(), frame.data.data(), out.data.size());
        return true;
    }
    if (frame.format == "DEPTH_U16_MM") {
        out.encoding = "16UC1";
        out.step = static_cast<Hako_uint32>(frame.width * static_cast<int>(sizeof(std::uint16_t)));

        std::vector<std::uint16_t> mm_values(expected_size, 0);
        for (std::size_t i = 0; i < frame.data.size(); ++i) {
            const float depth_m = frame.data[i];
            // 0 is used as invalid / no-return when the source value is not finite,
            // non-positive, or cannot be represented in uint16 millimeters.
            if (!std::isfinite(depth_m) || depth_m <= 0.0F) {
                continue;
            }
            const double mm_double = std::round(static_cast<double>(depth_m) * 1000.0);
            if (mm_double <= 0.0 ||
                mm_double > static_cast<double>(std::numeric_limits<std::uint16_t>::max()))
            {
                continue;
            }
            mm_values[i] = static_cast<std::uint16_t>(mm_double);
        }
        out.data.resize(expected_size * sizeof(std::uint16_t));
        std::memcpy(out.data.data(), mm_values.data(), out.data.size());
        return true;
    }

    std::cerr << "Failed to convert DepthFrame: unsupported format '"
              << frame.format << "'" << std::endl;
    return false;
}

bool ConvertCameraConfigToCameraInfo(
    const CameraConfig& config,
    double timestamp,
    HakoCpp_CameraInfo& out)
{
    return FillCameraInfoCommon(
        config.frame_id,
        config.image.width,
        config.image.height,
        config.horizontal_fov,
        timestamp,
        out);
}

bool ConvertDepthCameraConfigToCameraInfo(
    const DepthCameraConfig& config,
    double timestamp,
    HakoCpp_CameraInfo& out)
{
    return FillCameraInfoCommon(
        config.frame_id,
        config.image.width,
        config.image.height,
        config.horizontal_fov,
        timestamp,
        out);
}
}
