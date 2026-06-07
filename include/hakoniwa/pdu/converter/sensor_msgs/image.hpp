#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_Image.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    namespace detail
    {
        inline bool ValidateImageFrameCommon(
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
                static_cast<std::size_t>(width) *
                static_cast<std::size_t>(height) *
                static_cast<std::size_t>(channels);
            if (data_size != expected_size) {
                std::cerr << "Failed to convert ImageFrame: data size mismatch for format '"
                          << format << "': expected " << expected_size
                          << ", actual " << data_size << std::endl;
                return false;
            }
            return true;
        }
    }

    inline bool ToHakoPdu(
        const hako::robots::sensor::camera::ImageFrame& frame,
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

        if (!detail::ValidateImageFrameCommon(
                frame.format,
                frame.width,
                frame.height,
                frame.data.size(),
                channels))
        {
            return false;
        }

        out.header.stamp = hako::robots::pdu::converter::ToHakoTime(frame.timestamp);
        out.header.frame_id = frame.frame_id;
        out.height = static_cast<Hako_uint32>(frame.height);
        out.width = static_cast<Hako_uint32>(frame.width);
        out.encoding = std::move(encoding);
        out.is_bigendian = 0;
        out.step = step;
        out.data = frame.data;
        return true;
    }

    inline bool ToHakoPdu(
        const hako::robots::sensor::camera::DepthFrame& frame,
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

        out.header.stamp = hako::robots::pdu::converter::ToHakoTime(frame.timestamp);
        out.header.frame_id = frame.frame_id;
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
}
