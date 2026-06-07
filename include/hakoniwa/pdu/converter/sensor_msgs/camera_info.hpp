#pragma once

#include <cmath>
#include <iostream>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_CameraInfo.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    namespace detail
    {
        inline bool FillCameraInfoCommon(
            const std::string& frame_id,
            int width,
            int height,
            double horizontal_fov,
            double timestamp,
            HakoCpp_CameraInfo& out)
        {
            constexpr double kPi = 3.14159265358979323846;
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

            out.header.stamp = hako::robots::pdu::converter::ToHakoTime(timestamp);
            out.header.frame_id = frame_id;
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

    inline bool ToHakoPdu(
        const hako::robots::sensor::camera::CameraConfig& config,
        double timestamp,
        HakoCpp_CameraInfo& out)
    {
        return detail::FillCameraInfoCommon(
            config.frame_id,
            config.image.width,
            config.image.height,
            config.horizontal_fov,
            timestamp,
            out);
    }

    inline bool ToHakoPdu(
        const hako::robots::sensor::camera::DepthCameraConfig& config,
        double timestamp,
        HakoCpp_CameraInfo& out)
    {
        return detail::FillCameraInfoCommon(
            config.frame_id,
            config.image.width,
            config.image.height,
            config.horizontal_fov,
            timestamp,
            out);
    }
}
