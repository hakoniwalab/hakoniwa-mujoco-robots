#pragma once

#include "sensor_msgs/pdu_cpptype_CameraInfo.hpp"
#include "sensor_msgs/pdu_cpptype_Image.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera
{
    bool ConvertImageFrameToSensorMsgsImage(
        const ImageFrame& frame,
        HakoCpp_Image& out);

    bool ConvertDepthFrameToSensorMsgsImage(
        const DepthFrame& frame,
        HakoCpp_Image& out);

    bool ConvertCameraConfigToCameraInfo(
        const CameraConfig& config,
        double timestamp,
        HakoCpp_CameraInfo& out);

    bool ConvertDepthCameraConfigToCameraInfo(
        const DepthCameraConfig& config,
        double timestamp,
        HakoCpp_CameraInfo& out);
}
