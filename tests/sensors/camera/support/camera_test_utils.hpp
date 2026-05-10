#pragma once

#include "tests/sensors/support/sensor_test_utils.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera::test
{
    using ::hako::robots::sensor::test::FailExpectation;
    using ::hako::robots::sensor::test::NearlyEqual;
    using ::hako::robots::sensor::test::RepoRoot;
    using ::hako::robots::sensor::test::ExpectHakoTime;

    inline hako::robots::sensor::camera::ImageFrame MakeImageFrame(
        int width,
        int height,
        const std::string& format,
        const std::string& frame_id,
        double timestamp,
        std::vector<std::uint8_t> data)
    {
        hako::robots::sensor::camera::ImageFrame frame {};
        frame.width = width;
        frame.height = height;
        frame.format = format;
        frame.frame_id = frame_id;
        frame.timestamp = timestamp;
        frame.data = std::move(data);
        if (format == "R8G8B8" || format == "B8G8R8") {
            frame.channels = 3;
        } else if (format == "L8") {
            frame.channels = 1;
        }
        return frame;
    }

    inline hako::robots::sensor::camera::DepthFrame MakeDepthFrame(
        int width,
        int height,
        const std::string& format,
        const std::string& frame_id,
        double timestamp,
        std::vector<float> data)
    {
        hako::robots::sensor::camera::DepthFrame frame {};
        frame.width = width;
        frame.height = height;
        frame.format = format;
        frame.frame_id = frame_id;
        frame.timestamp = timestamp;
        frame.data = std::move(data);
        return frame;
    }
}
