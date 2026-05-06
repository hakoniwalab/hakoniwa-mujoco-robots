#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/pdu_cpptype_Time.hpp"
#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera::test
{
    [[noreturn]] inline void FailExpectation(const char* file, int line, const std::string& message)
    {
        std::cerr << file << ":" << line << ": " << message << std::endl;
        std::abort();
    }

    inline bool NearlyEqual(double lhs, double rhs, double epsilon = 1.0e-6)
    {
        return std::abs(lhs - rhs) <= epsilon;
    }

    inline bool NearlyEqual(float lhs, float rhs, float epsilon = 1.0e-6F)
    {
        return std::abs(lhs - rhs) <= epsilon;
    }

    inline std::filesystem::path RepoRoot()
    {
        return std::filesystem::current_path();
    }

    inline hako::robots::sensor::camera::ImageFrame MakeImageFrame(
        int width,
        int height,
        const std::string& format,
        const std::string& frame_id,
        double timestamp,
        std::vector<uint8_t> data)
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

    inline void ExpectHakoTime(
        const HakoCpp_Time& actual,
        int expected_sec,
        uint32_t expected_nanosec,
        const char* file,
        int line)
    {
        if (actual.sec != expected_sec || actual.nanosec != expected_nanosec) {
            std::ostringstream oss;
            oss << "unexpected HakoCpp_Time: actual=(" << actual.sec << ", " << actual.nanosec
                << "), expected=(" << expected_sec << ", " << expected_nanosec << ")";
            FailExpectation(file, line, oss.str());
        }
    }
}

#define HAKO_TEST_EXPECT(cond, msg) \
    do { \
        if (!(cond)) { \
            ::hako::robots::sensor::camera::test::FailExpectation(__FILE__, __LINE__, (msg)); \
        } \
    } while (false)

#define HAKO_TEST_EXPECT_TIME(actual, sec, nanosec) \
    do { \
        ::hako::robots::sensor::camera::test::ExpectHakoTime((actual), (sec), (nanosec), __FILE__, __LINE__); \
    } while (false)
