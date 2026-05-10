#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/pdu_cpptype_Time.hpp"

namespace hako::robots::sensor::test
{
    inline void FailExpectation(const char* file, int line, const std::string& message)
    {
        std::ostringstream oss;
        oss << file << ":" << line << ": " << message;
        throw std::runtime_error(oss.str());
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

    inline void ExpectHakoTime(
        const HakoCpp_Time& actual,
        int expected_sec,
        std::uint32_t expected_nanosec,
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
            ::hako::robots::sensor::test::FailExpectation(__FILE__, __LINE__, (msg)); \
        } \
    } while (false)

#define HAKO_TEST_EXPECT_TIME(actual, sec, nanosec) \
    do { \
        ::hako::robots::sensor::test::ExpectHakoTime((actual), (sec), (nanosec), __FILE__, __LINE__); \
    } while (false)
