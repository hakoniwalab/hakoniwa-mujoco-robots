#include "sensors/camera/camera_pdu_converter.hpp"
#include "tests/sensors/camera/support/camera_test_utils.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

namespace
{
using hako::robots::sensor::camera::CameraConfig;
using hako::robots::sensor::camera::DepthCameraConfig;
using hako::robots::sensor::camera::DepthFrame;
using hako::robots::sensor::camera::ImageFrame;
using hako::robots::sensor::camera::test::MakeDepthFrame;
using hako::robots::sensor::camera::test::MakeImageFrame;
using hako::robots::sensor::camera::test::NearlyEqual;

void TestRgbImageConversion()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        1,
        "R8G8B8",
        "camera_rgb_frame",
        12.345678,
        {1, 2, 3, 4, 5, 6});

    HakoCpp_Image out {};
    const bool ok = hako::robots::sensor::camera::ConvertImageFrameToSensorMsgsImage(frame, out);
    HAKO_TEST_EXPECT(ok, "RGB image conversion should succeed");
    HAKO_TEST_EXPECT(out.encoding == "rgb8", "unexpected RGB encoding");
    HAKO_TEST_EXPECT(out.step == 6, "unexpected RGB step");
    HAKO_TEST_EXPECT(out.width == 2, "unexpected RGB width");
    HAKO_TEST_EXPECT(out.height == 1, "unexpected RGB height");
    HAKO_TEST_EXPECT(out.is_bigendian == 0, "unexpected RGB endian");
    HAKO_TEST_EXPECT(out.header.frame_id == frame.frame_id, "unexpected RGB frame_id");
    HAKO_TEST_EXPECT_TIME(out.header.stamp, 12, 345678000);
    HAKO_TEST_EXPECT(out.data == frame.data, "unexpected RGB payload");
}

void TestBgrImageConversion()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        1,
        "B8G8R8",
        "camera_bgr_frame",
        0.0,
        {6, 5, 4, 3, 2, 1});

    HakoCpp_Image out {};
    const bool ok = hako::robots::sensor::camera::ConvertImageFrameToSensorMsgsImage(frame, out);
    HAKO_TEST_EXPECT(ok, "BGR image conversion should succeed");
    HAKO_TEST_EXPECT(out.encoding == "bgr8", "unexpected BGR encoding");
    HAKO_TEST_EXPECT(out.step == 6, "unexpected BGR step");
    HAKO_TEST_EXPECT(out.data == frame.data, "unexpected BGR payload");
}

void TestMonoImageConversion()
{
    const ImageFrame frame = MakeImageFrame(
        3,
        2,
        "L8",
        "camera_mono_frame",
        0.0,
        {0, 1, 2, 3, 4, 5});

    HakoCpp_Image out {};
    const bool ok = hako::robots::sensor::camera::ConvertImageFrameToSensorMsgsImage(frame, out);
    HAKO_TEST_EXPECT(ok, "mono image conversion should succeed");
    HAKO_TEST_EXPECT(out.encoding == "mono8", "unexpected mono encoding");
    HAKO_TEST_EXPECT(out.step == 3, "unexpected mono step");
    HAKO_TEST_EXPECT(out.data == frame.data, "unexpected mono payload");
}

void TestDepthF32Conversion()
{
    const DepthFrame frame = MakeDepthFrame(
        3,
        1,
        "DEPTH_F32_M",
        "camera_depth_frame",
        1.25,
        {1.25F, std::numeric_limits<float>::quiet_NaN(), 2.5F});

    HakoCpp_Image out {};
    const bool ok = hako::robots::sensor::camera::ConvertDepthFrameToSensorMsgsImage(frame, out);
    HAKO_TEST_EXPECT(ok, "DEPTH_F32_M conversion should succeed");
    HAKO_TEST_EXPECT(out.encoding == "32FC1", "unexpected DEPTH_F32_M encoding");
    HAKO_TEST_EXPECT(out.step == 3 * sizeof(float), "unexpected DEPTH_F32_M step");
    HAKO_TEST_EXPECT(out.width == 3, "unexpected DEPTH_F32_M width");
    HAKO_TEST_EXPECT(out.height == 1, "unexpected DEPTH_F32_M height");
    HAKO_TEST_EXPECT_TIME(out.header.stamp, 1, 250000000);
    HAKO_TEST_EXPECT(out.data.size() == frame.data.size() * sizeof(float), "unexpected DEPTH_F32_M byte size");

    std::vector<float> roundtrip(frame.data.size(), 0.0F);
    std::memcpy(roundtrip.data(), out.data.data(), out.data.size());
    HAKO_TEST_EXPECT(NearlyEqual(roundtrip[0], 1.25F), "unexpected first DEPTH_F32_M sample");
    HAKO_TEST_EXPECT(std::isnan(roundtrip[1]), "NaN should be preserved in DEPTH_F32_M");
    HAKO_TEST_EXPECT(NearlyEqual(roundtrip[2], 2.5F), "unexpected third DEPTH_F32_M sample");
}

void TestDepthU16Conversion()
{
    const DepthFrame frame = MakeDepthFrame(
        4,
        1,
        "DEPTH_U16_MM",
        "camera_depth_frame",
        0.0,
        {1.234F, std::numeric_limits<float>::quiet_NaN(), -1.0F, 1000.0F});

    HakoCpp_Image out {};
    const bool ok = hako::robots::sensor::camera::ConvertDepthFrameToSensorMsgsImage(frame, out);
    HAKO_TEST_EXPECT(ok, "DEPTH_U16_MM conversion should succeed");
    HAKO_TEST_EXPECT(out.encoding == "16UC1", "unexpected DEPTH_U16_MM encoding");
    HAKO_TEST_EXPECT(out.step == 4 * sizeof(std::uint16_t), "unexpected DEPTH_U16_MM step");
    HAKO_TEST_EXPECT(out.data.size() == frame.data.size() * sizeof(std::uint16_t), "unexpected DEPTH_U16_MM byte size");

    std::vector<std::uint16_t> roundtrip(frame.data.size(), 0);
    std::memcpy(roundtrip.data(), out.data.data(), out.data.size());
    HAKO_TEST_EXPECT(roundtrip[0] == 1234, "1.234m should become 1234mm");
    HAKO_TEST_EXPECT(roundtrip[1] == 0, "NaN should become 0");
    HAKO_TEST_EXPECT(roundtrip[2] == 0, "negative depth should become 0");
    HAKO_TEST_EXPECT(roundtrip[3] == 0, "out-of-range depth should become 0");
}

void TestCameraInfoConversion()
{
    CameraConfig config {};
    config.frame_id = "camera_rgb_frame";
    config.image.width = 640;
    config.image.height = 480;
    config.horizontal_fov = 1.0;

    HakoCpp_CameraInfo out {};
    const bool ok = hako::robots::sensor::camera::ConvertCameraConfigToCameraInfo(config, 12.345678, out);
    HAKO_TEST_EXPECT(ok, "CameraInfo conversion should succeed");

    const double vertical_fov = 2.0 * std::atan(std::tan(0.5) * 480.0 / 640.0);
    const double fx = 640.0 / (2.0 * std::tan(0.5));
    const double fy = 480.0 / (2.0 * std::tan(vertical_fov * 0.5));
    const double cx = 320.0;
    const double cy = 240.0;

    HAKO_TEST_EXPECT(out.header.frame_id == "camera_rgb_frame", "unexpected CameraInfo frame_id");
    HAKO_TEST_EXPECT_TIME(out.header.stamp, 12, 345678000);
    HAKO_TEST_EXPECT(out.width == 640, "unexpected CameraInfo width");
    HAKO_TEST_EXPECT(out.height == 480, "unexpected CameraInfo height");
    HAKO_TEST_EXPECT(out.distortion_model == "plumb_bob", "unexpected distortion model");
    HAKO_TEST_EXPECT(out.d.size() == 5, "unexpected distortion coefficient count");
    HAKO_TEST_EXPECT(NearlyEqual(out.k[0], fx), "unexpected fx in K");
    HAKO_TEST_EXPECT(NearlyEqual(out.k[2], cx), "unexpected cx in K");
    HAKO_TEST_EXPECT(NearlyEqual(out.k[4], fy), "unexpected fy in K");
    HAKO_TEST_EXPECT(NearlyEqual(out.k[5], cy), "unexpected cy in K");
    HAKO_TEST_EXPECT(NearlyEqual(out.k[8], 1.0), "unexpected K[8]");
    HAKO_TEST_EXPECT(NearlyEqual(out.r[0], 1.0), "unexpected R[0]");
    HAKO_TEST_EXPECT(NearlyEqual(out.r[4], 1.0), "unexpected R[4]");
    HAKO_TEST_EXPECT(NearlyEqual(out.r[8], 1.0), "unexpected R[8]");
    HAKO_TEST_EXPECT(NearlyEqual(out.p[0], fx), "unexpected P[0]");
    HAKO_TEST_EXPECT(NearlyEqual(out.p[2], cx), "unexpected P[2]");
    HAKO_TEST_EXPECT(NearlyEqual(out.p[5], fy), "unexpected P[5]");
    HAKO_TEST_EXPECT(NearlyEqual(out.p[6], cy), "unexpected P[6]");
    HAKO_TEST_EXPECT(NearlyEqual(out.p[10], 1.0), "unexpected P[10]");
}

void TestDepthCameraInfoConversion()
{
    DepthCameraConfig config {};
    config.frame_id = "camera_depth_frame";
    config.image.width = 320;
    config.image.height = 240;
    config.horizontal_fov = 1.2;

    HakoCpp_CameraInfo out {};
    const bool ok = hako::robots::sensor::camera::ConvertDepthCameraConfigToCameraInfo(config, 1.5, out);
    HAKO_TEST_EXPECT(ok, "Depth CameraInfo conversion should succeed");
    HAKO_TEST_EXPECT(out.header.frame_id == "camera_depth_frame", "unexpected depth CameraInfo frame_id");
    HAKO_TEST_EXPECT_TIME(out.header.stamp, 1, 500000000);
    HAKO_TEST_EXPECT(out.width == 320, "unexpected depth CameraInfo width");
    HAKO_TEST_EXPECT(out.height == 240, "unexpected depth CameraInfo height");
}

void TestInvalidInputFailures()
{
    const ImageFrame bad_image = MakeImageFrame(1, 1, "R8G8B8", "bad", 0.0, {1, 2});
    HakoCpp_Image out_image {};
    HAKO_TEST_EXPECT(
        !hako::robots::sensor::camera::ConvertImageFrameToSensorMsgsImage(bad_image, out_image),
        "invalid RGB image should fail");

    const DepthFrame bad_depth = MakeDepthFrame(2, 2, "DEPTH_F32_M", "bad", 0.0, {1.0F});
    HAKO_TEST_EXPECT(
        !hako::robots::sensor::camera::ConvertDepthFrameToSensorMsgsImage(bad_depth, out_image),
        "invalid depth image should fail");
}
}

int main()
{
    TestRgbImageConversion();
    TestBgrImageConversion();
    TestMonoImageConversion();
    TestDepthF32Conversion();
    TestDepthU16Conversion();
    TestCameraInfoConversion();
    TestDepthCameraInfoConversion();
    TestInvalidInputFailures();

    std::cout << "camera_pdu_converter_test passed" << std::endl;
    return 0;
}
