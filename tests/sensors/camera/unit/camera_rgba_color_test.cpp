#include "hakoniwa/pdu/converter/std_msgs/color_rgba.hpp"
#include "tests/sensors/camera/support/camera_test_utils.hpp"

#include <iostream>
#include <string>

namespace
{
using hako::robots::sensor::camera::CameraConfig;
using hako::robots::sensor::camera::ICameraSensor;
using hako::robots::sensor::camera::ImageFrame;
using hako::robots::sensor::camera::RGBAColor;
using hako::robots::sensor::camera::TryExtractAverageRGBAColor;
using hako::robots::sensor::camera::TryExtractRGBAColor;
using hako::robots::sensor::camera::test::MakeImageFrame;
using hako::robots::sensor::camera::test::NearlyEqual;

class FakeCameraSensor final : public ICameraSensor
{
public:
    explicit FakeCameraSensor(ImageFrame frame)
        : frame_(std::move(frame))
    {
    }

    bool LoadConfig(const CameraConfig& config) override
    {
        config_ = config;
        return true;
    }

    const CameraConfig& GetConfig() const override
    {
        return config_;
    }

    void Capture(ImageFrame& out) override
    {
        out = frame_;
    }

private:
    ImageFrame frame_;
    CameraConfig config_;
};

void ExpectColor(
    const RGBAColor& color,
    float r,
    float g,
    float b,
    float a,
    const std::string& label)
{
    HAKO_TEST_EXPECT(NearlyEqual(color.r, r), label + ": unexpected r");
    HAKO_TEST_EXPECT(NearlyEqual(color.g, g), label + ": unexpected g");
    HAKO_TEST_EXPECT(NearlyEqual(color.b, b), label + ": unexpected b");
    HAKO_TEST_EXPECT(NearlyEqual(color.a, a), label + ": unexpected a");
}

void TestExtractCenterRgb()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        2,
        "R8G8B8",
        "camera_rgb_frame",
        0.0,
        {
            0, 0, 0,
            255, 0, 0,
            0, 255, 0,
            0, 0, 255
        });

    RGBAColor color {};
    HAKO_TEST_EXPECT(TryExtractRGBAColor(frame, color), "center RGB extraction should succeed");
    ExpectColor(color, 0.0F, 0.0F, 1.0F, 1.0F, "center RGB");
}

void TestExtractExplicitBgr()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        1,
        "B8G8R8",
        "camera_bgr_frame",
        0.0,
        {
            30, 20, 10,
            60, 50, 40
        });

    RGBAColor color {};
    HAKO_TEST_EXPECT(TryExtractRGBAColor(frame, color, 0, 0), "BGR extraction should succeed");
    ExpectColor(color, 10.0F / 255.0F, 20.0F / 255.0F, 30.0F / 255.0F, 1.0F, "BGR");
}

void TestExtractMono()
{
    const ImageFrame frame = MakeImageFrame(
        1,
        1,
        "L8",
        "camera_mono_frame",
        0.0,
        {128});

    RGBAColor color {};
    HAKO_TEST_EXPECT(TryExtractRGBAColor(frame, color), "mono extraction should succeed");
    ExpectColor(color, 128.0F / 255.0F, 128.0F / 255.0F, 128.0F / 255.0F, 1.0F, "mono");
}

void TestCaptureAsRGBA()
{
    const ImageFrame frame = MakeImageFrame(
        1,
        1,
        "R8G8B8",
        "camera_rgb_frame",
        0.0,
        {64, 128, 255});

    FakeCameraSensor sensor(frame);
    const RGBAColor color = sensor.CaptureAsRGBA();
    ExpectColor(color, 64.0F / 255.0F, 128.0F / 255.0F, 1.0F, 1.0F, "CaptureAsRGBA");
}

void TestCaptureRegionAverageRGBA()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        2,
        "R8G8B8",
        "camera_rgb_frame",
        0.0,
        {
            0, 0, 0,
            255, 0, 0,
            0, 255, 0,
            0, 0, 255
        });

    FakeCameraSensor sensor(frame);
    const RGBAColor color = sensor.CaptureRegionAverageRGBA(0, 0, 2, 2);
    ExpectColor(color, 63.75F / 255.0F, 63.75F / 255.0F, 63.75F / 255.0F, 1.0F, "CaptureRegionAverageRGBA");
}

void TestExtractRegionAverageClampsToImage()
{
    const ImageFrame frame = MakeImageFrame(
        2,
        1,
        "R8G8B8",
        "camera_rgb_frame",
        0.0,
        {
            100, 50, 0,
            200, 150, 100
        });

    RGBAColor color {};
    HAKO_TEST_EXPECT(
        TryExtractAverageRGBAColor(frame, color, -10, 0, 20, 1),
        "clamped region average should succeed");
    ExpectColor(color, 150.0F / 255.0F, 100.0F / 255.0F, 50.0F / 255.0F, 1.0F, "clamped average");
}

void TestInvalidFrame()
{
    const ImageFrame frame = MakeImageFrame(1, 1, "R8G8B8", "bad", 0.0, {1, 2});
    RGBAColor color {1.0F, 1.0F, 1.0F, 1.0F};
    HAKO_TEST_EXPECT(!TryExtractRGBAColor(frame, color), "invalid frame should fail");
    ExpectColor(color, 0.0F, 0.0F, 0.0F, 0.0F, "invalid frame");
}

void TestColorRGBAPduConversion()
{
    const RGBAColor color {0.1F, 0.2F, 0.3F, 0.4F};
    const HakoCpp_ColorRGBA pdu = hako::robots::pdu::converter::std_msgs::ToHakoPdu(color);
    HAKO_TEST_EXPECT(NearlyEqual(pdu.r, color.r), "unexpected PDU r");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.g, color.g), "unexpected PDU g");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.b, color.b), "unexpected PDU b");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.a, color.a), "unexpected PDU a");
}
}

int main()
{
    TestExtractCenterRgb();
    TestExtractExplicitBgr();
    TestExtractMono();
    TestCaptureAsRGBA();
    TestCaptureRegionAverageRGBA();
    TestExtractRegionAverageClampsToImage();
    TestInvalidFrame();
    TestColorRGBAPduConversion();

    std::cout << "camera_rgba_color_test passed" << std::endl;
    return 0;
}
