#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "tests/sensors/camera/support/camera_test_utils.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

namespace
{
using hako::robots::sensor::camera::test::NearlyEqual;

void RunDepthEncodingTest()
{
    // This test only exercises the local depth conversion path with mock samples.
    // It does not validate real MuJoCo rendered distances against known scene geometry.
    hako::robots::sensor::camera::RawCameraFrame raw;
    raw.width = 3;
    raw.height = 1;
    raw.depth_buffer = {0.0F, 0.5F, 1.0F};
    raw.znear = 0.1;
    raw.zfar = 1000.0;

    hako::robots::sensor::camera::DepthCameraConfig config;
    config.clip.near = 0.19;
    config.clip.far = 500.0;

    hako::robots::sensor::camera::DepthFrame out_f32;
    config.image.format = "DEPTH_F32_M";
    bool success = hako::robots::sensor::camera::EncodeDepth(raw, config, out_f32);
    HAKO_TEST_EXPECT(success, "EncodeDepth DEPTH_F32_M should succeed");
    HAKO_TEST_EXPECT(out_f32.width == raw.width, "unexpected output width");
    HAKO_TEST_EXPECT(out_f32.height == raw.height, "unexpected output height");
    HAKO_TEST_EXPECT(out_f32.data.size() == static_cast<size_t>(raw.width * raw.height), "unexpected output sample count");
    HAKO_TEST_EXPECT(std::isnan(out_f32.data[0]), "near-clipped sample should be NaN");
    HAKO_TEST_EXPECT(NearlyEqual(out_f32.data[1], 0.2F, 0.001F), "middle sample should decode to about 0.2m");
    HAKO_TEST_EXPECT(std::isnan(out_f32.data[2]), "far-clipped sample should be NaN");

    hako::robots::sensor::camera::DepthFrame out_u16_hint;
    config.image.format = "DEPTH_U16_MM";
    success = hako::robots::sensor::camera::EncodeDepth(raw, config, out_u16_hint);
    HAKO_TEST_EXPECT(success, "EncodeDepth DEPTH_U16_MM should succeed");
    HAKO_TEST_EXPECT(out_u16_hint.data.size() == static_cast<size_t>(raw.width * raw.height), "unexpected DEPTH_U16_MM sample count");
    HAKO_TEST_EXPECT(out_u16_hint.format == "DEPTH_U16_MM", "unexpected DEPTH_U16_MM format tag");
}
}

int main()
{
    RunDepthEncodingTest();
    std::cout << "depth_encoding_test passed" << std::endl;
    return 0;
}
