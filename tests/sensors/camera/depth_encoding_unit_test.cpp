#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

#include <cassert>
#include <cstddef>
#include <cmath>
#include <iostream>

namespace
{
void run_depth_encoding_test()
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
    assert(success);
    assert(out_f32.width == raw.width);
    assert(out_f32.height == raw.height);
    assert(out_f32.data.size() == static_cast<size_t>(raw.width * raw.height));
    assert(std::isnan(out_f32.data[0]));
    assert(out_f32.data[1] > 0.199F && out_f32.data[1] < 0.201F);
    assert(std::isnan(out_f32.data[2]));

    hako::robots::sensor::camera::DepthFrame out_u16_hint;
    config.image.format = "DEPTH_U16_MM";
    success = hako::robots::sensor::camera::EncodeDepth(raw, config, out_u16_hint);
    assert(success);
    assert(out_u16_hint.data.size() == static_cast<size_t>(raw.width * raw.height));
    assert(out_u16_hint.format == "DEPTH_U16_MM");
}
}

int main()
{
    run_depth_encoding_test();
    std::cout << "depth_encoding_unit_test passed" << std::endl;
    return 0;
}
