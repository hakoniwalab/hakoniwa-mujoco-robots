#include "sensors/camera/camera_config_loader.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace
{
std::filesystem::path RepoRoot()
{
    return std::filesystem::current_path();
}

bool NearlyEqual(double lhs, double rhs, double epsilon = 1.0e-9)
{
    return std::abs(lhs - rhs) <= epsilon;
}

void TestCameraConfigLoader()
{
    hako::robots::sensor::camera::CameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path, config);
    assert(ok);
    assert(config.frame_id == "camera_rgb_frame");
    assert(NearlyEqual(config.update_rate, 30.0));
    assert(NearlyEqual(config.horizontal_fov, 1.39626));
    assert(config.image.width == 1280);
    assert(config.image.height == 720);
    assert(config.image.format == "R8G8B8");
    assert(NearlyEqual(config.clip.near, 0.02));
    assert(NearlyEqual(config.clip.far, 300.0));
    assert(config.noise.type == "gaussian");
    assert(NearlyEqual(config.noise.mean, 0.0));
    assert(NearlyEqual(config.noise.stddev, 0.007));
}

void TestDepthCameraConfigLoader()
{
    hako::robots::sensor::camera::DepthCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_depth_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadDepthCameraConfigFromJson(path, config);
    assert(ok);
    assert(config.frame_id == "camera_depth_frame");
    assert(NearlyEqual(config.update_rate, 30.0));
    assert(NearlyEqual(config.horizontal_fov, 1.047));
    assert(config.image.width == 640);
    assert(config.image.height == 480);
    assert(config.image.format == "DEPTH_F32_M");
    assert(NearlyEqual(config.clip.near, 0.1));
    assert(NearlyEqual(config.clip.far, 10.0));
    assert(config.noise.type == "gaussian");
    assert(NearlyEqual(config.noise.stddev, 0.01));
}

void TestRgbdCameraConfigLoader()
{
    hako::robots::sensor::camera::RgbdCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_rgbd_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadRgbdCameraConfigFromJson(path, config);
    assert(ok);
    assert(config.rgb.frame_id == "camera_rgb_frame");
    assert(config.depth.frame_id == "camera_depth_frame");
    assert(config.rgb.image.width == 640);
    assert(config.depth.image.width == 640);
    assert(config.rgb.image.height == 480);
    assert(config.depth.image.height == 480);
    assert(NearlyEqual(config.rgb.horizontal_fov, 1.047));
    assert(NearlyEqual(config.depth.horizontal_fov, 1.047));
}

void TestStereoCameraConfigLoader()
{
    hako::robots::sensor::camera::StereoCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_multicamera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadStereoCameraConfigFromJson(path, config);
    assert(ok);
    assert(config.left.frame_id == "left_camera_frame");
    assert(config.right.frame_id == "right_camera_frame");
    assert(config.left.image.width == 1280);
    assert(config.right.image.width == 1280);
    assert(NearlyEqual(config.left.horizontal_fov, 1.39626));
    assert(NearlyEqual(config.right.horizontal_fov, 1.39626));
    assert(NearlyEqual(config.baseline, 0.12, 1.0e-9));
}

void TestMissingFileFailure()
{
    hako::robots::sensor::camera::CameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/does_not_exist.json").string();
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path, config);
    assert(!ok);
}

void TestInvalidJsonFailure()
{
    const auto path = RepoRoot() / "tests/sensors/camera/invalid_camera_config.json";
    std::ofstream ofs(path);
    ofs << "{ invalid json }";
    ofs.close();

    hako::robots::sensor::camera::CameraConfig config {};
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path.string(), config);
    assert(!ok);

    std::filesystem::remove(path);
}
}

int main()
{
    TestCameraConfigLoader();
    TestDepthCameraConfigLoader();
    TestRgbdCameraConfigLoader();
    TestStereoCameraConfigLoader();
    TestMissingFileFailure();
    TestInvalidJsonFailure();

    std::cout << "camera_config_loader_unit_test passed" << std::endl;
    return 0;
}
