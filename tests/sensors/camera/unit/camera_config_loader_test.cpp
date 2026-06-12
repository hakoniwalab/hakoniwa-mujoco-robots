#include "sensors/camera/camera_config_loader.hpp"
#include "tests/sensors/camera/support/camera_test_utils.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace
{
using hako::robots::sensor::camera::test::NearlyEqual;
using hako::robots::sensor::camera::test::RepoRoot;

void TestCameraConfigLoader()
{
    hako::robots::sensor::camera::CameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path, config);
    HAKO_TEST_EXPECT(ok, "sample_camera.json should load");
    HAKO_TEST_EXPECT(config.frame_id == "camera_rgb_frame", "unexpected frame_id");
    HAKO_TEST_EXPECT(NearlyEqual(config.update_rate, 30.0), "unexpected update_rate");
    HAKO_TEST_EXPECT(NearlyEqual(config.horizontal_fov, 1.39626), "unexpected horizontal_fov");
    HAKO_TEST_EXPECT(config.image.width == 1280, "unexpected image.width");
    HAKO_TEST_EXPECT(config.image.height == 720, "unexpected image.height");
    HAKO_TEST_EXPECT(config.image.format == "R8G8B8", "unexpected image.format");
    HAKO_TEST_EXPECT(NearlyEqual(config.clip.near, 0.02), "unexpected clip.near");
    HAKO_TEST_EXPECT(NearlyEqual(config.clip.far, 300.0), "unexpected clip.far");
    HAKO_TEST_EXPECT(config.noise.type == "gaussian", "unexpected noise.type");
    HAKO_TEST_EXPECT(NearlyEqual(config.noise.mean, 0.0), "unexpected noise.mean");
    HAKO_TEST_EXPECT(NearlyEqual(config.noise.stddev, 0.007), "unexpected noise.stddev");
}

void TestCameraProfileConfigLoader()
{
    hako::robots::sensor::camera::CameraProfileConfig config {};
    const auto path = (RepoRoot() / "config/sensors/color_camera/simple-color-camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadCameraProfileConfigFromJson(path, config);
    HAKO_TEST_EXPECT(ok, "simple-color-camera.json should load");
    HAKO_TEST_EXPECT(config.spec.frame_id == "color_camera_frame", "unexpected profile frame_id");
    HAKO_TEST_EXPECT(NearlyEqual(config.spec.update_rate, 10.0), "unexpected profile update_rate");
    HAKO_TEST_EXPECT(NearlyEqual(config.spec.horizontal_fov, 1.2), "unexpected profile horizontal_fov");
    HAKO_TEST_EXPECT(config.spec.image.width == 256, "unexpected profile image.width");
    HAKO_TEST_EXPECT(config.spec.image.height == 128, "unexpected profile image.height");
    HAKO_TEST_EXPECT(config.spec.image.format == "R8G8B8", "unexpected profile image.format");
    HAKO_TEST_EXPECT(NearlyEqual(config.spec.clip.near, 0.05), "unexpected profile clip.near");
    HAKO_TEST_EXPECT(NearlyEqual(config.spec.clip.far, 10.0), "unexpected profile clip.far");
    HAKO_TEST_EXPECT(config.spec.noise.type == "none", "unexpected profile noise.type");
    HAKO_TEST_EXPECT(config.mjcf_binding.camera_name == "color_camera", "unexpected camera_name");
    HAKO_TEST_EXPECT(config.mjcf_binding.body_name == "color_sensor_body", "unexpected body_name");
    HAKO_TEST_EXPECT(config.mjcf_binding.freejoint_name == "color_sensor_freejoint", "unexpected freejoint_name");
    HAKO_TEST_EXPECT(config.pdu_config.pdu_name == "camera_image", "unexpected pdu_name");
    HAKO_TEST_EXPECT(NearlyEqual(config.pdu_config.update_rate_hz, 10.0), "unexpected pdu update_rate_hz");
}

void TestDepthCameraConfigLoader()
{
    hako::robots::sensor::camera::DepthCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_depth_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadDepthCameraConfigFromJson(path, config);
    HAKO_TEST_EXPECT(ok, "sample_depth_camera.json should load");
    HAKO_TEST_EXPECT(config.frame_id == "camera_depth_frame", "unexpected depth frame_id");
    HAKO_TEST_EXPECT(NearlyEqual(config.update_rate, 30.0), "unexpected depth update_rate");
    HAKO_TEST_EXPECT(NearlyEqual(config.horizontal_fov, 1.047), "unexpected depth horizontal_fov");
    HAKO_TEST_EXPECT(config.image.width == 640, "unexpected depth image.width");
    HAKO_TEST_EXPECT(config.image.height == 480, "unexpected depth image.height");
    HAKO_TEST_EXPECT(config.image.format == "DEPTH_F32_M", "unexpected depth image.format");
    HAKO_TEST_EXPECT(NearlyEqual(config.clip.near, 0.1), "unexpected depth clip.near");
    HAKO_TEST_EXPECT(NearlyEqual(config.clip.far, 10.0), "unexpected depth clip.far");
    HAKO_TEST_EXPECT(config.noise.type == "gaussian", "unexpected depth noise.type");
    HAKO_TEST_EXPECT(NearlyEqual(config.noise.stddev, 0.01), "unexpected depth noise.stddev");
}

void TestRgbdCameraConfigLoader()
{
    hako::robots::sensor::camera::RgbdCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_rgbd_camera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadRgbdCameraConfigFromJson(path, config);
    HAKO_TEST_EXPECT(ok, "sample_rgbd_camera.json should load");
    HAKO_TEST_EXPECT(config.rgb.frame_id == "camera_rgb_frame", "unexpected rgb frame_id");
    HAKO_TEST_EXPECT(config.depth.frame_id == "camera_depth_frame", "unexpected rgbd depth frame_id");
    HAKO_TEST_EXPECT(config.rgb.image.width == 640, "unexpected rgb width");
    HAKO_TEST_EXPECT(config.depth.image.width == 640, "unexpected depth width");
    HAKO_TEST_EXPECT(config.rgb.image.height == 480, "unexpected rgb height");
    HAKO_TEST_EXPECT(config.depth.image.height == 480, "unexpected depth height");
    HAKO_TEST_EXPECT(NearlyEqual(config.rgb.horizontal_fov, 1.047), "unexpected rgb fov");
    HAKO_TEST_EXPECT(NearlyEqual(config.depth.horizontal_fov, 1.047), "unexpected depth fov");
}

void TestStereoCameraConfigLoader()
{
    hako::robots::sensor::camera::StereoCameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/sample_multicamera.json").string();
    const bool ok = hako::robots::sensor::camera::LoadStereoCameraConfigFromJson(path, config);
    HAKO_TEST_EXPECT(ok, "sample_multicamera.json should load");
    HAKO_TEST_EXPECT(config.left.frame_id == "left_camera_frame", "unexpected left frame_id");
    HAKO_TEST_EXPECT(config.right.frame_id == "right_camera_frame", "unexpected right frame_id");
    HAKO_TEST_EXPECT(config.left.image.width == 1280, "unexpected left width");
    HAKO_TEST_EXPECT(config.right.image.width == 1280, "unexpected right width");
    HAKO_TEST_EXPECT(NearlyEqual(config.left.horizontal_fov, 1.39626), "unexpected left fov");
    HAKO_TEST_EXPECT(NearlyEqual(config.right.horizontal_fov, 1.39626), "unexpected right fov");
    HAKO_TEST_EXPECT(NearlyEqual(config.baseline, 0.12, 1.0e-9), "unexpected stereo baseline");
}

void TestMissingFileFailure()
{
    hako::robots::sensor::camera::CameraConfig config {};
    const auto path = (RepoRoot() / "config/sensors/camera/does_not_exist.json").string();
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path, config);
    HAKO_TEST_EXPECT(!ok, "missing file should fail");
}

void TestInvalidJsonFailure()
{
    const auto path = std::filesystem::temp_directory_path() / "invalid_camera_config.json";
    std::ofstream ofs(path);
    ofs << "{ invalid json }";
    ofs.close();

    hako::robots::sensor::camera::CameraConfig config {};
    const bool ok = hako::robots::sensor::camera::LoadCameraConfigFromJson(path.string(), config);
    HAKO_TEST_EXPECT(!ok, "invalid json should fail");

    std::filesystem::remove(path);
}
}

int main()
{
    TestCameraConfigLoader();
    TestCameraProfileConfigLoader();
    TestDepthCameraConfigLoader();
    TestRgbdCameraConfigLoader();
    TestStereoCameraConfigLoader();
    TestMissingFileFailure();
    TestInvalidJsonFailure();

    std::cout << "camera_config_loader_test passed" << std::endl;
    return 0;
}
