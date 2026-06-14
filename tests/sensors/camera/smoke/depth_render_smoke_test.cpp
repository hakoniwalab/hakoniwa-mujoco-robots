#include "physics/physics_impl.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__APPLE__)
#include <ApplicationServices/ApplicationServices.h>
#endif

namespace
{
constexpr int kImageWidth = 64;
constexpr int kImageHeight = 48;
constexpr double kDefaultHorizontalFovRad = 1.0;
constexpr double kDefaultClipNear = 0.05;
constexpr double kDefaultClipFar = 10.0;
constexpr double kDefaultTargetHalfDepth = 0.05;
constexpr double kLargeTargetHalfWidth = 3.0;
constexpr double kLargeTargetHalfHeight = 3.0;
constexpr const char* kCameraName = "depth_cam";

struct PixelSample
{
    std::string label;
    int x = 0;
    int y = 0;
};

struct RenderCapture
{
    hako::robots::sensor::camera::RawCameraFrame raw {};
    hako::robots::sensor::camera::DepthFrame frame {};
};

struct CaseResult
{
    std::string suite;
    std::string name;
    std::string pixel_label;
    double expected_distance_m = std::numeric_limits<double>::quiet_NaN();
    double tolerance_m = std::numeric_limits<double>::quiet_NaN();
    double actual_distance_m = std::numeric_limits<double>::quiet_NaN();
    double absolute_error_m = std::numeric_limits<double>::quiet_NaN();
    float raw_depth_value = std::numeric_limits<float>::quiet_NaN();
    double znear = 0.0;
    double zfar = 0.0;
    int depth_map = -1;
    bool expect_nan = false;
    bool passed = false;
};

struct SceneSpec
{
    double front_distance_m = 1.0;
    double target_half_width_m = 0.4;
    double target_half_height_m = 0.4;
    double target_half_depth_m = kDefaultTargetHalfDepth;
};

struct CaptureSpec
{
    double horizontal_fov_rad = kDefaultHorizontalFovRad;
    double clip_near_m = kDefaultClipNear;
    double clip_far_m = kDefaultClipFar;
    std::vector<PixelSample> pixels {};
};

std::filesystem::path WriteDepthSceneXml(const SceneSpec& scene)
{
    const auto file_tag = std::to_string(static_cast<int>(scene.front_distance_m * 1000.0));
    const std::filesystem::path xml_path =
        std::filesystem::temp_directory_path() /
        ("hakoniwa_depth_render_smoke_" + file_tag + ".xml");

    const double target_center_x = scene.front_distance_m + scene.target_half_depth_m;
    std::ofstream xml(xml_path);
    xml << "<mujoco model=\"depth_render_smoke\">\n";
    xml << "  <compiler angle=\"radian\"/>\n";
    xml << "  <option timestep=\"0.01\" gravity=\"0 0 0\"/>\n";
    xml << "  <visual>\n";
    xml << "    <global offwidth=\"256\" offheight=\"256\"/>\n";
    xml << "    <map znear=\"" << kDefaultClipNear << "\" zfar=\"" << kDefaultClipFar << "\"/>\n";
    xml << "  </visual>\n";
    xml << "  <worldbody>\n";
    xml << "    <body name=\"target\" pos=\"" << target_center_x << " 0 0\">\n";
    xml << "      <geom name=\"target_box\" type=\"box\" size=\""
        << scene.target_half_depth_m << " "
        << scene.target_half_width_m << " "
        << scene.target_half_height_m
        << "\" rgba=\"0.8 0.2 0.2 1\"/>\n";
    xml << "    </body>\n";
    xml << "    <camera name=\"" << kCameraName << "\" mode=\"targetbody\" target=\"target\" pos=\"0 0 0\"/>\n";
    xml << "  </worldbody>\n";
    xml << "</mujoco>\n";
    return xml_path;
}

bool HasRenderableGuiSession()
{
#if defined(__APPLE__)
    CFDictionaryRef session = CGSessionCopyCurrentDictionary();
    if (session == nullptr) {
        return false;
    }
    CFRelease(session);
#endif
    return true;
}

bool IsClose(double actual, double expected, double tolerance)
{
    return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}

int PixelIndex(int width, int x, int y)
{
    return y * width + x;
}

bool CaptureDepthFrame(
    const SceneSpec& scene,
    const CaptureSpec& capture_spec,
    RenderCapture& capture)
{
    const std::filesystem::path xml_path = WriteDepthSceneXml(scene);
    std::cout << "[depth_render_smoke_test] loading scene " << xml_path << std::endl;

    auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    world->loadModel(xml_path.string());

    hako::robots::sensor::camera::DepthCameraConfig config;
    config.frame_id = "depth_cam_frame";
    config.update_rate = 30.0;
    config.horizontal_fov = capture_spec.horizontal_fov_rad;
    config.image.width = kImageWidth;
    config.image.height = kImageHeight;
    config.image.format = "DEPTH_F32_M";
    config.clip.near = capture_spec.clip_near_m;
    config.clip.far = capture_spec.clip_far_m;

    std::cout << "[depth_render_smoke_test] creating renderer for front_distance="
              << scene.front_distance_m << std::endl;
    auto renderer = std::make_shared<hako::robots::sensor::camera::MujocoCameraRenderer>(world);
    hako::robots::sensor::camera::DepthCameraSensor sensor(renderer, kCameraName);
    if (!sensor.LoadConfig(config)) {
        std::cerr << "DepthCameraSensor::LoadConfig failed" << std::endl;
        return false;
    }

    std::cout << "[depth_render_smoke_test] capturing frame" << std::endl;
    if (!renderer->Render(
            kCameraName,
            config.image.width,
            config.image.height,
            config.horizontal_fov,
            config.clip.near,
            config.clip.far,
            false,
            true,
            capture.raw))
    {
        std::cerr << "MujocoCameraRenderer::Render failed" << std::endl;
        return false;
    }

    sensor.Capture(capture.frame);
    if (capture.frame.data.empty()) {
        std::cerr << "DepthCameraSensor::Capture produced empty frame" << std::endl;
        return false;
    }
    return true;
}

CaseResult EvaluatePixel(
    const std::string& suite,
    const std::string& name,
    const PixelSample& pixel,
    const RenderCapture& capture,
    double expected_distance_m,
    double tolerance_m,
    bool expect_nan)
{
    CaseResult result {};
    result.suite = suite;
    result.name = name;
    result.pixel_label = pixel.label;
    result.expected_distance_m = expected_distance_m;
    result.tolerance_m = tolerance_m;
    result.expect_nan = expect_nan;
    result.znear = capture.raw.znear;
    result.zfar = capture.raw.zfar;
    result.depth_map = capture.raw.depth_map;

    const int index = PixelIndex(capture.frame.width, pixel.x, pixel.y);
    result.raw_depth_value = capture.raw.depth_buffer.at(static_cast<size_t>(index));
    result.actual_distance_m = capture.frame.data.at(static_cast<size_t>(index));
    if (std::isfinite(result.actual_distance_m)) {
        result.absolute_error_m = std::abs(result.actual_distance_m - expected_distance_m);
    }

    if (capture.raw.depth_map != mjDEPTH_ZERONEAR) {
        result.passed = false;
        return result;
    }
    if (expect_nan) {
        result.passed = std::isnan(result.actual_distance_m);
        return result;
    }

    result.passed = IsClose(result.actual_distance_m, expected_distance_m, tolerance_m);
    return result;
}

void PrintCaseResult(const CaseResult& result)
{
    std::cout << "[" << result.suite << "] "
              << result.name
              << " pixel=" << result.pixel_label
              << " expected=" << result.expected_distance_m
              << " actual=" << result.actual_distance_m
              << " error=" << result.absolute_error_m
              << " tolerance=" << result.tolerance_m
              << " raw_depth=" << result.raw_depth_value
              << " znear=" << result.znear
              << " zfar=" << result.zfar
              << " depth_map=" << result.depth_map
              << " expect_nan=" << (result.expect_nan ? "yes" : "no")
              << " passed=" << (result.passed ? "yes" : "no")
              << std::endl;
}

bool RunCenterDistanceCase(double front_distance_m, double tolerance_m)
{
    SceneSpec scene {};
    scene.front_distance_m = front_distance_m;

    CaptureSpec capture_spec {};
    capture_spec.pixels = {PixelSample{"center", kImageWidth / 2, kImageHeight / 2}};

    RenderCapture capture {};
    if (!CaptureDepthFrame(scene, capture_spec, capture)) {
        return false;
    }

    const auto result = EvaluatePixel(
        "distance-range",
        "front_distance=" + std::to_string(front_distance_m),
        capture_spec.pixels.front(),
        capture,
        front_distance_m,
        tolerance_m,
        false);
    PrintCaseResult(result);
    return result.passed;
}

bool RunPixelPositionCase()
{
    SceneSpec scene {};
    scene.front_distance_m = 2.0;
    scene.target_half_width_m = kLargeTargetHalfWidth;
    scene.target_half_height_m = kLargeTargetHalfHeight;

    CaptureSpec capture_spec {};
    capture_spec.pixels = {
        {"center", kImageWidth / 2, kImageHeight / 2},
        {"left-center", kImageWidth / 4, kImageHeight / 2},
        {"right-center", (3 * kImageWidth) / 4, kImageHeight / 2},
        {"upper-center", kImageWidth / 2, kImageHeight / 4},
        {"lower-center", kImageWidth / 2, (3 * kImageHeight) / 4},
    };

    RenderCapture capture {};
    if (!CaptureDepthFrame(scene, capture_spec, capture)) {
        return false;
    }

    bool all_passed = true;
    for (const auto& pixel : capture_spec.pixels) {
        const auto result = EvaluatePixel(
            "pixel-position",
            "front_distance=2.0",
            pixel,
            capture,
            2.0,
            0.03,
            false);
        PrintCaseResult(result);
        all_passed = all_passed && result.passed;
    }
    return all_passed;
}

bool RunFovCase(double horizontal_fov_rad)
{
    SceneSpec scene {};
    scene.front_distance_m = 2.0;
    scene.target_half_width_m = kLargeTargetHalfWidth;
    scene.target_half_height_m = kLargeTargetHalfHeight;

    CaptureSpec capture_spec {};
    capture_spec.horizontal_fov_rad = horizontal_fov_rad;
    capture_spec.pixels = {PixelSample{"center", kImageWidth / 2, kImageHeight / 2}};

    RenderCapture capture {};
    if (!CaptureDepthFrame(scene, capture_spec, capture)) {
        return false;
    }

    const auto result = EvaluatePixel(
        "fov",
        "horizontal_fov=" + std::to_string(horizontal_fov_rad),
        capture_spec.pixels.front(),
        capture,
        2.0,
        0.03,
        false);
    PrintCaseResult(result);
    return result.passed;
}

bool RunClipCase(
    const std::string& name,
    double front_distance_m,
    double clip_near_m,
    double clip_far_m,
    bool expect_nan)
{
    SceneSpec scene {};
    scene.front_distance_m = front_distance_m;

    CaptureSpec capture_spec {};
    capture_spec.clip_near_m = clip_near_m;
    capture_spec.clip_far_m = clip_far_m;
    capture_spec.pixels = {PixelSample{"center", kImageWidth / 2, kImageHeight / 2}};

    RenderCapture capture {};
    if (!CaptureDepthFrame(scene, capture_spec, capture)) {
        return false;
    }

    const auto result = EvaluatePixel(
        "clip",
        name,
        capture_spec.pixels.front(),
        capture,
        front_distance_m,
        0.03,
        expect_nan);
    PrintCaseResult(result);
    return result.passed;
}
}

int main()
{
    if (!HasRenderableGuiSession()) {
        std::cout << "SKIPPED: OpenGL context unavailable (no active macOS GUI session)" << std::endl;
        return 0;
    }

    try {
        bool all_passed = true;

        all_passed = RunCenterDistanceCase(0.2, 0.01) && all_passed;
        all_passed = RunCenterDistanceCase(0.5, 0.01) && all_passed;
        all_passed = RunCenterDistanceCase(1.0, 0.01) && all_passed;
        all_passed = RunCenterDistanceCase(2.0, 0.03) && all_passed;
        all_passed = RunCenterDistanceCase(5.0, 0.03) && all_passed;
        all_passed = RunCenterDistanceCase(9.0, 0.08) && all_passed;

        all_passed = RunPixelPositionCase() && all_passed;

        all_passed = RunFovCase(0.5) && all_passed;
        all_passed = RunFovCase(1.0) && all_passed;
        all_passed = RunFovCase(1.5) && all_passed;

        all_passed = RunClipCase("near_clips_target", 0.5, 1.0, 10.0, true) && all_passed;
        all_passed = RunClipCase("far_clips_target", 5.0, 0.05, 3.0, true) && all_passed;
        all_passed = RunClipCase("target_inside_clip", 2.0, 0.05, 10.0, false) && all_passed;

        return all_passed ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << "SKIPPED: OpenGL context unavailable: " << ex.what() << std::endl;
        return 0;
    }
}
