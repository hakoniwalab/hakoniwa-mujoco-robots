#include "sensors/camera/mujoco_camera_renderer.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hako::robots::sensor::camera
{
namespace
{
class CameraFovyOverrideGuard
{
public:
    CameraFovyOverrideGuard(mjModel* model, int cam_id, mjtNum new_fovy)
        : model_(model), cam_id_(cam_id), original_fovy_(0), active_(false)
    {
        if (model_ == nullptr || cam_id_ < 0) {
            return;
        }
        original_fovy_ = model_->cam_fovy[cam_id_];
        model_->cam_fovy[cam_id_] = new_fovy;
        active_ = true;
    }

    ~CameraFovyOverrideGuard()
    {
        if (active_) {
            model_->cam_fovy[cam_id_] = original_fovy_;
        }
    }

    CameraFovyOverrideGuard(const CameraFovyOverrideGuard&) = delete;
    CameraFovyOverrideGuard& operator=(const CameraFovyOverrideGuard&) = delete;

private:
    mjModel* model_;
    int cam_id_;
    mjtNum original_fovy_;
    bool active_;
};
}

MujocoCameraRenderer::MujocoCameraRenderer(std::shared_ptr<hako::robots::physics::IWorld> world)
    : MujocoCameraRenderer(std::move(world), true)
{
}

MujocoCameraRenderer::MujocoCameraRenderer(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    bool create_hidden_window)
    : world_(world), glfw_manager_(GlfwManager::getInstance()), window_(nullptr)
{
    if (!world) {
        throw std::invalid_argument("World is null");
    }
    if (create_hidden_window) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(1, 1, "Offscreen", nullptr, nullptr);
        if (!window_) {
            throw std::runtime_error(
                "Failed to create hidden GLFW window for MuJoCo offscreen rendering. "
                "Ensure an OpenGL-capable display/GPU context is available."
            );
        }
        owns_window_ = true;
        glfwMakeContextCurrent(window_);
    } else if (glfwGetCurrentContext() == nullptr) {
        throw std::runtime_error(
            "MujocoCameraRenderer was asked to use the current OpenGL context, "
            "but no GLFW context is current."
        );
    }

    auto* model = world_->getModel();
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, 2000);
    mjr_defaultContext(&con_);
    mjr_makeContext(model, &con_, mjFONTSCALE_150);
}

MujocoCameraRenderer::~MujocoCameraRenderer()
{
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    if (owns_window_ && window_) {
        glfwDestroyWindow(window_);
    }
}

bool MujocoCameraRenderer::Render(
    const std::string& camera_name, int width, int height, double hfov_rad,
    bool need_rgb, bool need_depth, RawCameraFrame& out)
{
    if (!need_rgb && !need_depth) return false;

    if (window_ != nullptr) {
        glfwMakeContextCurrent(window_);
    }
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    if (con_.currentBuffer != mjFB_OFFSCREEN) {
        std::cerr << "Offscreen rendering is not available." << std::endl;
        return false;
    }

    if (width > con_.offWidth || height > con_.offHeight) {
        std::cerr << "Requested camera image size " << width << "x" << height
                  << " exceeds MuJoCo offscreen buffer size " << con_.offWidth << "x" << con_.offHeight
                  << std::endl;
        return false;
    }

    auto* model = world_->getModel();
    auto* data = world_->getData();
    int cam_id = mj_name2id(model, mjOBJ_CAMERA, camera_name.c_str());
    if (cam_id < 0) {
        std::cerr << "Camera not found: " << camera_name << std::endl;
        return false;
    }

    cam_.type = mjCAMERA_FIXED;
    cam_.fixedcamid = cam_id;
    // MuJoCo 3.8 no longer exposes mjvCamera::fovy, so fixed-camera rendering
    // temporarily overrides model->cam_fovy[cam_id] for the duration of this render.
    // This mutates the shared mjModel and is therefore not thread-safe across
    // concurrent Render() calls on the same model. The current design assumes
    // single-threaded sensor updates.
    std::unique_ptr<CameraFovyOverrideGuard> fovy_override_guard;
    if (width > 0 && height > 0) {
        const double vfov_rad = 2.0 * std::atan(std::tan(hfov_rad / 2.0) * (height / static_cast<double>(width)));
        fovy_override_guard = std::make_unique<CameraFovyOverrideGuard>(
            model, cam_id, static_cast<mjtNum>(vfov_rad * 180.0 / M_PI));
    }

    mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(model, data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);

    out.width = width;
    out.height = height;
    const double extent = static_cast<double>(model->stat.extent);
    out.znear = static_cast<double>(model->vis.map.znear) * extent;
    out.zfar = static_cast<double>(model->vis.map.zfar) * extent;
    out.depth_map = con_.readDepthMap;

    if (need_rgb) {
        out.rgb.resize(width * height * 3);
    }
    if (need_depth) {
        out.depth_buffer.resize(width * height);
    }

    mjr_readPixels(
        need_rgb ? out.rgb.data() : nullptr,
        need_depth ? out.depth_buffer.data() : nullptr,
        viewport,
        &con_
    );

    if (need_rgb) {
        std::vector<uint8_t> flipped_rgb(out.rgb.size());
        int row_size = width * 3;
        for (int y = 0; y < height; ++y) {
            std::memcpy(
                flipped_rgb.data() + y * row_size,
                out.rgb.data() + (height - 1 - y) * row_size,
                row_size);
        }
        out.rgb = std::move(flipped_rgb);
    }

    if (need_depth) {
        std::vector<float> flipped_depth(out.depth_buffer.size());
        for (int y = 0; y < height; ++y) {
            std::copy_n(
                out.depth_buffer.data() + (height - 1 - y) * width,
                width,
                flipped_depth.data() + y * width
            );
        }
        out.depth_buffer = std::move(flipped_depth);
    }
    
    out.timestamp = data->time;
    return true;
}

}
