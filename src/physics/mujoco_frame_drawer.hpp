#pragma once

// MuJoCo frame drawer for the render server: MuJoCo's built-in OpenGL renderer
// drawn offscreen at each request's pose and intrinsics, through a hidden GLFW
// window's context. Create and use it only on the server thread (on macOS, the
// main thread). Ported from a tested renderer; the drawing is unchanged.

#include "physics/render_server.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hako::robots::physics::impl
{

namespace frame_drawer_detail
{

// GLFW is initialised once per process and terminated with its last user, unless
// the host had initialised it already (a viewer in the same process owns it then).
inline std::mutex& glfw_mutex()
{
    static std::mutex mutex;
    return mutex;
}

inline int& glfw_users()
{
    static int users = 0;
    return users;
}

inline bool& glfw_owned()
{
    static bool owned = false;
    return owned;
}

inline bool acquire_glfw()
{
    std::lock_guard<std::mutex> lock(glfw_mutex());
    if (glfw_users() == 0) {
        glfwGetError(nullptr);
        glfwGetTime();
        const bool initialised = glfwGetError(nullptr) != GLFW_NOT_INITIALIZED;
        if (!initialised && !glfwInit()) {
            return false;
        }
        glfw_owned() = !initialised;
    }
    ++glfw_users();
    return true;
}

inline void release_glfw()
{
    std::lock_guard<std::mutex> lock(glfw_mutex());
    --glfw_users();
    if (glfw_users() == 0 && glfw_owned()) {
        glfwTerminate();
    }
}

// The caller's current OpenGL context is restored (for example the viewer's).
class CurrentContextRestorer
{
public:
    CurrentContextRestorer()
        : previous_(glfwGetCurrentContext())
    {
    }
    ~CurrentContextRestorer()
    {
        glfwMakeContextCurrent(previous_);
    }
    void forget(GLFWwindow* window)
    {
        if (previous_ == window) {
            previous_ = nullptr;
        }
    }
    CurrentContextRestorer(const CurrentContextRestorer&) = delete;
    CurrentContextRestorer& operator=(const CurrentContextRestorer&) = delete;

private:
    GLFWwindow* previous_;
};

inline bool positive_finite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

}  // namespace frame_drawer_detail

class MujocoFrameDrawer final : public IFrameDrawer
{
public:
    MujocoFrameDrawer() = default;
    MujocoFrameDrawer(const MujocoFrameDrawer&) = delete;
    MujocoFrameDrawer& operator=(const MujocoFrameDrawer&) = delete;

    ~MujocoFrameDrawer() override
    {
        using frame_drawer_detail::CurrentContextRestorer;
        if (window_ == nullptr) {
            return;
        }
        CurrentContextRestorer restore;
        restore.forget(window_);
        glfwMakeContextCurrent(window_);
        if (depth_context_made_) {
            mjr_freeContext(&depth_context_);
        }
        if (scene_made_) {
            mjr_freeContext(&color_context_);
            mjv_freeScene(&scene_);
        }
        glfwDestroyWindow(window_);
        frame_drawer_detail::release_glfw();
    }

    /// The factory the render server uses by default.
    static std::unique_ptr<IFrameDrawer> create(const mjModel* model, mjData* data, std::string& error)
    {
        auto drawer = std::make_unique<MujocoFrameDrawer>();
        if (!drawer->open(model, data, error)) {
            return nullptr;
        }
        return drawer;
    }

    bool draw(const hakoniwa::sensor_adapter::RenderRequest* requests, std::size_t count,
              hakoniwa::sensor_adapter::RenderStatus* statuses) override
    {
        namespace sa = hakoniwa::sensor_adapter;
        bool all_ok = true;
        frame_drawer_detail::CurrentContextRestorer restore;
        glfwMakeContextCurrent(window_);
        for (std::size_t i = 0; i < count; ++i) {
            const auto& request = requests[i];
            auto& status = statuses[i];
            status = {};
            if (!validate(request, status.error)) {
                all_ok = false;
                continue;
            }
            const int w = static_cast<int>(request.camera.width);
            const int h = static_cast<int>(request.camera.height);
            const std::size_t pixels = static_cast<std::size_t>(w) * h;
            if ((request.outputs & sa::kRenderRgb) != 0) {
                rgb_raw_.resize(pixels * 3);
                if (!draw_one(request, color_context_, rgb_raw_.data(), nullptr)) {
                    status.error = request.camera_name + ": offscreen colour rendering is not available";
                    all_ok = false;
                    continue;
                }
                auto* rgb = static_cast<unsigned char*>(request.rgb.data);
                // OpenGL rows are bottom-up; the interface's rows are top-down.
                for (int v = 0; v < h; ++v) {
                    std::copy_n(rgb_raw_.data() + static_cast<std::size_t>(h - 1 - v) * w * 3,
                                static_cast<std::size_t>(w) * 3,
                                rgb + static_cast<std::size_t>(v) * w * 3);
                }
            }
            if ((request.outputs & sa::kRenderDepth) != 0) {
                depth_raw_.resize(pixels);
                if (!draw_one(request, depth_context_, nullptr, depth_raw_.data())) {
                    status.error = request.camera_name + ": offscreen depth rendering is not available";
                    all_ok = false;
                    continue;
                }
                // Depth buffer to planar metres for either depth map; background is +inf.
                const double zn = request.camera.near_clip;
                const double zf = request.camera.far_clip;
                const bool reversed = depth_context_.readDepthMap == mjDEPTH_ZEROFAR;
                const double far_value = reversed ? 0.0 : 1.0;
                auto* depth = static_cast<float*>(request.depth.data);
                for (int v = 0; v < h; ++v) {
                    for (int u = 0; u < w; ++u) {
                        const double d = depth_raw_[static_cast<std::size_t>(h - 1 - v) * w + u];
                        double z = std::numeric_limits<double>::infinity();
                        if (d != far_value) {
                            z = reversed ? zn * zf / (zn + d * (zf - zn))
                                         : zn * zf / (zf - d * (zf - zn));
                        }
                        depth[static_cast<std::size_t>(v) * w + u] = static_cast<float>(z);
                    }
                }
            }
            status.ok = true;
        }
        return all_ok;
    }

private:
    bool open(const mjModel* model, mjData* data, std::string& error)
    {
        if (model == nullptr || data == nullptr) {
            error = "MuJoCo model and data are required";
            return false;
        }
        model_ = model;
        data_ = data;
        if (!frame_drawer_detail::acquire_glfw()) {
            error = "GLFW could not be initialised (no display?)";
            return false;
        }
        frame_drawer_detail::CurrentContextRestorer restore;
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(1, 1, "hakoniwa-render-server", nullptr, nullptr);
        // The hint must not leak into the host's windows.
        glfwDefaultWindowHints();
        if (window_ == nullptr) {
            frame_drawer_detail::release_glfw();
            error = "could not create a hidden OpenGL window for offscreen rendering";
            return false;
        }
        glfwMakeContextCurrent(window_);
        mjv_defaultScene(&scene_);
        mjv_defaultOption(&option_);
        mjv_defaultCamera(&camera_);
        mjr_defaultContext(&color_context_);
        mjr_defaultContext(&depth_context_);
        mjv_makeScene(model_, &scene_, 10000);
        mjr_makeContext(model_, &color_context_, mjFONTSCALE_100);
        scene_made_ = true;
        // The same model with offscreen multisampling off (a shallow copy used only
        // here): a resolved multisampled depth is shifted by a fraction of a pixel.
        mjModel no_msaa = *model_;
        no_msaa.vis.quality.offsamples = 0;
        mjr_makeContext(&no_msaa, &depth_context_, mjFONTSCALE_100);
        depth_context_made_ = true;
        mjr_setBuffer(mjFB_OFFSCREEN, &color_context_);
        if (color_context_.currentBuffer != mjFB_OFFSCREEN) {
            error = "offscreen rendering is not available";
            return false;
        }
        return true;
    }

    bool validate(const hakoniwa::sensor_adapter::RenderRequest& request, std::string& error) const
    {
        namespace sa = hakoniwa::sensor_adapter;
        using frame_drawer_detail::positive_finite;
        const auto& camera = request.camera;
        const std::string& name = request.camera_name;
        if (camera.width == 0 || camera.height == 0) {
            error = name + ": image width and height must be positive";
            return false;
        }
        if (camera.width > INT_MAX || camera.height > INT_MAX) {
            error = name + ": image size exceeds the renderer limit";
            return false;
        }
        if (!positive_finite(camera.fx)) {
            error = name + ": fx must be a positive finite number";
            return false;
        }
        if (!positive_finite(camera.fy)) {
            error = name + ": fy must be a positive finite number";
            return false;
        }
        if (!std::isfinite(camera.cx) || !std::isfinite(camera.cy)) {
            error = name + ": cx and cy must be finite";
            return false;
        }
        if (!positive_finite(camera.near_clip) || !std::isfinite(camera.far_clip)
            || !(camera.far_clip > camera.near_clip)) {
            error = name + ": clip must satisfy 0 < near < far";
            return false;
        }
        if (std::fabs(camera.fx - camera.fy) > 1e-9 * std::max(camera.fx, camera.fy)) {
            error = name + ": MuJoCo rendering requires square pixels (fx == fy)";
            return false;
        }
        const auto& position = request.pose.position_world;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
            error = name + ": camera pose position must be finite";
            return false;
        }
        const auto& q = request.pose.orientation_world_to_sensor;
        if (!positive_finite(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)) {
            error = name + ": camera pose quaternion must be finite and non-zero";
            return false;
        }
        if (request.outputs == 0 || (request.outputs & ~(sa::kRenderRgb | sa::kRenderDepth)) != 0) {
            error = name + ": unsupported render output";
            return false;
        }
        const std::size_t pixels = static_cast<std::size_t>(camera.width) * camera.height;
        if ((request.outputs & sa::kRenderRgb) != 0
            && (request.rgb.data == nullptr || request.rgb.capacity < pixels * 3)) {
            error = name + ": RGB destination too small";
            return false;
        }
        if ((request.outputs & sa::kRenderDepth) != 0
            && (request.depth.data == nullptr || request.depth.capacity < pixels * sizeof(float))) {
            error = name + ": depth destination too small";
            return false;
        }
        return true;
    }

    // Draws one request into an offscreen buffer and reads it back (rows bottom-up).
    bool draw_one(const hakoniwa::sensor_adapter::RenderRequest& request, mjrContext& context,
                  unsigned char* rgb, float* depth)
    {
        const int w = static_cast<int>(request.camera.width);
        const int h = static_cast<int>(request.camera.height);
        // Grow the offscreen buffer when needed (the model is not changed).
        if (w > context.offWidth || h > context.offHeight) {
            mjr_resizeOffscreen(std::max(w, context.offWidth), std::max(h, context.offHeight), &context);
        }
        mjr_setBuffer(mjFB_OFFSCREEN, &context);
        if (context.currentBuffer != mjFB_OFFSCREEN) {
            return false;
        }
        // The camera is set on the scene after mjv_updateScene, so the model is not changed.
        mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
        const auto& pose = request.pose;
        const auto& intrinsics = request.camera;
        mjtNum sensor_to_world[4] = {
            pose.orientation_world_to_sensor.w,
            -pose.orientation_world_to_sensor.x,
            -pose.orientation_world_to_sensor.y,
            -pose.orientation_world_to_sensor.z,
        };
        mju_normalize4(sensor_to_world);
        mjtNum rotation[9];
        mju_quat2Mat(rotation, sensor_to_world);
        const double top = intrinsics.near_clip * (intrinsics.cy + 0.5) / intrinsics.fy;
        const double bottom = -intrinsics.near_clip * (static_cast<double>(h) - 0.5 - intrinsics.cy) / intrinsics.fy;
        const double horizontal_center =
            intrinsics.near_clip * ((static_cast<double>(w) - 1.0) * 0.5 - intrinsics.cx) / intrinsics.fx;
        const double position[3] = {pose.position_world.x, pose.position_world.y, pose.position_world.z};
        for (auto& gl_camera : scene_.camera) {
            for (int i = 0; i < 3; ++i) {
                gl_camera.pos[i] = static_cast<float>(position[i]);
                gl_camera.forward[i] = static_cast<float>(rotation[3 * i + 0]);
                gl_camera.up[i] = static_cast<float>(rotation[3 * i + 2]);
            }
            gl_camera.frustum_center = static_cast<float>(horizontal_center);
            gl_camera.frustum_bottom = static_cast<float>(bottom);
            gl_camera.frustum_top = static_cast<float>(top);
            gl_camera.frustum_near = static_cast<float>(intrinsics.near_clip);
            gl_camera.frustum_far = static_cast<float>(intrinsics.far_clip);
            gl_camera.orthographic = 0;
        }
        const mjrRect viewport {0, 0, w, h};
        mjr_render(viewport, &scene_, &context);
        mjr_readPixels(rgb, depth, viewport, &context);
        return true;
    }

    const mjModel* model_ {nullptr};
    mjData* data_ {nullptr};
    GLFWwindow* window_ {nullptr};
    bool scene_made_ {false};
    bool depth_context_made_ {false};
    mjvScene scene_ {};
    mjvOption option_ {};
    mjvCamera camera_ {};
    mjrContext color_context_ {};
    mjrContext depth_context_ {};
    std::vector<unsigned char> rgb_raw_;
    std::vector<float> depth_raw_;
};

}  // namespace hako::robots::physics::impl
