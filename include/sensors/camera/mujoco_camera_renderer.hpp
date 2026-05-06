#pragma once

#include "sensors/camera/glfw_manager.hpp"
#include "physics.hpp"
#include <memory>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

namespace hako::robots::sensor::camera
{
    struct RawCameraFrame
    {
        int width = 0;
        int height = 0;
        std::vector<uint8_t> rgb;
        std::vector<float> depth_buffer;
        double timestamp = 0.0;
        double znear = 0.0;
        double zfar = 0.0;
        int depth_map = mjDEPTH_ZERONEAR;
    };

    class MujocoCameraRenderer
    {
    public:
        MujocoCameraRenderer(std::shared_ptr<hako::robots::physics::IWorld> world);
        ~MujocoCameraRenderer();

        bool Render(
            const std::string& camera_name,
            int width,
            int height,
            double hfov_rad,
            bool need_rgb,
            bool need_depth,
            RawCameraFrame& out
        );

    private:
        std::shared_ptr<hako::robots::physics::IWorld> world_;
        GlfwManager& glfw_manager_;
        GLFWwindow* window_ = nullptr;
        
        mjvScene scn_{};
        mjrContext con_{};
        mjvCamera cam_{};
        mjvOption opt_{};
    };
}
