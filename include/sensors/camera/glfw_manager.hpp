#pragma once

#include <stdexcept>
#include <GLFW/glfw3.h>

namespace hako::robots::sensor::camera
{
    class GlfwManager
    {
    public:
        static GlfwManager& getInstance()
        {
            static GlfwManager instance;
            return instance;
        }

        GlfwManager(const GlfwManager&) = delete;
        void operator=(const GlfwManager&) = delete;

    private:
        GlfwManager()
        {
#if defined(__APPLE__)
            glfwInitHint(GLFW_COCOA_MENUBAR, GLFW_FALSE);
            glfwInitHint(GLFW_COCOA_CHDIR_RESOURCES, GLFW_FALSE);
#endif
            if (!glfwInit()) {
                throw std::runtime_error("Failed to initialize GLFW");
            }
        }

        ~GlfwManager()
        {
            glfwTerminate();
        }
    };
}
