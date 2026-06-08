#include "viewer/mujoco_viewer.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

std::shared_ptr<hako::robots::sensor::camera::MujocoCameraRenderer>
MujocoRenderRuntime::CreateCameraRenderer(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    MakeContextCurrent();
    return std::make_shared<hako::robots::sensor::camera::MujocoCameraRenderer>(
        std::move(world),
        false);
}
