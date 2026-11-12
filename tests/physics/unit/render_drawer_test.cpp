// Render server with MuJoCo's OpenGL renderer (needs a display; a hidden window
// is used). The main thread runs the server; a worker thread asks for images
// through IWorld::createImageRenderer, as a Robot Runtime physics thread would.
#include "physics/physics_impl.hpp"

#include <mujoco/mujoco.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace sa = hakoniwa::sensor_adapter;
using hako::robots::physics::impl::RenderServer;
using hako::robots::physics::impl::WorldImpl;

// Floor at z = 0, a red box ahead and above the camera's axis (x 2.9..3.1,
// z 1.2..1.6). The camera is at (0, 0, 1) looking along +x.
const char* kXml = R"(
<mujoco>
  <visual><global offwidth="64" offheight="64"/></visual>
  <worldbody>
    <light pos="0 0 5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.5 0.5 0.5 1"/>
    <geom name="box" type="box" pos="3 0 1.4" size="0.1 0.5 0.2" rgba="1 0 0 1"/>
  </worldbody>
</mujoco>
)";

constexpr std::uint32_t kW = 32;
constexpr std::uint32_t kH = 24;

void Expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

sa::RenderRequest Request(const std::string& name, std::vector<std::uint8_t>& rgb, std::vector<float>& depth)
{
    sa::RenderRequest r {};
    r.camera_name = name;
    r.pose.position_world = {0.0, 0.0, 1.0};
    r.pose.orientation_world_to_sensor = {1.0, 0.0, 0.0, 0.0};
    const double f = 0.5 * kW / std::tan(0.5);  // horizontal field of view 1 rad
    r.camera = {kW, kH, f, f, (kW - 1) * 0.5, (kH - 1) * 0.5, 0.05, 50.0};
    r.outputs = sa::kRenderRgb | sa::kRenderDepth;
    rgb.assign(static_cast<std::size_t>(kW) * kH * 3, 0);
    depth.assign(static_cast<std::size_t>(kW) * kH, -1.0F);
    r.rgb = {rgb.data(), rgb.size()};
    r.depth = {depth.data(), depth.size() * sizeof(float)};
    return r;
}

// Planar depth of pixel (u, v) by an independent mj_ray along the pixel's ray.
double ExpectedDepth(const mjModel* m, mjData* d, const sa::RenderCamera& c, std::uint32_t u, std::uint32_t v)
{
    // Optical frame (x right, y down, z forward) to the sensor axes FLU (= world here).
    mjtNum dir[3] = {1.0, -(u - c.cx) / c.fx, -(v - c.cy) / c.fy};
    const mjtNum length = mju_norm3(dir);
    mju_scl3(dir, dir, 1.0 / length);
    const mjtNum origin[3] = {0.0, 0.0, 1.0};
    int geom = -1;
    const mjtNum range = mj_ray(m, d, origin, dir, nullptr, 1, -1, &geom, nullptr);
    if (range < 0) {
        return std::numeric_limits<double>::infinity();
    }
    return range * dir[0];
}

}  // namespace

int main()
{
    const std::string path = "render_drawer_test_model.xml";
    {
        FILE* f = std::fopen(path.c_str(), "w");
        if (f == nullptr) {
            std::cerr << "FAIL: temporary model file" << std::endl;
            return 1;
        }
        std::fputs(kXml, f);
        std::fclose(f);
    }
    WorldImpl world;
    world.loadModel(path);
    std::remove(path.c_str());
    auto renderer = world.createImageRenderer();
    if (!renderer) {
        std::cerr << "FAIL: the world creates no image renderer" << std::endl;
        return 1;
    }

    std::string failure;
    std::atomic_bool running {true};
    std::thread worker([&]() {
        try {
            // One camera, colour and depth.
            std::vector<std::uint8_t> rgb;
            std::vector<float> depth;
            sa::RenderRequest r = Request("front", rgb, depth);
            sa::RenderStatus status {};
            Expect(renderer->render(&r, 1, &status) && status.ok, "render: " + status.error);
            double worst = 0.0;
            int finite = 0;
            for (std::uint32_t v = 0; v < kH; ++v) {
                for (std::uint32_t u = 0; u < kW; ++u) {
                    const double expected = ExpectedDepth(world.getModel(), world.getData(), r.camera, u, v);
                    const double got = depth[static_cast<std::size_t>(v) * kW + u];
                    if (std::isinf(expected)) {
                        Expect(std::isinf(got), "a pixel with nothing to see is +inf");
                        continue;
                    }
                    ++finite;
                    // Relative to the distance: a depth buffer's resolution falls off with distance.
                    worst = std::max(worst, std::fabs(got - expected) / expected);
                    if (std::getenv("RENDER_TEST_VERBOSE") != nullptr && std::fabs(got - expected) > 1e-3) {
                        std::fprintf(stderr, "u %u v %u expected %.4f got %.4f\n", u, v, expected, got);
                    }
                }
            }
            // On the floor seen at a grazing angle, depth changes by about 1.5 m per pixel row,
            // so a rasteriser difference of 1/1000 pixel is about 1 mm: compare relative to the
            // distance. Surfaces facing the camera match to a few micrometres (the box below).
            Expect(finite > 0 && worst < 5e-4,
                   "depth matches mj_ray per pixel (worst relative " + std::to_string(worst) + ")");
            // Rows from the top: the box (above the axis) is in the upper half, the floor below.
            const std::size_t top = static_cast<std::size_t>(kH / 2 - 3) * kW + kW / 2;
            const std::size_t bottom = static_cast<std::size_t>(kH - 1) * kW + kW / 2;
            Expect(std::fabs(depth[top] - 2.9) < 2e-3, "the upper rows see the box");
            if (std::getenv("RENDER_TEST_VERBOSE") != nullptr) {
                std::fprintf(stderr, "box pixel depth error %.2e m\n", std::fabs(depth[top] - 2.9));
            }
            Expect(rgb[top * 3] > 2 * rgb[top * 3 + 1], "the box is red in the colour image");
            Expect(depth[bottom] < 2.9, "the bottom row sees the floor");

            // A stereo-like batch at one state: identical poses give identical images.
            std::vector<std::uint8_t> rgb_a;
            std::vector<std::uint8_t> rgb_b;
            std::vector<float> depth_a;
            std::vector<float> depth_b;
            sa::RenderRequest batch[2] = {Request("left", rgb_a, depth_a), Request("right", rgb_b, depth_b)};
            sa::RenderStatus statuses[2] {};
            Expect(renderer->render(batch, 2, statuses) && statuses[0].ok && statuses[1].ok, "batch of two");
            Expect(rgb_a == rgb_b && depth_a == depth_b, "a batch is drawn at one world state");

            // Too small a destination: an error, nothing written.
            std::vector<std::uint8_t> small(10, 33);
            sa::RenderRequest bad = Request("small", rgb, depth);
            bad.outputs = sa::kRenderRgb;
            bad.rgb = {small.data(), small.size()};
            sa::RenderStatus bad_status {};
            Expect(!renderer->render(&bad, 1, &bad_status) && !bad_status.ok, "too small a destination fails");
            Expect(small[0] == 33, "nothing is written to it");
        } catch (const std::exception& e) {
            failure = e.what();
        }
        running = false;
    });
    // The main thread serves, as the application's main loop would.
    RenderServer::instance().run(running);
    worker.join();
    RenderServer::instance().release_drawers();
    if (!failure.empty()) {
        std::cerr << "FAIL: " << failure << std::endl;
        return 1;
    }
    std::cout << "render_drawer_test: OK" << std::endl;
    return 0;
}
