#include "examples/sensors/common/freejoint_motion.hpp"
#include "examples/sensors/ultrasonic/support/ultrasonic_example_support.hpp"
#include "physics/physics_impl.hpp"
#include "viewer/mujoco_viewer.hpp"
#include "sensors/debug/raycast_debug.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultModelPath =
    "models/sensors/ultrasonic/ultrasonic-sensor-test.xml";
constexpr const char* kDefaultConfigPath =
    "config/sensors/ultrasonic/lego-spike-distance-sensor.json";
constexpr const char* kSensorSiteName = "front_ultrasonic_site";
constexpr const char* kExcludeBodyName = "base_footprint";
constexpr const char* kBaseJointName = "base_freejoint";

constexpr double kMoveStep = 0.05;
constexpr double kDebugRayWidth = 0.006;

} // namespace

int main(int argc, char** argv)
{
    const std::string model_path =
        (argc >= 2) ? argv[1] : kDefaultModelPath;

    const std::string config_path =
        (argc >= 3) ? argv[2] : kDefaultConfigPath;

    hako::examples::sensors::ultrasonic::AppState state {};
    bool viewer_running = true;
    std::mutex mujoco_mutex;
    std::thread command_thread;

    try {
        auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
        world->loadModel(model_path);

        auto* model = world->getModel();
        auto* data = world->getData();

        if (model == nullptr || data == nullptr) {
            std::cerr << "ERROR: model/data is null" << std::endl;
            return EXIT_FAILURE;
        }

        const int qpos_addr = hako::examples::sensors::FindFreejointQposAddr(model, kBaseJointName);
        const int sensor_site_id =
            hako::examples::sensors::ultrasonic::FindSiteId(model, kSensorSiteName);

        hako::robots::sensor::ultrasonic::UltrasonicSensor sensor(
            world,
            kSensorSiteName,
            kExcludeBodyName);

        if (!sensor.LoadConfig(config_path)) {
            std::cerr
                << "ERROR: failed to load ultrasonic config: "
                << config_path << std::endl;
            return EXIT_FAILURE;
        }

        hako::robots::sensor::ultrasonic::UltrasonicFrame last_frame {};
        bool has_last_frame = false;

        std::cout << "Hakoniwa Ultrasonic Sensor Example" << std::endl;
        std::cout << "model : " << model_path << std::endl;
        std::cout << "config: " << config_path << std::endl;
        std::cout << "site  : " << kSensorSiteName << std::endl;

        hako::examples::sensors::ultrasonic::PrintHelp();
        hako::examples::sensors::PrintFreejointPosition(data, qpos_addr, "base_pos");

        /*
         * macOS/Cocoa/GLFW expects window creation and event handling to run
         * on the main thread. Therefore:
         *
         *   - command_thread only records keyboard requests
         *   - main thread runs viewer_thread_with_overlay()
         *   - UltrasonicSensor::Measure() is called from the viewer callback
         */
        command_thread = std::thread(
            hako::examples::sensors::ultrasonic::TerminalCommandLoop,
            std::ref(state));

        viewer_thread_with_overlay(
            model,
            data,
            viewer_running,
            mujoco_mutex,
            [&](mjvScene& scene) {
                if (!state.running.load()) {
                    viewer_running = false;
                    return;
                }

                const int forward_steps = state.move_forward.exchange(0);
                const int left_steps = state.move_left.exchange(0);
                if (forward_steps != 0 || left_steps != 0) {
                    hako::examples::sensors::MoveFreejointPlanarSteps(
                        model,
                        data,
                        qpos_addr,
                        forward_steps,
                        left_steps,
                        kMoveStep);
                    hako::examples::sensors::PrintPlanarStepMove(
                        "base",
                        forward_steps,
                        left_steps,
                        kMoveStep);
                    hako::examples::sensors::PrintFreejointPosition(data, qpos_addr, "base_pos");
                    state.pending_measure.store(true);
                }

                if (state.pending_measure.exchange(false)) {
                    hako::robots::sensor::ultrasonic::UltrasonicFrame frame {};
                    sensor.Measure(frame);
                    last_frame = frame;
                    has_last_frame = true;
                    hako::examples::sensors::ultrasonic::PrintFrame(frame);
                }

                if (state.print_help.exchange(false)) {
                    hako::examples::sensors::ultrasonic::PrintHelp();
                }

                if (!has_last_frame) {
                    return;
                }

                const auto line =
                    hako::examples::sensors::ultrasonic::MakeUltrasonicCenterRayDebugLine(
                        data,
                        sensor_site_id,
                        last_frame);

                hako::robots::sensor::debug::AddRaycastDebugLine(
                    scene,
                    line,
                    kDebugRayWidth);
            });

        /*
         * If the viewer window was closed, stop the command loop.
         * The command loop may be blocked on std::cin, so do not join it here.
         * The process is about to exit, so detaching is acceptable for this
         * simple interactive example.
         */
        viewer_running = false;
        state.running.store(false);

        if (command_thread.joinable()) {
            command_thread.detach();
        }

        std::cout << "bye" << std::endl;
        return EXIT_SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;

        viewer_running = false;
        state.running.store(false);

        if (command_thread.joinable()) {
            command_thread.detach();
        }

        return EXIT_FAILURE;
    }
}
