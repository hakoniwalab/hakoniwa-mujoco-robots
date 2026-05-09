#include "physics.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "mujoco_viewer.hpp"

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultModelPath =
    "models/sensors/ultrasonic/ultrasonic-sensor-test.xml";

/*
 * If you created a no-noise test config, prefer:
 *
 *   config/sensors/ultrasonic/ultrasonic-sensor-test.json
 *
 * Keep this path aligned with RuntimeBinding.source_site.
 */
constexpr const char* kDefaultConfigPath =
    "config/sensors/ultrasonic/lego-spike-distance-sensor.json";

constexpr const char* kSensorSiteName = "front_ultrasonic_site";
constexpr const char* kExcludeBodyName = "base_footprint";

constexpr double kMoveStep = 0.05;

std::string status_to_string(
    hako::robots::sensor::ultrasonic::UltrasonicStatus status)
{
    using hako::robots::sensor::ultrasonic::UltrasonicStatus;

    switch (status) {
    case UltrasonicStatus::OK:
        return "OK";
    case UltrasonicStatus::NO_HIT:
        return "NO_HIT";
    case UltrasonicStatus::BELOW_MIN_RANGE:
        return "BELOW_MIN_RANGE";
    case UltrasonicStatus::INVALID:
        return "INVALID";
    default:
        return "UNKNOWN";
    }
}

class ExampleWorld final : public hako::robots::physics::IWorld {
public:
    void loadModel(const std::string& model_file) override
    {
        char error[1024] = {0};

        model = mj_loadXML(model_file.c_str(), nullptr, error, sizeof(error));
        if (model == nullptr) {
            throw std::runtime_error(
                std::string("failed to load MuJoCo model: ") +
                model_file +
                "\n" +
                error);
        }

        data = mj_makeData(model);
        if (data == nullptr) {
            throw std::runtime_error("failed to allocate MuJoCo data");
        }

        mj_forward(model, data);
    }

    void advanceTimeStep() override
    {
        if (model != nullptr && data != nullptr) {
            mj_step(model, data);
        }
    }

    std::shared_ptr<hako::robots::physics::IRigidBody>
    getRigidBody(const std::string& /*model_name*/) override
    {
        return nullptr;
    }

    std::shared_ptr<hako::robots::actuator::ITorqueActuator>
    getTorqueActuator(const std::string& /*name*/) override
    {
        return nullptr;
    }
};

int find_base_freejoint_qpos_addr(const mjModel* model)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, "base_freejoint");
    if (joint_id < 0) {
        throw std::runtime_error(
            "joint 'base_freejoint' was not found. "
            "Add <freejoint name=\"base_freejoint\"/> under base_footprint.");
    }

    if (model->jnt_type[joint_id] != mjJNT_FREE) {
        throw std::runtime_error("joint 'base_freejoint' is not a freejoint");
    }

    return model->jnt_qposadr[joint_id];
}

void print_pose(const mjData* data, int qpos_addr)
{
    std::cout
        << "base_pos=("
        << std::fixed << std::setprecision(3)
        << data->qpos[qpos_addr + 0] << ", "
        << data->qpos[qpos_addr + 1] << ", "
        << data->qpos[qpos_addr + 2] << ")"
        << std::endl;
}

void move_base(mjModel* model, mjData* data, int qpos_addr, double dx, double dy)
{
    data->qpos[qpos_addr + 0] += dx;
    data->qpos[qpos_addr + 1] += dy;

    /*
     * Keep the freejoint orientation fixed.
     *
     * MuJoCo freejoint qpos layout:
     *   qpos[0..2] = position
     *   qpos[3..6] = quaternion (w, x, y, z)
     */
    data->qpos[qpos_addr + 3] = 1.0;
    data->qpos[qpos_addr + 4] = 0.0;
    data->qpos[qpos_addr + 5] = 0.0;
    data->qpos[qpos_addr + 6] = 0.0;

    mj_forward(model, data);
}

void print_help()
{
    std::cout << R"(

Controls:
  i : move forward  (+X)
  k : move backward (-X)
  j : move left     (+Y)
  l : move right    (-Y)
  s : sense and print ultrasonic range
  h : help
  q : quit

Note:
  On macOS, the MuJoCo viewer runs on the main thread.
  Console input runs on a worker thread.

)" << std::endl;
}

void run_command_loop(
    bool& running,
    std::mutex& mujoco_mutex,
    mjModel* model,
    mjData* data,
    int qpos_addr,
    hako::robots::sensor::ultrasonic::UltrasonicSensor& sensor)
{
    while (running) {
        std::cout << "> " << std::flush;

        char key = '\0';
        std::cin >> key;

        if (!std::cin) {
            running = false;
            break;
        }

        if (key == 'q') {
            running = false;
            break;
        }

        switch (key) {
        case 'i':
        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            move_base(model, data, qpos_addr, +kMoveStep, 0.0);
            std::cout << "moved: x += " << kMoveStep << ", ";
            print_pose(data, qpos_addr);
            break;
        }

        case 'k':
        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            move_base(model, data, qpos_addr, -kMoveStep, 0.0);
            std::cout << "moved: x -= " << kMoveStep << ", ";
            print_pose(data, qpos_addr);
            break;
        }

        case 'j':
        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            move_base(model, data, qpos_addr, 0.0, +kMoveStep);
            std::cout << "moved: y += " << kMoveStep << ", ";
            print_pose(data, qpos_addr);
            break;
        }

        case 'l':
        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            move_base(model, data, qpos_addr, 0.0, -kMoveStep);
            std::cout << "moved: y -= " << kMoveStep << ", ";
            print_pose(data, qpos_addr);
            break;
        }

        case 's':
        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);

            hako::robots::sensor::ultrasonic::UltrasonicFrame frame;
            sensor.Measure(frame);

            std::cout
                << "range="
                << std::fixed << std::setprecision(3)
                << frame.range
                << " m"
                << ", status="
                << status_to_string(frame.status)
                << ", variance="
                << std::scientific << frame.variance
                << std::defaultfloat
                << std::endl;
            break;
        }

        case 'h':
            print_help();
            break;

        default:
            std::cout << "unknown command: " << key << std::endl;
            print_help();
            break;
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    const std::string model_path =
        (argc >= 2) ? argv[1] : kDefaultModelPath;

    const std::string config_path =
        (argc >= 3) ? argv[2] : kDefaultConfigPath;

    bool viewer_running = true;
    std::mutex mujoco_mutex;
    std::thread command_thread;

    try {
        auto world = std::make_shared<ExampleWorld>();
        world->loadModel(model_path);

        auto* model = world->getModel();
        auto* data = world->getData();

        if (model == nullptr || data == nullptr) {
            std::cerr << "ERROR: model/data is null" << std::endl;
            return EXIT_FAILURE;
        }

        const int qpos_addr = find_base_freejoint_qpos_addr(model);

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

        std::cout << "Hakoniwa Ultrasonic Sensor Example" << std::endl;
        std::cout << "model : " << model_path << std::endl;
        std::cout << "config: " << config_path << std::endl;
        std::cout << "site  : " << kSensorSiteName << std::endl;

        print_help();

        {
            std::lock_guard<std::mutex> lock(mujoco_mutex);
            print_pose(data, qpos_addr);
        }

        /*
         * macOS/Cocoa/GLFW expects window creation and event handling to run
         * on the main thread. Therefore:
         *
         *   - command_thread handles console input
         *   - main thread runs viewer_thread()
         */
        command_thread = std::thread(
            run_command_loop,
            std::ref(viewer_running),
            std::ref(mujoco_mutex),
            model,
            data,
            qpos_addr,
            std::ref(sensor));

        viewer_thread(
            model,
            data,
            viewer_running,
            mujoco_mutex);

        /*
         * If the viewer window was closed, stop the command loop.
         * The command loop may be blocked on std::cin, so do not join it here.
         * The process is about to exit, so detaching is acceptable for this
         * simple interactive example.
         */
        viewer_running = false;

        if (command_thread.joinable()) {
            command_thread.detach();
        }

        std::cout << "bye" << std::endl;
        return EXIT_SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;

        viewer_running = false;

        if (command_thread.joinable()) {
            command_thread.detach();
        }

        return EXIT_FAILURE;
    }
}