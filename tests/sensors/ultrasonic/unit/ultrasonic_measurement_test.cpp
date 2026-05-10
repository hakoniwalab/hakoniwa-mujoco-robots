#include "physics.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "tests/sensors/support/sensor_test_utils.hpp"

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using hako::robots::sensor::test::NearlyEqual;
using hako::robots::sensor::test::RepoRoot;
using hako::robots::sensor::ultrasonic::UltrasonicFrame;
using hako::robots::sensor::ultrasonic::UltrasonicStatus;

constexpr double kEpsilon = 1.0e-6;

class TestWorld final : public hako::robots::physics::IWorld {
public:
    void loadModel(const std::string& model_file) override
    {
        char error[1024] = {0};
        model = mj_loadXML(model_file.c_str(), nullptr, error, sizeof(error));
        if (model == nullptr) {
            throw std::runtime_error(
                std::string("failed to load MuJoCo model: ") + model_file + "\n" + error);
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

int FindBaseFreejointQposAddr(const mjModel* model)
{
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, "base_freejoint");
    HAKO_TEST_EXPECT(joint_id >= 0, "base_freejoint should exist");
    HAKO_TEST_EXPECT(model->jnt_type[joint_id] == mjJNT_FREE, "base_freejoint should be a freejoint");
    return model->jnt_qposadr[joint_id];
}

void SetBasePosition(mjModel* model, mjData* data, int qpos_addr, double x, double y, double z)
{
    data->qpos[qpos_addr + 0] = x;
    data->qpos[qpos_addr + 1] = y;
    data->qpos[qpos_addr + 2] = z;

    // Keep the freejoint orientation fixed.
    data->qpos[qpos_addr + 3] = 1.0;
    data->qpos[qpos_addr + 4] = 0.0;
    data->qpos[qpos_addr + 5] = 0.0;
    data->qpos[qpos_addr + 6] = 0.0;

    mj_forward(model, data);
}

UltrasonicFrame Measure(hako::robots::sensor::ultrasonic::UltrasonicSensor& sensor)
{
    UltrasonicFrame frame {};
    sensor.Measure(frame);
    return frame;
}

void ExpectRangeOk(const UltrasonicFrame& frame, double expected_range, const char* label)
{
    HAKO_TEST_EXPECT(frame.status == UltrasonicStatus::OK, std::string(label) + ": expected OK status");
    HAKO_TEST_EXPECT(NearlyEqual(frame.range, expected_range, kEpsilon), std::string(label) + ": unexpected range");
    HAKO_TEST_EXPECT(NearlyEqual(frame.variance, 0.0, kEpsilon), std::string(label) + ": unexpected variance");
}

void TestDeterministicUltrasonicMeasurement()
{
    auto world = std::make_shared<TestWorld>();
    world->loadModel((RepoRoot() / "models/sensors/ultrasonic/ultrasonic-sensor-test.xml").string());

    auto* model = world->getModel();
    auto* data = world->getData();
    HAKO_TEST_EXPECT(model != nullptr, "model should not be null");
    HAKO_TEST_EXPECT(data != nullptr, "data should not be null");

    const int qpos_addr = FindBaseFreejointQposAddr(model);

    hako::robots::sensor::ultrasonic::UltrasonicSensor sensor(
        world,
        "front_ultrasonic_site",
        "base_footprint");

    const auto config_path = (RepoRoot() / "config/sensors/ultrasonic/lego-spike-distance-sensor.json").string();
    HAKO_TEST_EXPECT(sensor.LoadConfig(config_path), "ultrasonic config should load");

    SetBasePosition(model, data, qpos_addr, 0.0, 0.0, 0.1);
    ExpectRangeOk(Measure(sensor), 0.86, "initial front wall range");

    SetBasePosition(model, data, qpos_addr, 0.05, 0.0, 0.1);
    ExpectRangeOk(Measure(sensor), 0.81, "front wall range after x move");

    SetBasePosition(model, data, qpos_addr, 0.45, 0.30, 0.1);
    ExpectRangeOk(Measure(sensor), 0.18, "diagonal obstacle range");

    SetBasePosition(model, data, qpos_addr, 0.45, 0.55, 0.1);
    const auto no_hit_frame = Measure(sensor);
    HAKO_TEST_EXPECT(no_hit_frame.status == UltrasonicStatus::NO_HIT, "expected NO_HIT status");
    HAKO_TEST_EXPECT(NearlyEqual(no_hit_frame.range, 2.0, kEpsilon), "unexpected no-hit range");
    HAKO_TEST_EXPECT(NearlyEqual(no_hit_frame.variance, 0.0, kEpsilon), "unexpected no-hit variance");
}
}

int main()
{
    try {
        TestDeterministicUltrasonicMeasurement();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "ultrasonic_measurement_test passed" << std::endl;
    return EXIT_SUCCESS;
}
