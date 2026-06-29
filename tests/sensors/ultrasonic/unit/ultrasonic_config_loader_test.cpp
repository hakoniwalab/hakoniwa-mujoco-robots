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
using hako::robots::sensor::ultrasonic::RangeRadiationType;

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

void TestLegoSpikeDistanceSensorConfig()
{
    auto world = std::make_shared<TestWorld>();
    world->loadModel((RepoRoot() / "models/sensors/ultrasonic/ultrasonic-sensor-test.xml").string());

    hako::robots::sensor::ultrasonic::UltrasonicSensor sensor(
        world,
        "front_ultrasonic_site",
        "base_footprint");

    const auto path = (RepoRoot() / "config/sensors/ultrasonic/lego-spike-distance-sensor.json").string();
    const bool ok = sensor.LoadConfig(path);
    HAKO_TEST_EXPECT(ok, "lego-spike-distance-sensor.json should load");

    const auto& config = sensor.GetConfig();
    HAKO_TEST_EXPECT(config.frame_id == "spike_distance_sensor_link", "unexpected frame_id");
    HAKO_TEST_EXPECT(config.radiation_type == RangeRadiationType::ULTRASOUND, "unexpected radiation_type");
    HAKO_TEST_EXPECT(NearlyEqual(config.detection_distance.min, 0.05), "unexpected min range");
    HAKO_TEST_EXPECT(NearlyEqual(config.detection_distance.max, 2.0), "unexpected max range");
    HAKO_TEST_EXPECT(config.distance_accuracy.size() == 1U, "unexpected distance accuracy count");
    HAKO_TEST_EXPECT(NearlyEqual(config.distance_accuracy[0].stddev, 0.0), "unexpected stddev");
    HAKO_TEST_EXPECT(NearlyEqual(config.distance_accuracy[0].precision, 0.0), "unexpected precision");
    HAKO_TEST_EXPECT(config.distance_accuracy[0].noise_distribution == "none", "unexpected noise distribution");
    HAKO_TEST_EXPECT(NearlyEqual(config.cone.horizontal, 0.0), "unexpected horizontal FOV");
    HAKO_TEST_EXPECT(NearlyEqual(config.cone.vertical, 0.0), "unexpected vertical FOV");
    HAKO_TEST_EXPECT(config.cone.ray_count == 1, "unexpected ray count");
    HAKO_TEST_EXPECT(NearlyEqual(config.update_rate_hz, 100.0), "unexpected update rate");
    HAKO_TEST_EXPECT(config.mjcf_binding.source_site == "front_ultrasonic_site", "unexpected source site");
    HAKO_TEST_EXPECT(config.pdu_config.pdu_name == "range", "unexpected pdu name");
    HAKO_TEST_EXPECT(NearlyEqual(config.pdu_config.update_rate_hz, 100.0), "unexpected pdu update rate");
    HAKO_TEST_EXPECT(config.pdu_config.message_type == "sensor_msgs/Range", "unexpected pdu message type");
}
}

int main()
{
    try {
        TestLegoSpikeDistanceSensorConfig();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "ultrasonic_config_loader_test passed" << std::endl;
    return EXIT_SUCCESS;
}
