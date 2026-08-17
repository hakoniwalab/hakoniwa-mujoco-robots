#include "physics/physics_impl.hpp"
#include "sensors/imu/imu_sensor.hpp"
#include "tests/sensors/support/sensor_test_utils.hpp"

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using hako::robots::sensor::ImuFrame;
using hako::robots::sensor::ImuSensor;
using hako::robots::sensor::test::NearlyEqual;
using hako::robots::sensor::test::RepoRoot;
using hako::robots::physics::impl::WorldImpl;

constexpr double kEpsilon = 1.0e-6;

void ExpectMatches(const double actual, const double expected, const char* label)
{
    HAKO_TEST_EXPECT(
        NearlyEqual(actual, expected, kEpsilon),
        std::string(label) + ": expected " + std::to_string(expected) +
            ", actual " + std::to_string(actual));
}

void TestBodyVelocityUsesRegularBodyFrame()
{
    auto world = std::make_shared<WorldImpl>();
    world->loadModel(
        (RepoRoot() / "models/sensors/imu/rotated-inertial-frame-test.xml").string());

    auto* model = world->getModel();
    auto* data = world->getData();
    HAKO_TEST_EXPECT(model != nullptr, "model should not be null");
    HAKO_TEST_EXPECT(data != nullptr, "data should not be null");

    const int body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, "base_freejoint");
    HAKO_TEST_EXPECT(body_id >= 0, "base_link should exist");
    HAKO_TEST_EXPECT(joint_id >= 0, "base_freejoint should exist");
    HAKO_TEST_EXPECT(
        model->jnt_type[joint_id] == mjJNT_FREE,
        "base_freejoint should be a freejoint");

    const int qvel_addr = model->jnt_dofadr[joint_id];

    // Free-joint linear velocity is world-frame; the body itself has identity
    // orientation, so these values are also the expected regular body-frame
    // components. Free-joint angular velocity is already body-local.
    data->qvel[qvel_addr + 0] = 1.0;
    data->qvel[qvel_addr + 1] = -2.0;
    data->qvel[qvel_addr + 2] = 0.5;
    data->qvel[qvel_addr + 3] = 0.0;
    data->qvel[qvel_addr + 4] = 0.0;
    data->qvel[qvel_addr + 5] = 0.3;
    mj_forward(model, data);

    // Prove the fixture is meaningful: the body's inertial frame is rotated,
    // so mjOBJ_BODY and mjOBJ_XBODY report different local components.
    mjtNum inertial_frame_velocity[6] = {};
    mjtNum body_frame_velocity[6] = {};
    mj_objectVelocity(model, data, mjOBJ_BODY, body_id, inertial_frame_velocity, 1);
    mj_objectVelocity(model, data, mjOBJ_XBODY, body_id, body_frame_velocity, 1);

    const bool angular_frames_differ =
        !NearlyEqual(inertial_frame_velocity[0], body_frame_velocity[0], 1.0e-4) ||
        !NearlyEqual(inertial_frame_velocity[1], body_frame_velocity[1], 1.0e-4) ||
        !NearlyEqual(inertial_frame_velocity[2], body_frame_velocity[2], 1.0e-4);
    const bool linear_frames_differ =
        !NearlyEqual(inertial_frame_velocity[3], body_frame_velocity[3], 1.0e-4) ||
        !NearlyEqual(inertial_frame_velocity[4], body_frame_velocity[4], 1.0e-4) ||
        !NearlyEqual(inertial_frame_velocity[5], body_frame_velocity[5], 1.0e-4);
    HAKO_TEST_EXPECT(angular_frames_differ, "rotated inertial frame should change local angular components");
    HAKO_TEST_EXPECT(linear_frames_differ, "rotated inertial frame should change local linear components");

    const auto body = world->getRigidBody("base_link");
    const auto linear = body->GetBodyVelocity();
    const auto angular = body->GetBodyAngularVelocity();

    ExpectMatches(linear.x, body_frame_velocity[3], "body velocity x");
    ExpectMatches(linear.y, body_frame_velocity[4], "body velocity y");
    ExpectMatches(linear.z, body_frame_velocity[5], "body velocity z");
    ExpectMatches(angular.x, body_frame_velocity[0], "body angular velocity x");
    ExpectMatches(angular.y, body_frame_velocity[1], "body angular velocity y");
    ExpectMatches(angular.z, body_frame_velocity[2], "body angular velocity z");

    // This is the user-visible regression from #46: planar yaw belongs on z,
    // regardless of how the body's inertial frame is oriented.
    ExpectMatches(angular.x, 0.0, "yaw angular velocity x");
    ExpectMatches(angular.y, 0.0, "yaw angular velocity y");
    ExpectMatches(angular.z, 0.3, "yaw angular velocity z");

    ImuSensor imu(world);
    HAKO_TEST_EXPECT(
        imu.LoadConfig((RepoRoot() / "config/sensors/imu/tb3-imu.json").string()),
        "IMU config should load");

    ImuFrame frame {};
    imu.Build(frame);
    ExpectMatches(frame.angular_velocity.x, 0.0, "IMU angular_velocity.x");
    ExpectMatches(frame.angular_velocity.y, 0.0, "IMU angular_velocity.y");
    ExpectMatches(frame.angular_velocity.z, 0.3, "IMU angular_velocity.z");
}
}

int main()
{
    try {
        TestBodyVelocityUsesRegularBodyFrame();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "imu_body_frame_velocity_test passed" << std::endl;
    return EXIT_SUCCESS;
}
