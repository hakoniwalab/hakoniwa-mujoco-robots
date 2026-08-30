#include "hako_msgs/pdu_cpptype_GameControllerOperation.hpp"
#include "hako_msgs/pdu_ctype_GameControllerOperation.h"
#include "config/asset_manifest.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "robots/ackermann/ackermann_kinematics.hpp"
#include "robots/ackermann/ackermann_runtime_config.hpp"
#include "runtime/hakoniwa_asset_lifecycle.hpp"
#include "viewer/mujoco_viewer.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr const char* kDefaultManifestPath =
    "recipes/generic_ackermann/asset-manifest.json";
std::atomic_bool running {true};
std::atomic_bool viewer_running {true};
std::mutex world_mutex;
std::shared_ptr<hako::robots::physics::impl::WorldImpl> world;
std::vector<std::shared_ptr<hako::robots::actuator::IJointActuator>> actuators;
hako::robots::config::AssetManifest asset_manifest;
hako::robots::ackermann::RuntimeConfig runtime_config;
hakoniwa::pdu::Endpoint* gamepad_endpoint {nullptr};
std::vector<std::byte> gamepad_buffer;
HakoCpp_GameControllerOperation latest_gamepad {};
double current_linear_velocity = 0.0;
double current_steering_angle = 0.0;

double apply_deadzone(double value)
{
    const double clamped = std::clamp(value, -1.0, 1.0);
    if (std::abs(clamped) <= runtime_config.control.deadzone) {
        return 0.0;
    }
    const double sign = clamped < 0.0 ? -1.0 : 1.0;
    return sign * (std::abs(clamped) - runtime_config.control.deadzone) /
        (1.0 - runtime_config.control.deadzone);
}

struct DriveCommand {
    double linear_velocity {0.0};
    double steering_angle {0.0};
};

DriveCommand gamepad_to_command(
    const HakoCpp_GameControllerOperation& gamepad)
{
    DriveCommand command {};
    if (gamepad.axis.size() >= 4) {
        command.steering_angle =
            -apply_deadzone(gamepad.axis[0]) * runtime_config.geometry.max_steering_angle;
        command.linear_velocity =
            -apply_deadzone(gamepad.axis[3]) * runtime_config.control.max_linear_velocity;
    }
    return command;
}

double move_toward(double current, double target, double max_delta)
{
    return current + std::clamp(target - current, -max_delta, max_delta);
}

void apply_targets(const hako::robots::ackermann::Targets& targets)
{
    actuators[0]->SetTarget(targets.front_left_steering);
    actuators[1]->SetTarget(targets.front_right_steering);
    actuators[2]->SetTarget(targets.rear_left_wheel_velocity);
    actuators[3]->SetTarget(targets.rear_right_wheel_velocity);
}

bool recv_gamepad(HakoCpp_GameControllerOperation& out)
{
    if (gamepad_endpoint == nullptr || gamepad_buffer.empty()) {
        return false;
    }
    size_t received_size = 0;
    const auto rc = gamepad_endpoint->recv(
        hakoniwa::pdu::PduKey {
            runtime_config.bindings.asset_name,
            runtime_config.bindings.pdu_name},
        std::span<std::byte>(gamepad_buffer.data(), gamepad_buffer.size()),
        received_size);
    if (rc != HAKO_PDU_ERR_OK || received_size < gamepad_buffer.size()) {
        return false;
    }
    void* base_ptr = hako_get_base_ptr_pdu(gamepad_buffer.data());
    if (base_ptr == nullptr) {
        // The SHM cache is zero-filled until the first controller sample arrives.
        return false;
    }
    const auto* raw = static_cast<const Hako_GameControllerOperation*>(base_ptr);
    std::copy(std::begin(raw->axis), std::end(raw->axis), out.axis.begin());
    std::copy(std::begin(raw->button), std::end(raw->button), out.button.begin());
    return true;
}

int base_qpos_address()
{
    const int joint = mj_name2id(
        world->getModel(), mjOBJ_JOINT,
        runtime_config.bindings.base_freejoint.c_str());
    return joint < 0 ? -1 : world->getModel()->jnt_qposadr[joint];
}

int joint_qvel_address(const char* name)
{
    const int joint = mj_name2id(world->getModel(), mjOBJ_JOINT, name);
    return joint < 0 ? -1 : world->getModel()->jnt_dofadr[joint];
}

int joint_qpos_address(const char* name)
{
    const int joint = mj_name2id(world->getModel(), mjOBJ_JOINT, name);
    return joint < 0 ? -1 : world->getModel()->jnt_qposadr[joint];
}

double base_yaw(int qpos)
{
    const double w = world->getData()->qpos[qpos + 3];
    const double x = world->getData()->qpos[qpos + 4];
    const double y = world->getData()->qpos[qpos + 5];
    const double z = world->getData()->qpos[qpos + 6];
    return std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
}

bool load_model_and_actuators(const std::string& manifest_path)
{
    std::string manifest_error;
    if (!hako::robots::config::LoadAssetManifestFromJson(
            manifest_path, asset_manifest, &manifest_error)) {
        std::cerr << "[ERROR] Failed to load asset manifest: "
                  << manifest_error << std::endl;
        return false;
    }
    if (asset_manifest.runtime_config.empty() ||
        !hako::robots::ackermann::LoadRuntimeConfigFromJson(
            asset_manifest.runtime_config, runtime_config, &manifest_error)) {
        std::cerr << "[ERROR] Failed to load Ackermann runtime config: "
                  << manifest_error << std::endl;
        return false;
    }
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(asset_manifest.model);
    } catch (const std::exception& exception) {
        std::cerr << "[ERROR] Failed to load model: " << exception.what() << std::endl;
        return false;
    }
    for (const auto& component_id : runtime_config.bindings.component_ids) {
        const std::string config = asset_manifest.ComponentConfig(component_id);
        if (config.empty()) {
            std::cerr << "[ERROR] Actuator component missing from manifest: "
                      << component_id << std::endl;
            return false;
        }
        auto actuator = world->createJointActuator();
        if (!actuator->LoadConfig(config)) {
            std::cerr << "[ERROR] Failed to load actuator: " << config << std::endl;
            return false;
        }
        actuators.push_back(std::move(actuator));
    }
    return true;
}

int run_smoke()
{
    const int qpos = base_qpos_address();
    if (qpos < 0) {
        std::cerr << "[ERROR] configured base freejoint not found" << std::endl;
        return 1;
    }
    const int steps = std::max(
        1, static_cast<int>(std::ceil(
            runtime_config.smoke.duration_sec / world->getModel()->opt.timestep)));
    const auto stopped = hako::robots::ackermann::ToJointTargets(
        0.0, 0.0, runtime_config.geometry);
    for (int step = 0; step < steps; ++step) {
        apply_targets(stopped);
        world->advanceTimeStep();
    }
    const double idle_x = world->getData()->qpos[qpos];
    const double idle_y = world->getData()->qpos[qpos + 1];
    const double idle_distance = std::hypot(idle_x, idle_y);
    std::cout << std::fixed << std::setprecision(3)
              << "idle base=(" << idle_x << ", " << idle_y << ")" << std::endl;
    if (idle_distance > runtime_config.smoke.idle_max_horizontal_drift_m ||
        !std::isfinite(idle_distance)) {
        std::cerr << "[ERROR] stationary Ackermann smoke failed" << std::endl;
        return 1;
    }

    mj_resetData(world->getModel(), world->getData());
    mj_forward(world->getModel(), world->getData());
    const auto straight = hako::robots::ackermann::ToJointTargets(
        runtime_config.smoke.straight_speed_m_s, 0.0, runtime_config.geometry);
    for (int step = 0; step < steps; ++step) {
        apply_targets(straight);
        world->advanceTimeStep();
    }
    const double x = world->getData()->qpos[qpos];
    const int rear_left_qvel = joint_qvel_address(
        runtime_config.bindings.drive_left_joint.c_str());
    const int rear_right_qvel = joint_qvel_address(
        runtime_config.bindings.drive_right_joint.c_str());
    if (rear_left_qvel < 0 || rear_right_qvel < 0) {
        std::cerr << "[ERROR] configured drive joint not found" << std::endl;
        return 1;
    }
    std::cout << std::fixed << std::setprecision(3)
              << "smoke base.x=" << x
              << " time=" << world->getData()->time
              << " rear_qvel=(" << world->getData()->qvel[rear_left_qvel]
              << ", " << world->getData()->qvel[rear_right_qvel] << ")"
              << std::endl;
    const double straight_minimum =
        runtime_config.smoke.straight_speed_m_s *
        runtime_config.smoke.duration_sec *
        runtime_config.smoke.straight_min_progress_ratio;
    if (x < straight_minimum || !std::isfinite(x)) {
        std::cerr << "[ERROR] forward Ackermann smoke failed" << std::endl;
        return 1;
    }

    mj_resetData(world->getModel(), world->getData());
    mj_forward(world->getModel(), world->getData());
    const auto turn = hako::robots::ackermann::ToJointTargets(
        runtime_config.smoke.turn_speed_m_s,
        runtime_config.smoke.turn_steering_rad,
        runtime_config.geometry);
    for (int step = 0; step < steps; ++step) {
        apply_targets(turn);
        world->advanceTimeStep();
    }
    const double turn_x = world->getData()->qpos[qpos];
    const double turn_y = world->getData()->qpos[qpos + 1];
    const int front_left_qpos = joint_qpos_address(
        runtime_config.bindings.steering_left_joint.c_str());
    const int front_right_qpos = joint_qpos_address(
        runtime_config.bindings.steering_right_joint.c_str());
    const int turn_rear_left_qvel = joint_qvel_address(
        runtime_config.bindings.drive_left_joint.c_str());
    const int turn_rear_right_qvel = joint_qvel_address(
        runtime_config.bindings.drive_right_joint.c_str());
    const int front_left_actuator = mj_name2id(
        world->getModel(), mjOBJ_ACTUATOR,
        runtime_config.bindings.steering_left_actuator.c_str());
    const int front_right_actuator = mj_name2id(
        world->getModel(), mjOBJ_ACTUATOR,
        runtime_config.bindings.steering_right_actuator.c_str());
    if (front_left_qpos < 0 || front_right_qpos < 0 ||
        turn_rear_left_qvel < 0 || turn_rear_right_qvel < 0 ||
        front_left_actuator < 0 || front_right_actuator < 0) {
        std::cerr << "[ERROR] configured Ackermann diagnostic binding not found" << std::endl;
        return 1;
    }
    const double turn_yaw = base_yaw(qpos);
    const double actual_front_left = world->getData()->qpos[front_left_qpos];
    const double actual_front_right = world->getData()->qpos[front_right_qpos];
    std::cout << "turn base=(" << turn_x << ", " << turn_y << ")"
              << " yaw=" << turn_yaw
              << " steer=(" << turn.front_left_steering << ", "
              << turn.front_right_steering << ")"
              << " actual_steer=(" << actual_front_left
              << ", " << actual_front_right << ")"
              << " actual_rear_qvel=(" << world->getData()->qvel[turn_rear_left_qvel]
              << ", " << world->getData()->qvel[turn_rear_right_qvel] << ")"
              << " steer_ctrl=(" << world->getData()->ctrl[front_left_actuator]
              << ", " << world->getData()->ctrl[front_right_actuator] << ")"
              << " steer_force=(" << world->getData()->actuator_force[front_left_actuator]
              << ", " << world->getData()->actuator_force[front_right_actuator] << ")"
              << std::endl;
    const bool steering_tracks =
        std::abs(actual_front_left - turn.front_left_steering) <
            runtime_config.smoke.steering_tracking_error_rad
        && std::abs(actual_front_right - turn.front_right_steering) <
            runtime_config.smoke.steering_tracking_error_rad;
    if (std::hypot(turn_x, turn_y) < runtime_config.smoke.turn_min_displacement_m ||
        std::abs(turn_yaw) < runtime_config.smoke.turn_min_abs_yaw_rad
        || !steering_tracks || !std::isfinite(turn_y)) {
        std::cerr << "[ERROR] steering Ackermann smoke failed" << std::endl;
        return 1;
    }
    std::cout << "smoke ok" << std::endl;
    return 0;
}

int manual_timing(hakoniwa::pdu::Endpoint&)
{
    const hako_time_t delta_usec = static_cast<hako_time_t>(world->getModel()->opt.timestep * 1.0e6);
    const int realtime_sync_steps = runtime_config.control.realtime_sync_steps;
    const auto realtime_sync_period = std::chrono::microseconds(
        delta_usec * realtime_sync_steps);
    const double timestep_sec = world->getModel()->opt.timestep;
    auto next_realtime_sync = std::chrono::steady_clock::now() + realtime_sync_period;
    const int qpos = base_qpos_address();
    int step = 0;
    while (running.load()) {
        {
            // Keep the Viewer lock limited to state access and one physics
            // step.  Real-time and Hakoniwa waits must not starve rendering.
            std::lock_guard<std::mutex> lock(world_mutex);
            HakoCpp_GameControllerOperation incoming {};
            if (recv_gamepad(incoming)) {
                latest_gamepad = incoming;
            }
            const auto command = gamepad_to_command(latest_gamepad);
            current_linear_velocity = move_toward(
                current_linear_velocity,
                command.linear_velocity,
                runtime_config.control.max_linear_acceleration * timestep_sec);
            current_steering_angle = move_toward(
                current_steering_angle,
                command.steering_angle,
                runtime_config.control.max_steering_rate * timestep_sec);
            const auto targets = hako::robots::ackermann::ToJointTargets(
                current_linear_velocity, current_steering_angle,
                runtime_config.geometry);
            apply_targets(targets);
            world->advanceTimeStep();
            if ((step % 250) == 0 && qpos >= 0) {
                std::cout << std::fixed << std::setprecision(3)
                          << "time=" << world->getData()->time
                          << " base=(" << world->getData()->qpos[qpos] << ", "
                          << world->getData()->qpos[qpos + 1] << ")"
                          << " steer=(" << targets.front_left_steering << ", "
                          << targets.front_right_steering << ")"
                          << " rear=(" << targets.rear_left_wheel_velocity << ", "
                          << targets.rear_right_wheel_velocity << ")" << std::endl;
            }
        }
        ++step;
        hako_asset_usleep(delta_usec);
        if ((step % realtime_sync_steps) == 0) {
            std::this_thread::sleep_until(next_realtime_sync);
            const auto now = std::chrono::steady_clock::now();
            do {
                next_realtime_sync += realtime_sync_period;
            } while (next_realtime_sync <= now);
        }
    }
    return 0;
}

int reset_world()
{
    std::lock_guard<std::mutex> lock(world_mutex);
    mj_resetData(world->getModel(), world->getData());
    mj_forward(world->getModel(), world->getData());
    latest_gamepad = {};
    current_linear_velocity = 0.0;
    current_steering_angle = 0.0;
    return 0;
}
}

int main(int argc, char** argv)
{
    bool smoke_only = false;
    bool view_model_only = false;
    bool no_viewer = false;
    std::string manifest_path = kDefaultManifestPath;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--smoke") {
            smoke_only = true;
        } else if (argument == "--view-model") {
            view_model_only = true;
        } else if (argument == "--no-viewer") {
            no_viewer = true;
        } else if (argument == "--manifest" && index + 1 < argc) {
            manifest_path = argv[++index];
        } else {
            std::cerr << "[ERROR] Unknown or incomplete argument: "
                      << argument << std::endl;
            return 2;
        }
    }
    if (!load_model_and_actuators(manifest_path)) {
        return 1;
    }
    if (smoke_only) {
        return run_smoke();
    }
    if (view_model_only) {
        MujocoRenderRuntime viewer(
            world->getModel(), world->getData(), viewer_running, world_mutex,
            MujocoRenderWindowMode::Visible);
        viewer.Run();
        return 0;
    }

    const hako_time_t delta_usec = static_cast<hako_time_t>(world->getModel()->opt.timestep * 1.0e6);
    hako::robots::runtime::HakoniwaAssetLifecycle lifecycle({
        runtime_config.bindings.endpoint_name,
        asset_manifest.endpoint,
        runtime_config.bindings.asset_name,
        asset_manifest.pdu_def,
        delta_usec,
        HAKO_ASSET_MODEL_PLANT,
    });
    std::string error;
    if (!lifecycle.OpenEndpoint(&error)) {
        std::cerr << "[ERROR] " << error << std::endl;
        return 1;
    }
    gamepad_endpoint = &lifecycle.Endpoint();
    gamepad_buffer.assign(
        gamepad_endpoint->get_pdu_size(hakoniwa::pdu::PduKey {
            runtime_config.bindings.asset_name,
            runtime_config.bindings.pdu_name}),
        std::byte {0});

    int asset_result = 0;
    std::thread asset_thread([&]() {
        if (!lifecycle.RegisterAndRunAssetNoWait(
                manual_timing,
                []() { return running.load() ? 0 : 1; },
                reset_world,
                &error)) {
            std::cerr << "[ERROR] " << error << std::endl;
            asset_result = 1;
        }
        running.store(false);
        viewer_running.store(false);
    });

    if (!no_viewer) {
        try {
            MujocoRenderRuntime viewer(
                world->getModel(), world->getData(), viewer_running, world_mutex,
                MujocoRenderWindowMode::Visible);
            viewer.Run();
        } catch (const std::exception& exception) {
            std::cerr << "[ERROR] Viewer failed: " << exception.what() << std::endl;
            asset_result = 1;
        }
    } else {
        while (running.load()) {
            hako_asset_usleep(100000);
        }
    }

    running.store(false);
    viewer_running.store(false);
    if (asset_thread.joinable()) {
        asset_thread.join();
    }
    gamepad_endpoint = nullptr;
    gamepad_buffer.clear();
    lifecycle.StopAndClose();
    return asset_result;
}
