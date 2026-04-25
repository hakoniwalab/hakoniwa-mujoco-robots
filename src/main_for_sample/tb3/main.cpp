#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>

#include <mujoco/mujoco.h>

#include "mujoco_debug.hpp"
#if USE_VIEWER
#include "mujoco_viewer.hpp"
#endif

#include "actuator/actuator_impl.hpp"
#include "geometry_msgs/pdu_cpptype_Twist.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "hako_asset.h"
#include "hako_conductor.h"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/hako_msgs/pdu_cpptype_GameControllerOperation.hpp"
#include "hakoniwa/pdu/hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "physics/physics_impl.hpp"

namespace {
std::shared_ptr<hako::robots::physics::IWorld> world;
std::mutex data_mutex;
bool running_flag = true;

std::filesystem::path repo_root_path()
{
    const auto source_path = std::filesystem::path(__FILE__).lexically_normal();
    return source_path.parent_path().parent_path().parent_path().parent_path();
}

const std::string model_path =
    (repo_root_path() / "models/tb3/turtlebot3_burger_world.xml").string();
const std::string hako_config_path = (repo_root_path() / "config/tb3-pdudef-compact.json").string();
const std::string endpoint_config_path = (repo_root_path() / "config/endpoint/tb3_sim_endpoint.json").string();

std::string get_env_string(const char* name, const std::string& default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    return std::string(env);
}

double get_env_double(const char* name, double default_value)
{
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return default_value;
    }
    try {
        return std::stod(env);
    } catch (...) {
        return default_value;
    }
}

class Tb3Drive {
public:
    explicit Tb3Drive(std::shared_ptr<hako::robots::physics::IWorld> world)
        : world_(world)
        , base_(world->getRigidBody("base_link"))
        , base_scan_(world->getRigidBody("base_scan"))
        , left_motor_(world->getTorqueActuator("left_motor"))
        , right_motor_(world->getTorqueActuator("right_motor"))
    {
    }

    void set_torque(double left, double right)
    {
        left_motor_->SetTorque(left);
        right_motor_->SetTorque(right);
    }

    hako::robots::types::Position position() const
    {
        return base_->GetPosition();
    }

    hako::robots::types::Euler euler() const
    {
        return base_->GetEuler();
    }

    hako::robots::types::BodyVelocity body_velocity() const
    {
        return base_->GetBodyVelocity();
    }

    hako::robots::types::Position base_scan_position() const
    {
        return base_scan_->GetPosition();
    }

    hako::robots::types::Euler base_scan_euler() const
    {
        return base_scan_->GetEuler();
    }

    void fill_laser_scan(HakoCpp_LaserScan& scan, int ray_count, double angle_min, double angle_max, double range_min, double range_max, double yaw_offset, double origin_offset) const
    {
        auto* model = world_->getModel();
        auto* data = world_->getData();
        const int base_footprint_id = mj_name2id(model, mjOBJ_BODY, "base_footprint");
        const int base_scan_id = mj_name2id(model, mjOBJ_BODY, "base_scan");
        if (base_scan_id < 0) {
            return;
        }

        const mjtNum* pos = &data->xpos[3 * base_scan_id];
        const mjtNum* mat = &data->xmat[9 * base_scan_id];

        static int debug_counter = 0;
        if ((debug_counter++ % 20) == 0) {
            const mjtNum dir_world_x[3] = {1.0, 0.0, 0.0};
            int geomid = -1;
            mjtNum normal[3] = {0.0, 0.0, 0.0};
            mjtNum dist = mj_ray(
                model,
                data,
                pos,
                dir_world_x,
                nullptr,
                1,
                base_footprint_id,
                &geomid,
                normal);
            std::cout << "[LIDAR_DEBUG] world+X ray: dist=" << dist
                      << " geomid=" << geomid << std::endl;
            std::cout << "[LIDAR_DEBUG] scan_xaxis=("
                      << mat[0] << "," << mat[1] << "," << mat[2] << ")"
                      << " scan_yaxis=("
                      << mat[3] << "," << mat[4] << "," << mat[5] << ")"
                      << std::endl;
        }

        // MuJoCo xmat is row-major 3x3 body orientation in world coordinates.
        const mjtNum xaxis[3] = {mat[0], mat[1], mat[2]};
        const mjtNum yaxis[3] = {mat[3], mat[4], mat[5]};

        std::vector<mjtNum> dirs(static_cast<size_t>(ray_count) * 3, 0.0);
        std::vector<mjtNum> dist(static_cast<size_t>(ray_count), -1.0);
        std::vector<mjtNum> ray_origin(3, 0.0);
        for (int i = 0; i < ray_count; ++i) {
            const double t = (ray_count == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(ray_count - 1);
            const double angle = angle_min + (angle_max - angle_min) * t + yaw_offset;
            dirs[3 * i + 0] = std::cos(angle) * xaxis[0] + std::sin(angle) * yaxis[0];
            dirs[3 * i + 1] = std::cos(angle) * xaxis[1] + std::sin(angle) * yaxis[1];
            dirs[3 * i + 2] = std::cos(angle) * xaxis[2] + std::sin(angle) * yaxis[2];
        }
        for (int i = 0; i < ray_count; ++i) {
            ray_origin[0] = pos[0] + dirs[3 * i + 0] * origin_offset;
            ray_origin[1] = pos[1] + dirs[3 * i + 1] * origin_offset;
            ray_origin[2] = pos[2] + dirs[3 * i + 2] * origin_offset;
            dist[static_cast<size_t>(i)] = mj_ray(
                model,
                data,
                ray_origin.data(),
                &dirs[3 * i],
                nullptr,
                1,
                base_footprint_id,
                nullptr,
                nullptr);
        }

        scan.angle_min = static_cast<float>(angle_min);
        scan.angle_max = static_cast<float>(angle_max);
        scan.angle_increment = static_cast<float>((ray_count > 1) ? ((angle_max - angle_min) / static_cast<double>(ray_count - 1)) : 0.0);
        scan.time_increment = 0.0F;
        scan.scan_time = static_cast<float>(model->opt.timestep);
        scan.range_min = static_cast<float>(range_min);
        scan.range_max = static_cast<float>(range_max);
        scan.ranges.assign(static_cast<size_t>(ray_count), static_cast<float>(range_max));
        scan.intensities.assign(static_cast<size_t>(ray_count), 0.0F);
        for (int i = 0; i < ray_count; ++i) {
            if (dist[static_cast<size_t>(i)] >= range_min && dist[static_cast<size_t>(i)] <= range_max) {
                scan.ranges[static_cast<size_t>(i)] = static_cast<float>(dist[static_cast<size_t>(i)]);
            }
        }
    }

private:
    std::shared_ptr<hako::robots::physics::IWorld> world_;
    std::shared_ptr<hako::robots::physics::IRigidBody> base_;
    std::shared_ptr<hako::robots::physics::IRigidBody> base_scan_;
    std::shared_ptr<hako::robots::actuator::ITorqueActuator> left_motor_;
    std::shared_ptr<hako::robots::actuator::ITorqueActuator> right_motor_;
};

struct Tb3CommandState {
    HakoCpp_GameControllerOperation gamepad {};
    bool has_input {false};
};

static int my_on_initialize(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

static int my_on_reset(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;

    const double sim_timestep = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);
    const std::string endpoint_path = get_env_string("HAKO_TB3_ENDPOINT_CONFIG_PATH", endpoint_config_path);
    const double drive_gain = get_env_double("HAKO_TB3_DRIVE_GAIN", 0.1);
    const double turn_gain = get_env_double("HAKO_TB3_TURN_GAIN", 0.15);
    const double max_torque = get_env_double("HAKO_TB3_MAX_TORQUE", 1.0);
    const int lidar_ray_count = static_cast<int>(get_env_double("HAKO_TB3_LIDAR_RAY_COUNT", 181));
    const double lidar_angle_min = get_env_double("HAKO_TB3_LIDAR_ANGLE_MIN", -1.57079632679);
    const double lidar_angle_max = get_env_double("HAKO_TB3_LIDAR_ANGLE_MAX", 1.57079632679);
    const double lidar_range_min = get_env_double("HAKO_TB3_LIDAR_RANGE_MIN", 0.05);
    const double lidar_range_max = get_env_double("HAKO_TB3_LIDAR_RANGE_MAX", 5.0);
    const double lidar_period_sec = get_env_double("HAKO_TB3_LIDAR_PERIOD_SEC", 0.1);
    const double lidar_spin_rate = get_env_double("HAKO_TB3_LIDAR_SPIN_RATE", 6.28318530718);
    const double lidar_origin_offset = get_env_double("HAKO_TB3_LIDAR_ORIGIN_OFFSET", 0.05);
    const std::string endpoint_name = get_env_string("HAKO_TB3_ENDPOINT_NAME", "tb3_sim_endpoint");

    Tb3Drive tb3(world);
    hakoniwa::pdu::Endpoint endpoint(endpoint_name, HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    endpoint.open(endpoint_path);

    const hakoniwa::pdu::PduKey gamepad_key {"TB3", "hako_cmd_game"};
    const hakoniwa::pdu::PduKey base_pos_key {"TB3", "base_link_pos"};
    const hakoniwa::pdu::PduKey base_scan_pos_key {"TB3", "base_scan_pos"};
    const hakoniwa::pdu::PduKey laser_scan_key {"TB3", "laser_scan"};

    endpoint.start();
    endpoint.post_start();

    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> base_pos_ep(endpoint, base_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> base_scan_pos_ep(endpoint, base_scan_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_LaserScan, hako::pdu::msgs::sensor_msgs::LaserScan> laser_scan_ep(endpoint, laser_scan_key);
    hako::pdu::msgs::hako_msgs::GameControllerOperation gamepad_conv;
    std::vector<std::byte> gamepad_buf(endpoint.get_pdu_size(gamepad_key), std::byte{0});
    HakoCpp_LaserScan laser_scan {};
    double lidar_elapsed_sec = lidar_period_sec;
    double lidar_yaw_offset = 0.0;

    int step = 0;
    Tb3CommandState command_state {};
    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            HakoCpp_GameControllerOperation latest {};
            size_t gamepad_received = 0;
            const auto gamepad_rc = endpoint.recv(gamepad_key, std::span<std::byte>(gamepad_buf.data(), gamepad_buf.size()), gamepad_received);
            if ((gamepad_rc == HAKO_PDU_ERR_OK) && (gamepad_received > 0)) {
                if (gamepad_conv.pdu2cpp(reinterpret_cast<char*>(gamepad_buf.data()), latest)) {
                    command_state.gamepad = latest;
                    command_state.has_input = true;
                }
                else {
                    continue;
                }
            }

            double left_torque = 0.0;
            double right_torque = 0.0;
            if (command_state.has_input && command_state.gamepad.axis.size() >= 4) {
                const double turn = -1 * std::clamp(static_cast<double>(command_state.gamepad.axis[0]), -1.0, 1.0);
                const double forward = std::clamp(-static_cast<double>(command_state.gamepad.axis[3]), -1.0, 1.0);
                left_torque = std::clamp((forward - turn * turn_gain) * drive_gain, -max_torque, max_torque);
                right_torque = std::clamp((forward + turn * turn_gain) * drive_gain, -max_torque, max_torque);
            }
            tb3.set_torque(left_torque, right_torque);
            world->advanceTimeStep();

            HakoCpp_Twist base_pos {};
            const auto pos = tb3.position();
            const auto euler = tb3.euler();
            const auto body_vel = tb3.body_velocity();
            base_pos.linear.x = pos.x;
            base_pos.linear.y = pos.y;
            base_pos.linear.z = pos.z;
            base_pos.angular.x = euler.x;
            base_pos.angular.y = euler.y;
            base_pos.angular.z = euler.z;
            (void)base_pos_ep.send(base_pos);

            HakoCpp_Twist base_scan_pos {};
            const auto scan_pos = tb3.base_scan_position();
            const auto scan_euler = tb3.base_scan_euler();
            base_scan_pos.linear.x = scan_pos.x;
            base_scan_pos.linear.y = scan_pos.y;
            base_scan_pos.linear.z = scan_pos.z;
            base_scan_pos.angular.x = scan_euler.x;
            base_scan_pos.angular.y = scan_euler.y;
            base_scan_pos.angular.z = scan_euler.z + lidar_yaw_offset;
            (void)base_scan_pos_ep.send(base_scan_pos);

            lidar_elapsed_sec += sim_timestep;
            if (lidar_elapsed_sec >= lidar_period_sec) {
                lidar_yaw_offset += lidar_spin_rate * lidar_elapsed_sec;
                lidar_yaw_offset = std::atan2(std::sin(lidar_yaw_offset), std::cos(lidar_yaw_offset));
                tb3.fill_laser_scan(
                    laser_scan,
                    std::max(1, lidar_ray_count),
                    lidar_angle_min,
                    lidar_angle_max,
                    lidar_range_min,
                    lidar_range_max,
                    lidar_yaw_offset,
                    lidar_origin_offset);
                (void)laser_scan_ep.send(laser_scan);
                lidar_elapsed_sec = 0.0;
            }

            if ((step % 500) == 0) {
                float lidar_min = std::numeric_limits<float>::infinity();
                float lidar_max = 0.0F;
                int lidar_hits = 0;
                for (float value : laser_scan.ranges) {
                    if (value >= laser_scan.range_min && value < laser_scan.range_max) {
                        lidar_min = std::min(lidar_min, value);
                        lidar_max = std::max(lidar_max, value);
                        ++lidar_hits;
                    }
                }
                std::cout << "[TB3] step=" << step
                          << " pos=(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                          << " body_vx=" << body_vel.x
                          << " torque=(" << left_torque << ", " << right_torque << ")"
                          << " lidar_center=" << (laser_scan.ranges.empty() ? 0.0F : laser_scan.ranges[laser_scan.ranges.size() / 2])
                          << " lidar_hits=" << lidar_hits
                          << " lidar_min=" << (std::isfinite(lidar_min) ? lidar_min : -1.0F)
                          << " lidar_max=" << lidar_max
                          << std::endl;
            }
            ++step;
        }

        hako_asset_usleep(delta_time_usec);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        const double sleep_time = sim_timestep - elapsed.count();
        if (sleep_time > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    (void)endpoint.stop();
    endpoint.close();
    return 0;
}

static hako_asset_callbacks_t my_callback;

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    my_callback.on_initialize = my_on_initialize;
    my_callback.on_simulation_step = nullptr;
    my_callback.on_manual_timing_control = my_manual_timing_control;
    my_callback.on_reset = my_on_reset;

    const std::string asset_name = get_env_string("HAKO_ASSET_NAME", "tb3_sim");
    const std::string config_path = get_env_string("HAKO_ASSET_CONFIG_PATH", hako_config_path);
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(world->getModel()->opt.timestep * 1e6);

    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(asset_name.c_str(), config_path.c_str(), &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_register() returns " << ret << std::endl;
        return;
    }

    ret = hako_asset_start();
    if (ret != 0) {
        std::cerr << "ERROR: hako_asset_start() returns " << ret << std::endl;
        return;
    }
}
} // namespace

int main(int argc, const char* argv[])
{
    (void)argc;
    (void)argv;

    std::cout << "[INFO] Creating TB3 world and loading model..." << std::endl;
    world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
    try {
        world->loadModel(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load TB3 model: " << e.what() << std::endl;
        return 1;
    }

    std::thread sim_thread(simulation_thread, world);

#if USE_VIEWER
    viewer_thread(world->getModel(), world->getData(), std::ref(running_flag), std::ref(data_mutex));
#else
    while (running_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    running_flag = false;
    sim_thread.join();
    std::cout << "[INFO] TB3 simulation completed successfully." << std::endl;
    return 0;
}
