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
#include "sensors/lidar/lidar_2d_sensor.hpp"

namespace {
std::shared_ptr<hako::robots::physics::IWorld> world;
std::mutex data_mutex;
bool running_flag = true;
std::string lidar_config_override_path;

std::filesystem::path repo_root_path()
{
    const auto source_path = std::filesystem::path(__FILE__).lexically_normal();
    return source_path.parent_path().parent_path().parent_path().parent_path();
}

const std::string model_path =
    (repo_root_path() / "models/tb3/turtlebot3_burger_world.xml").string();
const std::string hako_config_path = (repo_root_path() / "config/tb3-pdudef-compact.json").string();
const std::string endpoint_config_path = (repo_root_path() / "config/endpoint/tb3_sim_endpoint.json").string();
const std::string lidar_config_path =
    (repo_root_path() / "config/sensors/lds-02.json").string();

std::string resolve_repo_path(const std::string& path)
{
    const std::filesystem::path candidate(path);
    if (candidate.is_absolute()) {
        return candidate.lexically_normal().string();
    }
    return (repo_root_path() / candidate).lexically_normal().string();
}

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

    hako::robots::types::Position position() const { return base_->GetPosition(); }
    hako::robots::types::Euler euler() const { return base_->GetEuler(); }
    hako::robots::types::BodyVelocity body_velocity() const { return base_->GetBodyVelocity(); }
    hako::robots::types::Position base_scan_position() const { return base_scan_->GetPosition(); }
    hako::robots::types::Euler base_scan_euler() const { return base_scan_->GetEuler(); }

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

HakoCpp_LaserScan to_hako_scan(const hako::robots::sensor::LaserScanFrame& frame)
{
    HakoCpp_LaserScan out {};
    out.angle_min = frame.angle_min;
    out.angle_max = frame.angle_max;
    out.angle_increment = frame.angle_increment;
    out.time_increment = frame.time_increment;
    out.scan_time = frame.scan_time;
    out.range_min = frame.range_min;
    out.range_max = frame.range_max;
    out.ranges = frame.ranges;
    out.intensities = frame.intensities;
    return out;
}

static int my_on_initialize(hako_asset_context_t* context) { (void)context; return 0; }
static int my_on_reset(hako_asset_context_t* context)      { (void)context; return 0; }

static int my_manual_timing_control(hako_asset_context_t* context)
{
    (void)context;

    const double sim_timestep  = world->getModel()->opt.timestep;
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(sim_timestep * 1e6);

    const std::string endpoint_path = get_env_string("HAKO_TB3_ENDPOINT_CONFIG_PATH", endpoint_config_path);
    std::string lidar_config = get_env_string("HAKO_TB3_LIDAR_CONFIG_PATH", lidar_config_path);
    if (!lidar_config_override_path.empty()) {
        lidar_config = lidar_config_override_path;
    }
    lidar_config = resolve_repo_path(lidar_config);
    const double drive_gain  = get_env_double("HAKO_TB3_DRIVE_GAIN",  0.1);
    const double turn_gain   = get_env_double("HAKO_TB3_TURN_GAIN",   0.15);
    const double max_torque  = get_env_double("HAKO_TB3_MAX_TORQUE",  1.0);
    const double lidar_yaw_bias_deg =
        get_env_double("HAKO_TB3_LIDAR_YAW_BIAS_DEG", 0.0);
    const double lidar_origin_offset = get_env_double("HAKO_TB3_LIDAR_ORIGIN_OFFSET", 0.0);
    const std::string endpoint_name = get_env_string("HAKO_TB3_ENDPOINT_NAME", "tb3_sim_endpoint");

    Tb3Drive tb3(world);
    hakoniwa::pdu::Endpoint endpoint(endpoint_name, HAKO_PDU_ENDPOINT_DIRECTION_INOUT);
    endpoint.open(endpoint_path);

    const hakoniwa::pdu::PduKey gamepad_key      {"TB3", "hako_cmd_game"};
    const hakoniwa::pdu::PduKey base_pos_key     {"TB3", "base_link_pos"};
    const hakoniwa::pdu::PduKey base_scan_pos_key{"TB3", "base_scan_pos"};
    const hakoniwa::pdu::PduKey laser_scan_key   {"TB3", "laser_scan"};

    endpoint.start();
    endpoint.post_start();

    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_pos_ep(endpoint, base_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_scan_pos_ep(endpoint, base_scan_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_LaserScan, hako::pdu::msgs::sensor_msgs::LaserScan>
        laser_scan_ep(endpoint, laser_scan_key);

    hako::pdu::msgs::hako_msgs::GameControllerOperation gamepad_conv;
    std::vector<std::byte> gamepad_buf(endpoint.get_pdu_size(gamepad_key), std::byte{0});

    hako::robots::sensor::lidar::LiDAR2DSensor lidar_sensor(world, "base_scan", "base_footprint");
    if (!lidar_sensor.LoadConfig(lidar_config)) {
        std::cerr << "ERROR: failed to load LiDAR config: " << lidar_config << std::endl;
        endpoint.stop();
        endpoint.close();
        return -1;
    }
    lidar_sensor.SetRuntimeOptions(lidar_yaw_bias_deg, lidar_origin_offset);

    HakoCpp_LaserScan laser_scan {};
    hako::robots::sensor::LaserScanFrame laser_scan_frame {};

    int step = 0;
    Tb3CommandState command_state {};

    while (running_flag) {
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            // --- gamepad 受信 ---
            HakoCpp_GameControllerOperation latest {};
            size_t gamepad_received = 0;
            const auto gamepad_rc = endpoint.recv(
                gamepad_key,
                std::span<std::byte>(gamepad_buf.data(), gamepad_buf.size()),
                gamepad_received);
            if ((gamepad_rc == HAKO_PDU_ERR_OK) && (gamepad_received > 0)) {
                if (gamepad_conv.pdu2cpp(reinterpret_cast<char*>(gamepad_buf.data()), latest)) {
                    command_state.gamepad = latest;
                    command_state.has_input = true;
                } else {
                    hako_asset_usleep(delta_time_usec * 1000);
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    continue;
                }
            }

            // --- 制御 ---
            double left_torque  = 0.0;
            double right_torque = 0.0;
            if (command_state.has_input && command_state.gamepad.axis.size() >= 4) {
                const double turn    = -1.0 * std::clamp(static_cast<double>(command_state.gamepad.axis[0]), -1.0, 1.0);
                const double forward =        std::clamp(-static_cast<double>(command_state.gamepad.axis[3]), -1.0, 1.0);
                left_torque  = std::clamp((forward - turn * turn_gain) * drive_gain, -max_torque, max_torque);
                right_torque = std::clamp((forward + turn * turn_gain) * drive_gain, -max_torque, max_torque);
            }
            tb3.set_torque(left_torque, right_torque);
            world->advanceTimeStep();

            // --- base_link_pos 送信（1ms周期） ---
            {
                HakoCpp_Twist base_pos {};
                const auto pos      = tb3.position();
                const auto euler    = tb3.euler();
                base_pos.linear.x   = pos.x;
                base_pos.linear.y   = pos.y;
                base_pos.linear.z   = pos.z;
                base_pos.angular.x  = euler.x;
                base_pos.angular.y  = euler.y;
                base_pos.angular.z  = euler.z;
                (void)base_pos_ep.send(base_pos);
            }

            // --- LiDAR スキャン（lidar_period_sec 周期） ---
            // Unity: EventTick() — update_cycle ごとに Scan() → FlushNamedPdu()
            if (lidar_sensor.ShouldUpdate(sim_timestep)) {
                lidar_sensor.Scan(laser_scan_frame);
                laser_scan = to_hako_scan(laser_scan_frame);
                (void)laser_scan_ep.send(laser_scan);

                // base_scan_pos も同じタイミングでだけ送る
                HakoCpp_Twist base_scan_pos {};
                const auto scan_pos   = tb3.base_scan_position();
                const auto scan_euler = tb3.base_scan_euler();
                base_scan_pos.linear.x  = scan_pos.x;
                base_scan_pos.linear.y  = scan_pos.y;
                base_scan_pos.linear.z  = scan_pos.z;
                base_scan_pos.angular.x = scan_euler.x;
                base_scan_pos.angular.y = scan_euler.y;
                base_scan_pos.angular.z = scan_euler.z;
                (void)base_scan_pos_ep.send(base_scan_pos);
            }

            // --- デバッグログ（500ステップごと） ---
            if ((step % 500) == 0) {
                float lidar_min  = std::numeric_limits<float>::infinity();
                float lidar_max  = 0.0F;
                int   lidar_hits = 0;
                for (float v : laser_scan.ranges) {
                    if (v >= laser_scan.range_min && v < laser_scan.range_max) {
                        lidar_min = std::min(lidar_min, v);
                        lidar_max = std::max(lidar_max, v);
                        ++lidar_hits;
                    }
                }
                const auto pos      = tb3.position();
                const auto body_vel = tb3.body_velocity();
                std::cout << "[TB3] step=" << step
                          << " pos=(" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                          << " body_vx=" << body_vel.x
                          << " torque=(" << left_torque << ", " << right_torque << ")"
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

void simulation_thread(std::shared_ptr<hako::robots::physics::IWorld> w)
{
    my_callback.on_initialize          = my_on_initialize;
    my_callback.on_simulation_step     = nullptr;
    my_callback.on_manual_timing_control = my_manual_timing_control;
    my_callback.on_reset               = my_on_reset;

    const std::string asset_name  = get_env_string("HAKO_ASSET_NAME",        "tb3_sim");
    const std::string config_path = get_env_string("HAKO_ASSET_CONFIG_PATH", hako_config_path);
    const hako_time_t delta_time_usec = static_cast<hako_time_t>(w->getModel()->opt.timestep * 1e6);

    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(asset_name.c_str(), config_path.c_str(),
                                  &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
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
    if (argc >= 2) {
        lidar_config_override_path = argv[1];
    }

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
