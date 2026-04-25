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
#include "hakoniwa/pdu/nav_msgs/pdu_cpptype_Odometry.hpp"
#include "hakoniwa/pdu/nav_msgs/pdu_cpptype_conv_Odometry.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_Imu.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_JointState.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_Imu.hpp"
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray
#include "hakoniwa/pdu/sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "hakoniwa/pdu/tf2_msgs/pdu_cpptype_TFMessage.hpp"
#include "hakoniwa/pdu/tf2_msgs/pdu_cpptype_conv_TFMessage.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "physics/physics_impl.hpp"
#include "sensors/imu/imu_sensor.hpp"
#include "sensors/joint_state/joint_state_sensor.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"
#include "sensors/odometry/odometry_sensor.hpp"
#include "sensors/tf/tf_publisher.hpp"

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
    (repo_root_path() / "config/sensors/lidar/lds-02.json").string();
const std::string imu_config_path =
    (repo_root_path() / "config/sensors/imu/tb3-imu.json").string();
const std::string joint_state_config_path =
    (repo_root_path() / "config/sensors/joint_state/tb3-wheel-joint-states.json").string();
const std::string odom_config_path =
    (repo_root_path() / "config/sensors/odometry/tb3-ground-truth-odom.json").string();
const std::string tf_config_path =
    (repo_root_path() / "config/sensors/tf/tb3-basic-tf.json").string();

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

HakoCpp_LaserScan to_hako_scan(const hako::robots::sensor::lidar::LaserScanFrame& frame)
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

HakoCpp_Time to_hako_time(double stamp_sec)
{
    HakoCpp_Time out {};
    const double sec_floor = std::floor(stamp_sec);
    out.sec = static_cast<Hako_int32>(sec_floor);
    out.nanosec = static_cast<Hako_uint32>((stamp_sec - sec_floor) * 1.0e9);
    return out;
}

HakoCpp_Header to_hako_header(const hako::robots::sensor::MessageHeader& header)
{
    HakoCpp_Header out {};
    out.stamp = to_hako_time(header.stamp_sec);
    out.frame_id = header.frame_id;
    return out;
}

HakoCpp_Quaternion to_hako_quaternion(const hako::robots::sensor::Quaternion& q)
{
    HakoCpp_Quaternion out {};
    out.x = q.x;
    out.y = q.y;
    out.z = q.z;
    out.w = q.w;
    return out;
}

HakoCpp_Vector3 to_hako_vector3(const hako::robots::types::Vector3& v)
{
    HakoCpp_Vector3 out {};
    out.x = v.x;
    out.y = v.y;
    out.z = v.z;
    return out;
}

HakoCpp_Imu to_hako_imu(const hako::robots::sensor::ImuFrame& frame)
{
    HakoCpp_Imu out {};
    out.header = to_hako_header(frame.header);
    out.orientation = to_hako_quaternion(frame.orientation);
    out.angular_velocity = to_hako_vector3(frame.angular_velocity);
    out.linear_acceleration = to_hako_vector3(frame.linear_acceleration);
    out.orientation_covariance.fill(0.0);
    out.angular_velocity_covariance.fill(0.0);
    out.linear_acceleration_covariance.fill(0.0);
    return out;
}

HakoCpp_JointState to_hako_joint_state(const hako::robots::sensor::JointStateFrame& frame)
{
    HakoCpp_JointState out {};
    out.header = to_hako_header(frame.header);
    out.name = frame.names;
    out.position = frame.position;
    out.velocity = frame.velocity;
    out.effort = frame.effort;
    return out;
}

HakoCpp_Odometry to_hako_odometry(const hako::robots::sensor::OdometryFrame& frame)
{
    HakoCpp_Odometry out {};
    out.header = to_hako_header(frame.header);
    out.child_frame_id = frame.child_frame_id;
    out.pose.pose.position.x = frame.pose.position.x;
    out.pose.pose.position.y = frame.pose.position.y;
    out.pose.pose.position.z = frame.pose.position.z;
    out.pose.pose.orientation = to_hako_quaternion(frame.pose.orientation);
    out.pose.covariance.fill(0.0);
    out.twist.twist.linear = to_hako_vector3(frame.twist.linear);
    out.twist.twist.angular = to_hako_vector3(frame.twist.angular);
    out.twist.covariance.fill(0.0);
    return out;
}

HakoCpp_TFMessage to_hako_tf(const hako::robots::sensor::TfFrame& frame)
{
    HakoCpp_TFMessage out {};
    out.transforms.reserve(frame.transforms.size());
    for (const auto& src : frame.transforms) {
        HakoCpp_TransformStamped dst {};
        dst.header = to_hako_header(src.header);
        dst.child_frame_id = src.child_frame_id;
        dst.transform.translation.x = src.transform.position.x;
        dst.transform.translation.y = src.transform.position.y;
        dst.transform.translation.z = src.transform.position.z;
        dst.transform.rotation = to_hako_quaternion(src.transform.orientation);
        out.transforms.push_back(std::move(dst));
    }
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
    const std::string imu_config = resolve_repo_path(get_env_string("HAKO_TB3_IMU_CONFIG_PATH", imu_config_path));
    const std::string joint_state_config =
        resolve_repo_path(get_env_string("HAKO_TB3_JOINT_STATE_CONFIG_PATH", joint_state_config_path));
    const std::string odom_config =
        resolve_repo_path(get_env_string("HAKO_TB3_ODOM_CONFIG_PATH", odom_config_path));
    const std::string tf_config =
        resolve_repo_path(get_env_string("HAKO_TB3_TF_CONFIG_PATH", tf_config_path));
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
    const hakoniwa::pdu::PduKey imu_key          {"TB3", "imu"};
    const hakoniwa::pdu::PduKey joint_state_key  {"TB3", "joint_states"};
    const hakoniwa::pdu::PduKey odom_key         {"TB3", "odom"};
    const hakoniwa::pdu::PduKey tf_key           {"TB3", "tf"};

    endpoint.start();
    endpoint.post_start();

    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_pos_ep(endpoint, base_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist>
        base_scan_pos_ep(endpoint, base_scan_pos_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_LaserScan, hako::pdu::msgs::sensor_msgs::LaserScan>
        laser_scan_ep(endpoint, laser_scan_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Imu, hako::pdu::msgs::sensor_msgs::Imu>
        imu_ep(endpoint, imu_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_JointState, hako::pdu::msgs::sensor_msgs::JointState>
        joint_state_ep(endpoint, joint_state_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_Odometry, hako::pdu::msgs::nav_msgs::Odometry>
        odom_ep(endpoint, odom_key);
    hakoniwa::pdu::TypedEndpoint<HakoCpp_TFMessage, hako::pdu::msgs::tf2_msgs::TFMessage>
        tf_ep(endpoint, tf_key);

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
    hako::robots::sensor::ImuSensor imu_sensor(world);
    hako::robots::sensor::JointStateSensor joint_state_sensor(world);
    hako::robots::sensor::OdometryPublisher odom_sensor(world);
    hako::robots::sensor::TfPublisher tf_sensor(world);
    if (!imu_sensor.LoadConfig(imu_config) ||
        !joint_state_sensor.LoadConfig(joint_state_config) ||
        !odom_sensor.LoadConfig(odom_config) ||
        !tf_sensor.LoadConfig(tf_config))
    {
        std::cerr << "ERROR: failed to load TB3 sensor configs" << std::endl;
        endpoint.stop();
        endpoint.close();
        return -1;
    }

    HakoCpp_LaserScan laser_scan {};
    hako::robots::sensor::lidar::LaserScanFrame laser_scan_frame {};
    HakoCpp_Imu imu {};
    hako::robots::sensor::ImuFrame imu_frame {};
    HakoCpp_JointState joint_state {};
    hako::robots::sensor::JointStateFrame joint_state_frame {};
    HakoCpp_Odometry odom {};
    hako::robots::sensor::OdometryFrame odom_frame {};
    HakoCpp_TFMessage tf {};
    hako::robots::sensor::TfFrame tf_frame {};

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
                const double turn    = -1.5 * std::clamp(static_cast<double>(command_state.gamepad.axis[0]), -1.0, 1.0);
                const double forward =  0.5 * std::clamp(-static_cast<double>(command_state.gamepad.axis[3]), -1.0, 1.0);
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

            const double sim_time_sec = static_cast<double>(hako_asset_simulation_time()) / 1.0e6;

            if (imu_sensor.ShouldUpdate(sim_timestep)) {
                imu_sensor.Build(imu_frame);
                imu_frame.header.frame_id = imu_sensor.GetConfig().frame_id;
                imu_frame.header.stamp_sec = sim_time_sec;
                imu = to_hako_imu(imu_frame);
                (void)imu_ep.send(imu);
            }
            if (joint_state_sensor.ShouldUpdate(sim_timestep)) {
                joint_state_sensor.Build(joint_state_frame);
                joint_state_frame.header.frame_id = "";
                joint_state_frame.header.stamp_sec = sim_time_sec;
                joint_state = to_hako_joint_state(joint_state_frame);
                (void)joint_state_ep.send(joint_state);
            }
            if (odom_sensor.ShouldUpdate(sim_timestep)) {
                odom_sensor.Build(odom_frame);
                odom_frame.header.frame_id = odom_sensor.GetConfig().frame_id;
                odom_frame.header.stamp_sec = sim_time_sec;
                odom = to_hako_odometry(odom_frame);
                (void)odom_ep.send(odom);
            }
            if (tf_sensor.ShouldUpdate(sim_timestep)) {
                tf_sensor.Build(tf_frame);
                for (auto& transform : tf_frame.transforms) {
                    transform.header.stamp_sec = sim_time_sec;
                }
                tf = to_hako_tf(tf_frame);
                (void)tf_ep.send(tf);
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
