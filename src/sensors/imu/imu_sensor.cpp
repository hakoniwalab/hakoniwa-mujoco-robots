#include "sensors/imu/imu_sensor.hpp"

#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor
{
namespace
{
using json = nlohmann::json;

double get_json_number(const json& j, const char* key, double default_value)
{
    if (!j.contains(key) || !j.at(key).is_number()) {
        return default_value;
    }
    return j.at(key).get<double>();
}

std::string get_json_string(const json& j, const char* key, const std::string& default_value)
{
    if (!j.contains(key) || !j.at(key).is_string()) {
        return default_value;
    }
    return j.at(key).get<std::string>();
}

noise::NoiseType parse_noise_type(const std::string& value)
{
    if (value == "gaussian") {
        return noise::NoiseType::Gaussian;
    }
    if (value == "gaussian_quantized") {
        return noise::NoiseType::GaussianQuantized;
    }
    return noise::NoiseType::None;
}

noise::NoiseModelConfig parse_noise_model_config(const json& j)
{
    noise::NoiseModelConfig config {};
    config.type = get_json_string(j, "type", config.type);
    config.mean = get_json_number(j, "mean", config.mean);
    config.stddev = get_json_number(j, "stddev", config.stddev);
    config.bias_mean = get_json_number(j, "bias_mean", config.bias_mean);
    config.bias_stddev = get_json_number(j, "bias_stddev", config.bias_stddev);
    config.dynamic_bias_stddev = get_json_number(j, "dynamic_bias_stddev", config.dynamic_bias_stddev);
    config.dynamic_bias_correlation_time =
        get_json_number(j, "dynamic_bias_correlation_time", config.dynamic_bias_correlation_time);
    config.precision = get_json_number(j, "precision", config.precision);
    return config;
}

noise::AxisNoiseConfig parse_axis_noise_config(const json& j)
{
    noise::AxisNoiseConfig config {};
    if (j.contains("x") && j.at("x").is_object()) {
        config.x = parse_noise_model_config(j.at("x"));
    }
    if (j.contains("y") && j.at("y").is_object()) {
        config.y = parse_noise_model_config(j.at("y"));
    }
    if (j.contains("z") && j.at("z").is_object()) {
        config.z = parse_noise_model_config(j.at("z"));
    }
    return config;
}

noise::AxisNoiseParams to_axis_noise_params(const noise::AxisNoiseConfig& config)
{
    auto convert = [](const noise::NoiseModelConfig& in) {
        noise::NoiseParams out {};
        out.type = parse_noise_type(in.type);
        out.mean = in.mean;
        out.stddev = in.stddev;
        out.bias_mean = in.bias_mean;
        out.bias_stddev = in.bias_stddev;
        out.dynamic_bias_stddev = in.dynamic_bias_stddev;
        out.dynamic_bias_correlation_time = in.dynamic_bias_correlation_time;
        out.precision = in.precision;
        return out;
    };

    noise::AxisNoiseParams params {};
    params.x = convert(config.x);
    params.y = convert(config.y);
    params.z = convert(config.z);
    return params;
}

Quaternion quat_from_mj(const mjtNum* q)
{
    Quaternion out {};
    out.w = q[0];
    out.x = q[1];
    out.y = q[2];
    out.z = q[3];
    return out;
}
}

ImuSensor::ImuSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
    : world_(std::move(world))
    , angular_velocity_noise_(noise::AxisNoiseParams {})
    , linear_acceleration_noise_(noise::AxisNoiseParams {})
{
}

bool ImuSensor::LoadConfig(const std::string& config_path)
{
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        return false;
    }

    json root;
    ifs >> root;

    config_ = ImuConfig {};
    config_.output.name = get_json_string(root, "name", "imu");
    config_.output.pdu_name = get_json_string(root, "pdu_name", "imu");
    config_.output.update_rate_hz = get_json_number(root, "update_rate_hz", 100.0);
    config_.frame_id = get_json_string(root, "frame_id", "imu_link");
    config_.parent_body = get_json_string(root, "parent_body", "");
    config_.source_body = get_json_string(root, "source_body", config_.parent_body);
    config_.mode = get_json_string(root, "mode", "ground_truth");

    if (root.contains("noise") && root.at("noise").is_object()) {
        const auto& noise_root = root.at("noise");
        if (noise_root.contains("angular_velocity") && noise_root.at("angular_velocity").is_object()) {
            config_.noise.angular_velocity = parse_axis_noise_config(noise_root.at("angular_velocity"));
        }
        if (noise_root.contains("linear_acceleration") && noise_root.at("linear_acceleration").is_object()) {
            config_.noise.linear_acceleration = parse_axis_noise_config(noise_root.at("linear_acceleration"));
        }
    }

    source_body_ = world_->getRigidBody(config_.source_body);
    elapsed_sec_ = GetUpdatePeriodSec();
    has_prev_velocity_ = false;
    RebuildNoisePipeline();
    return true;
}

const ImuConfig& ImuSensor::GetConfig() const
{
    return config_;
}

void ImuSensor::Build(ImuFrame& out)
{
    auto* model = world_->getModel();
    auto* data = world_->getData();
    const int body_id = mj_name2id(model, mjOBJ_BODY, config_.source_body.c_str());
    if (body_id < 0) {
        return;
    }

    out.header.frame_id = config_.frame_id;
    out.orientation = quat_from_mj(&data->xquat[4 * body_id]);

    const auto body_ang_vel = source_body_->GetBodyAngularVelocity();
    const auto ang_vel_noisy = angular_velocity_noise_.Apply({body_ang_vel.x, body_ang_vel.y, body_ang_vel.z});
    out.angular_velocity.x = ang_vel_noisy.x;
    out.angular_velocity.y = ang_vel_noisy.y;
    out.angular_velocity.z = ang_vel_noisy.z;

    const auto body_vel = source_body_->GetBodyVelocity();
    noise::AxisValue lin_acc_input {};
    const double dt = GetUpdatePeriodSec();
    if (has_prev_velocity_ && dt > 0.0) {
        lin_acc_input.x = (body_vel.x - prev_body_velocity_.x) / dt;
        lin_acc_input.y = (body_vel.y - prev_body_velocity_.y) / dt;
        lin_acc_input.z = (body_vel.z - prev_body_velocity_.z) / dt;
    }
    prev_body_velocity_ = body_vel;
    has_prev_velocity_ = true;

    const auto lin_acc_noisy = linear_acceleration_noise_.Apply(lin_acc_input);
    out.linear_acceleration.x = lin_acc_noisy.x;
    out.linear_acceleration.y = lin_acc_noisy.y;
    out.linear_acceleration.z = lin_acc_noisy.z;
}

void ImuSensor::Reset()
{
    elapsed_sec_ = 0.0;
    has_prev_velocity_ = false;
    prev_body_velocity_ = {};
    angular_velocity_noise_.Reset();
    linear_acceleration_noise_.Reset();
}

double ImuSensor::GetUpdatePeriodSec() const
{
    return (config_.output.update_rate_hz > 0.0) ? (1.0 / config_.output.update_rate_hz) : 0.01;
}

bool ImuSensor::ShouldUpdate(double delta_sec)
{
    elapsed_sec_ += delta_sec;
    const double period = GetUpdatePeriodSec();
    if (elapsed_sec_ + 1.0e-9 < period) {
        return false;
    }
    elapsed_sec_ -= period;
    if (elapsed_sec_ < 0.0) {
        elapsed_sec_ = 0.0;
    }
    return true;
}

void ImuSensor::RebuildNoisePipeline()
{
    angular_velocity_noise_ = noise::AxisNoisePipeline(to_axis_noise_params(config_.noise.angular_velocity), GetUpdatePeriodSec());
    linear_acceleration_noise_ = noise::AxisNoisePipeline(to_axis_noise_params(config_.noise.linear_acceleration), GetUpdatePeriodSec());
}
}
