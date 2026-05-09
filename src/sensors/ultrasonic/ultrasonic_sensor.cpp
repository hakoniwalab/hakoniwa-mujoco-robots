#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "sensors/common/json_utils.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace hako::robots::sensor::ultrasonic {

namespace {

noise::NoiseType parse_noise_type(const std::string& value)
{
    if (value == "none" || value == "None") {
        return noise::NoiseType::None;
    }
    if (value == "gaussian_quantized" || value == "GaussianQuantized") {
        return noise::NoiseType::GaussianQuantized;
    }
    return noise::NoiseType::Gaussian;
}

void rebuild_noise_pipeline(
    const UltrasonicConfig& config,
    noise::RangeNoisePipeline& pipeline)
{
    pipeline.Clear();

    for (const auto& accuracy : config.distance_accuracy) {
        noise::RangeNoiseRule rule{};
        rule.range.min = accuracy.range.min;
        rule.range.max = accuracy.range.max;
        rule.distance_dependent = false;
        rule.noise.stddev = accuracy.stddev;
        rule.noise.precision = accuracy.precision;
        rule.noise.type = parse_noise_type(accuracy.noise_distribution);
        pipeline.AddRule(rule);
    }
}

void set_invalid(UltrasonicFrame& out, const UltrasonicConfig& config)
{
    out.frame_id = config.frame_id;
    out.range = 0.0;
    out.variance = 0.0;
    out.status = UltrasonicStatus::INVALID;
}

double find_stddev_for_range(const UltrasonicConfig& config, double range)
{
    for (const auto& acc : config.distance_accuracy) {
        if (range >= acc.range.min && range <= acc.range.max) {
            return acc.stddev;
        }
    }
    return 0.0;
}

} // namespace

UltrasonicSensor::UltrasonicSensor(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    std::string sensor_body_name,
    std::string exclude_body_name)
    : world_(std::move(world)),
      sensor_body_name_(std::move(sensor_body_name)),
      exclude_body_name_(std::move(exclude_body_name))
{
}

bool UltrasonicSensor::LoadConfig(const std::string& config_path)
{
    common::json root;
    if (!common::load_json_file(config_path, root)) {
        std::cerr
            << "ERROR: UltrasonicSensor::LoadConfig: Failed to open or parse config file: "
            << config_path << std::endl;
        return false;
    }

    try {
        config_.frame_id = root.value("frame_id", "ultrasonic");

        if (root.contains("DetectionDistance")) {
            const auto& j_dist = root.at("DetectionDistance");
            config_.detection_distance.min = common::get_json_number(j_dist, "Min", 0.0);
            config_.detection_distance.max = common::get_json_number(j_dist, "Max", 0.0);
        }

        if (root.contains("Cone")) {
            const auto& j_cone = root.at("Cone");
            config_.cone.horizontal = common::get_json_number(j_cone, "Horizontal", 0.0);
            config_.cone.vertical = common::get_json_number(j_cone, "Vertical", 0.0);
            config_.cone.ray_count = common::get_json_int(j_cone, "RayCount", 1);
        }

        config_.update_rate = root.value("UpdateRate", 10.0);

        config_.distance_accuracy.clear();
        if (root.contains("DistanceAccuracy") && root.at("DistanceAccuracy").is_array()) {
            for (const auto& j_acc : root.at("DistanceAccuracy")) {
                DistanceAccuracy accuracy{};

                if (j_acc.contains("Range")) {
                    const auto& j_range = j_acc.at("Range");
                    accuracy.range.min = common::get_json_number(j_range, "Min", 0.0);
                    accuracy.range.max = common::get_json_number(j_range, "Max", 0.0);
                }

                accuracy.stddev = common::get_json_number(j_acc, "StdDev", 0.0);
                accuracy.precision = common::get_json_number(j_acc, "Precision", 0.0);
                accuracy.noise_distribution = j_acc.value("NoiseDistribution", "gaussian");

                config_.distance_accuracy.push_back(accuracy);
            }
        }

        if (root.contains("RuntimeBinding") && root.at("RuntimeBinding").is_object()) {
            const auto& rb = root.at("RuntimeBinding");

            config_.runtime_binding.config_style =
                rb.value("config_style", config_.runtime_binding.config_style);

            config_.runtime_binding.runtime_source =
                rb.value("runtime_source", config_.runtime_binding.runtime_source);

            config_.runtime_binding.parent_body =
                rb.value("parent_body", config_.runtime_binding.parent_body);

            config_.runtime_binding.source_site =
                rb.value("source_site", config_.runtime_binding.source_site);
        }

        if (config_.detection_distance.max <= config_.detection_distance.min) {
            std::cerr
                << "ERROR: UltrasonicSensor::LoadConfig: invalid DetectionDistance: min="
                << config_.detection_distance.min
                << ", max=" << config_.detection_distance.max
                << std::endl;
            return false;
        }

        if (config_.cone.ray_count < 1) {
            config_.cone.ray_count = 1;
        }

        if (config_.update_rate <= 0.0) {
            std::cerr
                << "ERROR: UltrasonicSensor::LoadConfig: invalid UpdateRate: "
                << config_.update_rate << std::endl;
            return false;
        }

        rebuild_noise_pipeline(config_, noise_pipeline_);
        scheduler_.StartReady(GetUpdatePeriodSec());

        /*
         * Runtime binding policy:
         *
         * - If RuntimeBinding.source_site is specified, Measure() resolves it
         *   directly as a MuJoCo site using mj_name2id(mjOBJ_SITE).
         *
         * - Otherwise, this class falls back to sensor_body_name_ and resolves
         *   it through the Hakoniwa physics abstraction.
         *
         * Do not resolve source_site using getRigidBody(); MuJoCo sites and
         * bodies are different object types.
         */
        sensor_body_.reset();

        if (config_.runtime_binding.source_site.empty()) {
            sensor_body_ = world_->getRigidBody(sensor_body_name_);
            if (sensor_body_ == nullptr) {
                std::cerr
                    << "ERROR: UltrasonicSensor::LoadConfig: Failed to find sensor body: "
                    << sensor_body_name_ << std::endl;
                return false;
            }
        } else {
            auto* model = world_->getModel();
            if (model == nullptr) {
                std::cerr
                    << "ERROR: UltrasonicSensor::LoadConfig: MuJoCo model is null."
                    << std::endl;
                return false;
            }

            const int site_id = mj_name2id(
                model,
                mjOBJ_SITE,
                config_.runtime_binding.source_site.c_str());

            if (site_id < 0) {
                std::cerr
                    << "ERROR: UltrasonicSensor::LoadConfig: Failed to find source_site: "
                    << config_.runtime_binding.source_site << std::endl;
                return false;
            }
        }
    } catch (const nlohmann::json::exception& e) {
        std::cerr
            << "ERROR: UltrasonicSensor::LoadConfig: JSON parsing error: "
            << e.what() << std::endl;
        return false;
    }

    return true;
}

const UltrasonicConfig& UltrasonicSensor::GetConfig() const
{
    return config_;
}

void UltrasonicSensor::Reset()
{
    scheduler_.Reset();
}

double UltrasonicSensor::GetUpdatePeriodSec() const
{
    if (config_.update_rate > 0.0) {
        return 1.0 / config_.update_rate;
    }
    return 0.1;
}

bool UltrasonicSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}

void UltrasonicSensor::Measure(UltrasonicFrame& out)
{
    auto* model = world_->getModel();
    auto* data = world_->getData();

    if (model == nullptr || data == nullptr) {
        set_invalid(out, config_);
        return;
    }

    const mjtNum* sensor_pos = nullptr;
    const mjtNum* sensor_mat = nullptr;

    /*
     * Resolve sensor origin/orientation.
     *
     * Preferred:
     * - RuntimeBinding.source_site as MuJoCo site.
     *
     * Fallback:
     * - sensor_body_name_ as MuJoCo body.
     */
    if (!config_.runtime_binding.source_site.empty()) {
        const int site_id = mj_name2id(
            model,
            mjOBJ_SITE,
            config_.runtime_binding.source_site.c_str());

        if (site_id < 0) {
            set_invalid(out, config_);
            return;
        }

        sensor_pos = &data->site_xpos[3 * site_id];
        sensor_mat = &data->site_xmat[9 * site_id];
    } else {
        const int body_id = mj_name2id(
            model,
            mjOBJ_BODY,
            sensor_body_name_.c_str());

        if (body_id < 0) {
            set_invalid(out, config_);
            return;
        }

        sensor_pos = &data->xpos[3 * body_id];
        sensor_mat = &data->xmat[9 * body_id];
    }

    /*
     * Sensor local frame convention:
     * - +X: forward / measurement axis
     * - +Y: horizontal direction
     * - +Z: vertical direction
     *
     * MuJoCo xmat/site_xmat is a 3x3 orientation matrix.
     * mju_mulMatVec3(dst, mat, vec) transforms the local vector by the matrix.
     */
    mjtNum sensor_forward_local[3] = {1.0, 0.0, 0.0};
    mjtNum sensor_forward_world[3] = {0.0, 0.0, 0.0};

    mju_mulMatVec3(
        sensor_forward_world,
        sensor_mat,
        sensor_forward_local);

    mju_normalize3(sensor_forward_world);

    double min_projected_dist = config_.detection_distance.max;
    bool hit_found = false;
    bool below_min_found = false;

    const int ray_count = std::max(1, config_.cone.ray_count);
    const int side = std::max(
        1,
        static_cast<int>(std::ceil(std::sqrt(static_cast<double>(ray_count)))));

    const int body_exclude_id = mj_name2id(
        model,
        mjOBJ_BODY,
        exclude_body_name_.c_str());

    for (int i = 0; i < ray_count; ++i) {
        const int row = i / side;
        const int col = i % side;

        const double h_ratio =
            (side == 1)
                ? 0.0
                : -0.5 + static_cast<double>(col) / static_cast<double>(side - 1);

        const double v_ratio =
            (side == 1)
                ? 0.0
                : -0.5 + static_cast<double>(row) / static_cast<double>(side - 1);

        const double yaw = h_ratio * config_.cone.horizontal;
        const double pitch = v_ratio * config_.cone.vertical;

        /*
         * Generate a ray direction in the sensor local frame.
         *
         * yaw:
         * - horizontal angular offset around local Z.
         *
         * pitch:
         * - vertical angular offset.
         *
         * The local forward axis is +X.
         */
        mjtNum ray_dir_local[3] = {
            static_cast<mjtNum>(std::cos(pitch) * std::cos(yaw)),
            static_cast<mjtNum>(std::cos(pitch) * std::sin(yaw)),
            static_cast<mjtNum>(-std::sin(pitch))
        };

        mju_normalize3(ray_dir_local);

        mjtNum ray_dir_world[3] = {0.0, 0.0, 0.0};

        mju_mulMatVec3(
            ray_dir_world,
            sensor_mat,
            ray_dir_local);

        mju_normalize3(ray_dir_world);

        mjtNum from[3] = {
            sensor_pos[0],
            sensor_pos[1],
            sensor_pos[2]
        };

        int geom_id[1] = {-1};
        mjtNum normal[3] = {0.0, 0.0, 0.0};

        /*
         * mj_ray returns the distance along ray_dir_world.
         *
         * In this MuJoCo version, mj_ray takes 9 arguments:
         * - model
         * - data
         * - ray origin
         * - ray direction
         * - geom group filter
         * - static geom flag
         * - excluded body id
         * - output geom id array
         * - output normal
         */
        const mjtNum hit_dist = mj_ray(
            model,
            data,
            from,
            ray_dir_world,
            nullptr,
            1,
            body_exclude_id,
            geom_id,
            normal);

        if (hit_dist < 0.0) {
            continue;
        }
        if (IsSelfGeom(model, body_exclude_id, geom_id[0])) {
            continue;
        }
        /*
         * Convert the ray distance into forward-axis projected distance.
         *
         * projected = ray_distance * cos(theta)
         *
         * Since both vectors are normalized:
         * cos(theta) = dot(sensor_forward_world, ray_dir_world)
         */
        const double cos_theta =
            static_cast<double>(
                sensor_forward_world[0] * ray_dir_world[0] +
                sensor_forward_world[1] * ray_dir_world[1] +
                sensor_forward_world[2] * ray_dir_world[2]);

        if (cos_theta <= 0.0) {
            continue;
        }

        const double projected_dist = static_cast<double>(hit_dist) * cos_theta;

        if (projected_dist < config_.detection_distance.min) {
            below_min_found = true;
            min_projected_dist = config_.detection_distance.min;
            continue;
        }

        if (projected_dist <= config_.detection_distance.max &&
            projected_dist < min_projected_dist) {
            min_projected_dist = projected_dist;
            hit_found = true;
        }
    }

    out.frame_id = config_.frame_id;

    if (hit_found) {
        out.status = UltrasonicStatus::OK;
        out.range = min_projected_dist;
    } else if (below_min_found) {
        out.status = UltrasonicStatus::BELOW_MIN_RANGE;
        out.range = config_.detection_distance.min;
    } else {
        out.status = UltrasonicStatus::NO_HIT;
        out.range = config_.detection_distance.max;
    }

    const double noisy_range = noise_pipeline_.Apply(out.range);

    out.range = std::clamp(
        noisy_range,
        config_.detection_distance.min,
        config_.detection_distance.max);

    const double stddev = find_stddev_for_range(config_, out.range);
    out.variance = stddev * stddev;
}

} // namespace hako::robots::sensor::ultrasonic 