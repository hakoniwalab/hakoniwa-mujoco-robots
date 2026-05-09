#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "sensors/common/json_utils.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
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

std::array<double, 3> make_local_ray_direction(double yaw, double pitch)
{
    std::array<double, 3> dir {
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch)
    };

    const double norm = std::sqrt(
        dir[0] * dir[0] +
        dir[1] * dir[1] +
        dir[2] * dir[2]);

    if (norm > 0.0) {
        dir[0] /= norm;
        dir[1] /= norm;
        dir[2] /= norm;
    } else {
        dir = {1.0, 0.0, 0.0};
    }

    return dir;
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

        auto* model = world_->getModel();
        if (model == nullptr) {
            std::cerr
                << "ERROR: UltrasonicSensor::LoadConfig: MuJoCo model is null."
                << std::endl;
            return false;
        }

        /*
         * Resolve runtime frame once.
         *
         * Resolution policy:
         * 1. RuntimeBinding.source_site, if specified, is resolved as mjOBJ_SITE.
         * 2. Otherwise sensor_body_name_ is used as a fallback runtime object name.
         *    It is resolved as site first, then body.
         * 3. If sensor_body_name_ is empty, frame_id is used as a convention fallback.
         */
        runtime_frame_type_ = RuntimeFrameType::None;
        runtime_frame_id_ = -1;

        if (!config_.runtime_binding.source_site.empty()) {
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

            runtime_frame_type_ = RuntimeFrameType::Site;
            runtime_frame_id_ = site_id;
        } else {
            const std::string runtime_name =
                !sensor_body_name_.empty() ? sensor_body_name_ : config_.frame_id;

            int site_id = mj_name2id(model, mjOBJ_SITE, runtime_name.c_str());
            if (site_id >= 0) {
                runtime_frame_type_ = RuntimeFrameType::Site;
                runtime_frame_id_ = site_id;
            } else {
                int body_id = mj_name2id(model, mjOBJ_BODY, runtime_name.c_str());
                if (body_id >= 0) {
                    runtime_frame_type_ = RuntimeFrameType::Body;
                    runtime_frame_id_ = body_id;
                }
            }

            if (runtime_frame_type_ == RuntimeFrameType::None) {
                std::cerr
                    << "ERROR: UltrasonicSensor::LoadConfig: Failed to resolve runtime frame as site/body: "
                    << runtime_name << std::endl;
                return false;
            }
        }

        /*
         * Resolve excluded body once.
         *
         * body_exclude_id_ may remain -1.
         * In that case mj_ray will not exclude a specific body, and IsSelfGeom()
         * will also return false.
         */
        body_exclude_id_ = -1;
        if (!exclude_body_name_.empty()) {
            body_exclude_id_ = mj_name2id(
                model,
                mjOBJ_BODY,
                exclude_body_name_.c_str());

            if (body_exclude_id_ < 0) {
                std::cerr
                    << "WARN: UltrasonicSensor::LoadConfig: Failed to find exclude body: "
                    << exclude_body_name_ << std::endl;
            }
        }

        /*
         * Precompute local ray directions once.
         *
         * Cone.Horizontal and Cone.Vertical are full angles in radians.
         * Local frame convention:
         * - +X: forward / measurement axis
         * - +Y: horizontal direction
         * - +Z: vertical direction
         */
        ray_dirs_local_.clear();

        const int ray_count = std::max(1, config_.cone.ray_count);
        const int side = std::max(
            1,
            static_cast<int>(std::ceil(std::sqrt(static_cast<double>(ray_count)))));

        ray_dirs_local_.reserve(static_cast<std::size_t>(ray_count));

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

            ray_dirs_local_.push_back(make_local_ray_direction(yaw, pitch));
        }

        rebuild_noise_pipeline(config_, noise_pipeline_);
        scheduler_.StartReady(GetUpdatePeriodSec());

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

    if (runtime_frame_type_ == RuntimeFrameType::None || runtime_frame_id_ < 0) {
        set_invalid(out, config_);
        return;
    }

    if (ray_dirs_local_.empty()) {
        set_invalid(out, config_);
        return;
    }

    const mjtNum* sensor_pos = nullptr;
    const mjtNum* sensor_mat = nullptr;

    if (runtime_frame_type_ == RuntimeFrameType::Site) {
        sensor_pos = &data->site_xpos[3 * runtime_frame_id_];
        sensor_mat = &data->site_xmat[9 * runtime_frame_id_];
    } else if (runtime_frame_type_ == RuntimeFrameType::Body) {
        sensor_pos = &data->xpos[3 * runtime_frame_id_];
        sensor_mat = &data->xmat[9 * runtime_frame_id_];
    } else {
        set_invalid(out, config_);
        return;
    }

    /*
     * Convert sensor local +X into world coordinates.
     *
     * This is the measurement axis used for forward-axis projection.
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

    for (const auto& local_dir : ray_dirs_local_) {
        mjtNum ray_dir_local[3] = {
            static_cast<mjtNum>(local_dir[0]),
            static_cast<mjtNum>(local_dir[1]),
            static_cast<mjtNum>(local_dir[2])
        };

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

        const mjtNum hit_dist = mj_ray(
            model,
            data,
            from,
            ray_dir_world,
            nullptr,
            1,
            body_exclude_id_,
            geom_id,
            normal);

        if (hit_dist < 0.0) {
            continue;
        }

        if (IsSelfGeom(model, body_exclude_id_, geom_id[0])) {
            continue;
        }

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