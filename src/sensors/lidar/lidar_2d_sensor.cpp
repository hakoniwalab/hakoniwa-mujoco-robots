#include "sensors/lidar/lidar_2d_sensor.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor::lidar
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

int get_json_int(const json& j, const char* key, int default_value)
{
    if (!j.contains(key) || !j.at(key).is_number_integer()) {
        return default_value;
    }
    return j.at(key).get<int>();
}
}

LiDAR2DSensor::LiDAR2DSensor(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    std::string sensor_body_name,
    std::string exclude_body_name)
    : world_(std::move(world))
    , sensor_body_name_(std::move(sensor_body_name))
    , exclude_body_name_(std::move(exclude_body_name))
{
    sensor_body_ = world_->getRigidBody(sensor_body_name_);
}

bool LiDAR2DSensor::LoadConfig(const std::string& config_path)
{
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        return false;
    }

    json root;
    ifs >> root;

    config_ = LiDAR2DConfig {};
    config_.frame_id = root.value("frame_id", std::string("laser"));

    if (root.contains("DetectionDistance")) {
        const auto& det = root.at("DetectionDistance");
        config_.detection_distance.min = get_json_number(det, "Min", config_.detection_distance.min) / 1000.0;
        config_.detection_distance.max = get_json_number(det, "Max", config_.detection_distance.max) / 1000.0;
    }

    if (root.contains("AngleRange")) {
        const auto& angle = root.at("AngleRange");
        config_.angle_range.min_deg = get_json_number(angle, "Min", config_.angle_range.min_deg);
        config_.angle_range.max_deg = get_json_number(angle, "Max", config_.angle_range.max_deg);
        if (angle.contains("AscendingOrderOfData") && angle.at("AscendingOrderOfData").is_boolean()) {
            config_.angle_range.ascending_order_of_data = angle.at("AscendingOrderOfData").get<bool>();
        }
        config_.angle_range.resolution_deg = get_json_number(angle, "Resolution", config_.angle_range.resolution_deg);
        config_.angle_range.scan_frequency_hz = get_json_int(angle, "ScanFrequency", config_.angle_range.scan_frequency_hz);
        if (angle.contains("BlindPaddingRange") && angle.at("BlindPaddingRange").is_object()) {
            const auto& blind = angle.at("BlindPaddingRange");
            config_.angle_range.blind_padding.enabled = true;
            config_.angle_range.blind_padding.size = get_json_int(blind, "Size", 0);
            config_.angle_range.blind_padding.value = get_json_number(blind, "Value", 0.0);
        }
    }

    config_.distance_accuracy.clear();
    if (root.contains("DistanceAccuracy") && root.at("DistanceAccuracy").is_array()) {
        for (const auto& entry : root.at("DistanceAccuracy")) {
            DistanceAccuracy accuracy {};
            if (entry.contains("Range") && entry.at("Range").is_object()) {
                const auto& range = entry.at("Range");
                accuracy.range.min = get_json_number(range, "Min", 0.0) / 1000.0;
                accuracy.range.max = get_json_number(range, "Max", 0.0) / 1000.0;
            }
            std::string type = entry.value("type", std::string(""));
            if (type.empty()) {
                type = entry.value("Type", std::string("independent"));
            }
            accuracy.distance_dependent = (type == "dependent");
            if (accuracy.distance_dependent) {
                if (entry.contains("DistanceDepedentAccuracy")) {
                    const auto& dep = entry.at("DistanceDepedentAccuracy");
                    accuracy.percentage = get_json_number(dep, "Percentage", 0.0);
                    accuracy.noise_distribution = dep.value("NoiseDistribution", std::string("Gaussian"));
                }
            } else if (entry.contains("DistanceIndepedentAccuracy")) {
                const auto& indep = entry.at("DistanceIndepedentAccuracy");
                accuracy.stddev = get_json_number(indep, "StdDev", 0.0);
                accuracy.noise_distribution = indep.value("NoiseDistribution", std::string("Gaussian"));
            }
            config_.distance_accuracy.push_back(std::move(accuracy));
        }
    }

    elapsed_sec_ = GetUpdatePeriodSec();
    RebuildNoisePipeline();
    return true;
}

void LiDAR2DSensor::SetRuntimeOptions(double yaw_bias_deg, double origin_offset_m)
{
    config_.yaw_bias_deg = yaw_bias_deg;
    config_.origin_offset_m = origin_offset_m;
}

const LiDAR2DConfig& LiDAR2DSensor::GetConfig() const
{
    return config_;
}

void LiDAR2DSensor::Reset()
{
    elapsed_sec_ = 0.0;
}

double LiDAR2DSensor::GetUpdatePeriodSec() const
{
    if (config_.angle_range.scan_frequency_hz <= 0) {
        return 0.1;
    }
    return 1.0 / static_cast<double>(config_.angle_range.scan_frequency_hz);
}

bool LiDAR2DSensor::ShouldUpdate(double delta_sec)
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

float LiDAR2DSensor::CastRay(
    const mjModel* model,
    mjData* data,
    const mjtNum* sensor_pos,
    int body_exclude,
    double base_yaw_rad,
    double degree_yaw) const
{
    const double local_yaw_rad = (degree_yaw + config_.yaw_bias_deg) * M_PI / 180.0;
    const double world_yaw_rad = base_yaw_rad + local_yaw_rad;
    mjtNum dir[3] = {
        std::cos(world_yaw_rad),
        std::sin(world_yaw_rad),
        0.0,
    };
    mjtNum origin[3] = {sensor_pos[0], sensor_pos[1], sensor_pos[2]};
    const mjtNum epsilon = std::max<mjtNum>(1.0e-4, static_cast<mjtNum>(config_.origin_offset_m * 0.1));
    mjtNum traveled = 0.0;

    for (int attempt = 0; attempt < 16; ++attempt) {
        int geomid = -1;
        mjtNum normal[3] = {0.0, 0.0, 0.0};
        const mjtNum hit_dist = mj_ray(
            model,
            data,
            origin,
            dir,
            nullptr,
            1,
            -1,
            &geomid,
            normal);
        if (hit_dist < 0.0) {
            return static_cast<float>(config_.detection_distance.max);
        }

        if (!IsSelfGeom(model, body_exclude, geomid)) {
            const float true_dist = static_cast<float>(traveled + hit_dist);
            if (true_dist < static_cast<float>(config_.detection_distance.min) ||
                true_dist > static_cast<float>(config_.detection_distance.max)) {
                return static_cast<float>(config_.detection_distance.max);
            }
            return true_dist;
        }

        const mjtNum step = hit_dist + epsilon;
        traveled += step;
        if (traveled >= static_cast<mjtNum>(config_.detection_distance.max)) {
            return static_cast<float>(config_.detection_distance.max);
        }
        origin[0] += dir[0] * step;
        origin[1] += dir[1] * step;
        origin[2] += dir[2] * step;
    }

    return static_cast<float>(config_.detection_distance.max);
}

bool LiDAR2DSensor::IsSelfGeom(const mjModel* model, int body_exclude, int geom_id) const
{
    if (geom_id < 0 || body_exclude < 0) {
        return false;
    }
    int body_id = model->geom_bodyid[geom_id];
    while (body_id >= 0) {
        if (body_id == body_exclude) {
            return true;
        }
        const int parent_id = model->body_parentid[body_id];
        if (parent_id == body_id) {
            break;
        }
        body_id = parent_id;
    }
    return false;
}

void LiDAR2DSensor::ApplyBlindPadding(std::vector<float>& ranges) const
{
    if (!config_.angle_range.blind_padding.enabled || config_.angle_range.blind_padding.size <= 0) {
        return;
    }
    const size_t pad = static_cast<size_t>(config_.angle_range.blind_padding.size);
    std::vector<float> padded(ranges.size() + pad, static_cast<float>(config_.angle_range.blind_padding.value));
    for (size_t i = 0; i < ranges.size(); ++i) {
        padded[pad + i] = ranges[i];
    }
    ranges.swap(padded);
}

void LiDAR2DSensor::RebuildNoisePipeline()
{
    noise_pipeline_.Clear();
    for (const auto& accuracy : config_.distance_accuracy) {
        noise::RangeNoiseRule rule {};
        rule.range.min = accuracy.range.min;
        rule.range.max = accuracy.range.max;
        rule.distance_dependent = accuracy.distance_dependent;
        rule.percentage = accuracy.percentage;
        rule.stddev = accuracy.stddev;
        rule.distribution = accuracy.noise_distribution;
        noise_pipeline_.AddRule(rule);
    }
}

void LiDAR2DSensor::Scan(LaserScanFrame& out)
{
    auto* model = world_->getModel();
    auto* data = world_->getData();
    const int sensor_body_id = mj_name2id(model, mjOBJ_BODY, sensor_body_name_.c_str());
    const int body_exclude_id = mj_name2id(model, mjOBJ_BODY, exclude_body_name_.c_str());
    if (sensor_body_id < 0) {
        return;
    }

    const mjtNum* pos = &data->xpos[3 * sensor_body_id];
    const double base_yaw_rad = sensor_body_->GetEuler().z;

    if (config_.angle_range.resolution_deg <= 0.0) {
        return;
    }

    const int ray_count = std::max(
        1,
        static_cast<int>(std::ceil((config_.angle_range.max_deg - config_.angle_range.min_deg) / config_.angle_range.resolution_deg)));
    std::vector<float> ranges(static_cast<size_t>(ray_count), static_cast<float>(config_.detection_distance.max));

    double yaw_deg = config_.angle_range.ascending_order_of_data
        ? config_.angle_range.min_deg
        : config_.angle_range.max_deg;
    const double delta_yaw = config_.angle_range.ascending_order_of_data
        ? config_.angle_range.resolution_deg
        : -config_.angle_range.resolution_deg;

    for (int i = 0; i < ray_count; ++i, yaw_deg += delta_yaw) {
        const float raw = CastRay(model, data, pos, body_exclude_id, base_yaw_rad, yaw_deg);
        const float noisy = noise_pipeline_.Apply(raw);
        ranges[static_cast<size_t>(i)] = std::min(noisy, static_cast<float>(config_.detection_distance.max));
    }

    ApplyBlindPadding(ranges);

    out.frame_id = config_.frame_id;
    out.angle_min = static_cast<float>(config_.angle_range.min_deg * M_PI / 180.0);
    out.angle_max = static_cast<float>(config_.angle_range.max_deg * M_PI / 180.0);
    out.angle_increment = static_cast<float>(config_.angle_range.resolution_deg * M_PI / 180.0);
    out.scan_time = static_cast<float>(GetUpdatePeriodSec());
    out.range_min = static_cast<float>(config_.detection_distance.min);
    out.range_max = static_cast<float>(config_.detection_distance.max);
    out.ranges = std::move(ranges);
    out.intensities.assign(out.ranges.size(), 0.0F);
    const int measurement_count = std::max(1, static_cast<int>(out.ranges.size()));
    out.time_increment = out.scan_time / static_cast<float>(measurement_count);
}
}
