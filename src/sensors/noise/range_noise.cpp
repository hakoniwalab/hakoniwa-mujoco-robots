#include "sensors/noise/range_noise.hpp"

#include <algorithm>

namespace hako::robots::sensor::noise
{
GaussianNoiseModel::GaussianNoiseModel()
    : rng_(std::random_device{}())
{
}

float GaussianNoiseModel::Apply(float value, const RangeNoiseRule& rule)
{
    double sigma = rule.stddev;
    if (rule.distance_dependent) {
        sigma = static_cast<double>(value) * (rule.percentage / 100.0);
    }
    if (sigma <= 0.0) {
        return value;
    }
    std::normal_distribution<double> dist(0.0, sigma);
    return static_cast<float>(static_cast<double>(value) + dist(rng_));
}

RangeNoisePipeline::RangeNoisePipeline(std::unique_ptr<INoiseModel> default_model)
    : default_model_(std::move(default_model))
{
    if (!default_model_) {
        default_model_ = std::make_unique<GaussianNoiseModel>();
    }
}

void RangeNoisePipeline::Clear()
{
    rules_.clear();
}

void RangeNoisePipeline::AddRule(const RangeNoiseRule& rule)
{
    rules_.push_back(rule);
}

float RangeNoisePipeline::Apply(float value) const
{
    for (const auto& rule : rules_) {
        if (value < rule.range.min || value >= rule.range.max) {
            continue;
        }
        if (rule.distribution != "Gaussian") {
            return value;
        }
        return default_model_->Apply(value, rule);
    }
    return value;
}
}
