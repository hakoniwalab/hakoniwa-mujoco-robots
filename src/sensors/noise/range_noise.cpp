#include "sensors/noise/noise.hpp"

#include <algorithm>

namespace hako::robots::sensor::noise
{
void GaussianNoiseModel::Reset()
{
    current_bias_ = 0.0;
}
GaussianNoiseModel::GaussianNoiseModel(double dt_sec)
    : dt_(dt_sec), current_bias_(0.0)
{
}

double RangeNoisePipeline::Apply(double value) const
{
    for (const auto& rule : rules_) {
        if (value >= rule.range.min && value <= rule.range.max) {
            return model_->ApplyRangeRule(value, rule);
        }
    }
    return value;
}
double GaussianNoiseModel::Apply(double value, const NoiseParams& params)
{
    if (params.stddev <= 0.0) {
        return value;
    }
    std::normal_distribution<double> dist(params.mean, params.stddev);
    return value + dist(rng_);
}
double GaussianNoiseModel::ApplyRangeRule(double value, const RangeNoiseRule& rule)
{
    double sigma = rule.noise.stddev;
    if (rule.distance_dependent) {
        sigma = value * (rule.percentage / 100.0);
    }
    if (sigma <= 0.0) {
        return value;
    }
    std::normal_distribution<double> dist(0.0, sigma);
    return value + dist(rng_);
}

RangeNoisePipeline::RangeNoisePipeline(std::unique_ptr<INoiseModel> model)
    : model_(std::move(model))
{
    if (!model_) {
        model_ = std::make_unique<GaussianNoiseModel>();
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

}