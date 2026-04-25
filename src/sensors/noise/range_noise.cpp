#include "sensors/noise/noise.hpp"

#include <algorithm>

namespace hako::robots::sensor::noise
{
void GaussianNoiseModel::Reset()
{
    current_bias_     = 0.0;
    static_bias_      = 0.0;
    bias_initialized_ = false;
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
    if (params.type == NoiseType::None) return value;

    // 静的バイアスは初回のみサンプリング
    if (!bias_initialized_) {
        static_bias_      = SampleGaussian(params.bias_mean, params.bias_stddev);
        bias_initialized_ = true;
    }
    const double dynamic_bias = UpdateDynamicBias(params);
    const double noise        = SampleGaussian(params.mean, params.stddev);

    double result = value + noise + static_bias_ + dynamic_bias;
    if (params.type == NoiseType::GaussianQuantized) {
        result = Quantize(result, params.precision);
    }
    return result;
}

double GaussianNoiseModel::ApplyRangeRule(double value, const RangeNoiseRule& rule)
{
    double sigma = rule.noise.stddev;
    if (rule.distance_dependent) {
        sigma = value * (rule.percentage / 100.0);
    }
    if (sigma <= 0.0) return value;

    std::normal_distribution<double> dist(0.0, sigma);
    double result = value + dist(rng_);
    if (rule.noise.type == NoiseType::GaussianQuantized) {
        result = Quantize(result, rule.noise.precision);
    }
    return result;
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