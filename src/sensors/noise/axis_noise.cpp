#include "sensors/noise/noise.hpp"
#include <cmath>
#include <stdexcept>

namespace hako::robots::sensor::noise
{
    std::unique_ptr<INoiseModel> CreateNoiseModel(NoiseType type, double dt_sec)
    {
        switch (type) {
        case NoiseType::None:
            return std::make_unique<PassthroughNoiseModel>();
        case NoiseType::Gaussian:
        case NoiseType::GaussianQuantized:
            return std::make_unique<GaussianNoiseModel>(dt_sec);
        default:
            throw std::invalid_argument("Unknown NoiseType");
        }
    }

    double GaussianNoiseModel::UpdateDynamicBias(const NoiseParams& params)
    {
        if (params.dynamic_bias_stddev <= 0.0 ||
            params.dynamic_bias_correlation_time <= 0.0) {
            return 0.0;
        }
        const double tau   = params.dynamic_bias_correlation_time;
        const double sigma = params.dynamic_bias_stddev;
        const double decay = std::exp(-dt_ / tau);
        const double noise = sigma * std::sqrt(1.0 - decay * decay) * dist_normal_(rng_);
        current_bias_ = current_bias_ * decay + noise;
        return current_bias_;
    }

    double GaussianNoiseModel::SampleGaussian(double mean, double stddev)
    {
        if (stddev <= 0.0) return mean;
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng_);
    }

    double GaussianNoiseModel::Quantize(double value, double precision)
    {
        if (precision <= 0.0) return value;
        return std::round(value / precision) * precision;
    }

    AxisNoisePipeline::AxisNoisePipeline(const AxisNoiseParams& params, double dt_sec)
        : params_(params)
        , model_x_(CreateNoiseModel(params.x.type, dt_sec))
        , model_y_(CreateNoiseModel(params.y.type, dt_sec))
        , model_z_(CreateNoiseModel(params.z.type, dt_sec))
    {
    }

    AxisValue AxisNoisePipeline::Apply(const AxisValue& value) const
    {
        return AxisValue {
            model_x_->Apply(value.x, params_.x),
            model_y_->Apply(value.y, params_.y),
            model_z_->Apply(value.z, params_.z),
        };
    }

    void AxisNoisePipeline::Reset()
    {
        model_x_->Reset();
        model_y_->Reset();
        model_z_->Reset();
    }

} // namespace hako::robots::sensor::noise