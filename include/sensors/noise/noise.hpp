#pragma once

#include <memory>
#include <random>
#include <vector>

namespace hako {
namespace robots {
namespace sensor {
namespace noise {

enum class NoiseType {
    None,
    Gaussian,
    GaussianQuantized,
};

struct NoiseParams {
    NoiseType type {NoiseType::None};
    double mean {0.0};
    double stddev {0.0};
    double bias_mean {0.0};
    double bias_stddev {0.0};
    double dynamic_bias_stddev {0.0};
    double dynamic_bias_correlation_time {0.0};
    double precision {0.0};
};

struct AxisNoiseParams {
    NoiseParams x {};
    NoiseParams y {};
    NoiseParams z {};
};

struct Range {
    double min {0.0};
    double max {0.0};
};

struct RangeNoiseRule {
    Range range {};
    bool distance_dependent {false};
    double percentage {0.0};
    NoiseParams noise {};
};

class INoiseModel {
public:
    virtual ~INoiseModel() = default;

    virtual double Apply(double value, const NoiseParams& params) = 0;
    virtual double ApplyRangeRule(double value, const RangeNoiseRule& rule) = 0;
    virtual void Reset() = 0;
};

class GaussianNoiseModel : public INoiseModel {
public:
    explicit GaussianNoiseModel(double dt_sec = 0.001);

    double Apply(double value, const NoiseParams& params) override;
    double ApplyRangeRule(double value, const RangeNoiseRule& rule) override;
    void Reset() override;

private:
    double dt_;
    double current_bias_ {0.0};
    std::mt19937 rng_;
    std::normal_distribution<double> dist_normal_ {0.0, 1.0};
    double static_bias_ {0.0};
    bool bias_initialized_ {false};

    double SampleGaussian(double mean, double stddev);
    double UpdateDynamicBias(const NoiseParams& params);
    double Quantize(double value, double precision);
};

class PassthroughNoiseModel : public INoiseModel {
public:
    double Apply(double value, const NoiseParams& params) override;
    double ApplyRangeRule(double value, const RangeNoiseRule& rule) override;
    void Reset() override;
};

std::unique_ptr<INoiseModel> CreateNoiseModel(NoiseType type, double dt_sec = 0.001);

class RangeNoisePipeline {
public:
    explicit RangeNoisePipeline(std::unique_ptr<INoiseModel> model = nullptr);

    void Clear();
    void AddRule(const RangeNoiseRule& rule);
    double Apply(double value) const;

private:
    std::vector<RangeNoiseRule> rules_;
    std::unique_ptr<INoiseModel> model_;
};

struct AxisValue {
    double x {0.0};
    double y {0.0};
    double z {0.0};
};

class AxisNoisePipeline {
public:
    explicit AxisNoisePipeline(const AxisNoiseParams& params, double dt_sec = 0.001);

    AxisValue Apply(const AxisValue& value) const;
    void Reset();

private:
    AxisNoiseParams params_;
    std::unique_ptr<INoiseModel> model_x_;
    std::unique_ptr<INoiseModel> model_y_;
    std::unique_ptr<INoiseModel> model_z_;
};

}  // namespace noise
}  // namespace sensor
}  // namespace robots
}  // namespace hako
