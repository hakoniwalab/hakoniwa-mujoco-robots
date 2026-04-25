#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace hako::robots::sensor::noise
{
    struct Range
    {
        double min {0.0};
        double max {0.0};
    };

    struct RangeNoiseRule
    {
        Range range {};
        bool distance_dependent {false};
        double percentage {0.0};
        double stddev {0.0};
        std::string distribution {"Gaussian"};
    };

    class INoiseModel
    {
    public:
        virtual ~INoiseModel() = default;
        virtual float Apply(float value, const RangeNoiseRule& rule) = 0;
    };

    class GaussianNoiseModel : public INoiseModel
    {
    public:
        GaussianNoiseModel();
        float Apply(float value, const RangeNoiseRule& rule) override;

    private:
        std::mt19937 rng_;
    };

    class RangeNoisePipeline
    {
    public:
        explicit RangeNoisePipeline(std::unique_ptr<INoiseModel> default_model = nullptr);

        void Clear();
        void AddRule(const RangeNoiseRule& rule);
        float Apply(float value) const;

    private:
        std::vector<RangeNoiseRule> rules_;
        std::unique_ptr<INoiseModel> default_model_;
    };
}
