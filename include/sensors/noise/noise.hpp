#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace hako::robots::sensor::noise
{
    // -----------------------------------------------------------------------
    // noise-model.schema 対応
    // -----------------------------------------------------------------------
    enum class NoiseType
    {
        None,
        Gaussian,
        GaussianQuantized,
    };

    struct NoiseParams
    {
        NoiseType type                       {NoiseType::None};
        double    mean                       {0.0};
        double    stddev                     {0.0};
        double    bias_mean                  {0.0};
        double    bias_stddev                {0.0};
        double    dynamic_bias_stddev        {0.0};
        double    dynamic_bias_correlation_time {0.0};  // seconds
        double    precision                  {0.0};     // gaussian_quantized 用
    };

    // -----------------------------------------------------------------------
    // axis-noise.schema 対応（IMU など軸ごとに独立したノイズ）
    // -----------------------------------------------------------------------
    struct AxisNoiseParams
    {
        NoiseParams x {};
        NoiseParams y {};
        NoiseParams z {};
    };

    // -----------------------------------------------------------------------
    // lidar-2d.schema 対応（距離範囲ごとのノイズルール）
    // -----------------------------------------------------------------------
    struct Range
    {
        double min {0.0};
        double max {0.0};
    };

    struct RangeNoiseRule
    {
        Range       range              {};
        bool        distance_dependent {false};
        double      percentage         {0.0};   // dependent 時に使用
        NoiseParams noise              {};       // independent / dependent 共通
    };

    // -----------------------------------------------------------------------
    // ノイズモデル基底
    // Apply はステートレスに見えるが、dynamic_bias はモデル内部で状態保持
    // -----------------------------------------------------------------------
    class INoiseModel
    {
    public:
        virtual ~INoiseModel() = default;

        // 汎用インタフェース（IMU・Odometry など）
        virtual double Apply(double value, const NoiseParams& params) = 0;

        // LiDAR 向けレンジルールつきインタフェース
        virtual double ApplyRangeRule(double value, const RangeNoiseRule& rule) = 0;

        // dynamic_bias のランダムウォーク状態をリセット
        virtual void Reset() = 0;
    };

    // -----------------------------------------------------------------------
    // Gaussian / GaussianQuantized 実装
    // dynamic_bias はウィーナー過程で更新（correlation_time でローパス）
    // -----------------------------------------------------------------------
    class GaussianNoiseModel : public INoiseModel
    {
    public:
        explicit GaussianNoiseModel(double dt_sec = 0.001);

        double Apply(double value, const NoiseParams& params) override;
        double ApplyRangeRule(double value, const RangeNoiseRule& rule) override;
        void   Reset() override;

    private:
        double                               dt_;
        double                               current_bias_ {0.0};
        std::mt19937                         rng_;
        std::normal_distribution<double>     dist_normal_ {0.0, 1.0};

        double SampleGaussian(double mean, double stddev);
        double UpdateDynamicBias(const NoiseParams& params);
        double Quantize(double value, double precision);
    };

    // -----------------------------------------------------------------------
    // パススルー（type: none 用）
    // -----------------------------------------------------------------------
    class PassthroughNoiseModel : public INoiseModel
    {
    public:
        double Apply(double value, const NoiseParams&) override       { return value; }
        double ApplyRangeRule(double value, const RangeNoiseRule&) override { return value; }
        void   Reset() override {}
    };

    // -----------------------------------------------------------------------
    // ファクトリ
    // -----------------------------------------------------------------------
    std::unique_ptr<INoiseModel> CreateNoiseModel(NoiseType type, double dt_sec = 0.001);

    // -----------------------------------------------------------------------
    // LiDAR 用パイプライン（範囲ルールリストを順に評価）
    // -----------------------------------------------------------------------
    class RangeNoisePipeline
    {
    public:
        explicit RangeNoisePipeline(std::unique_ptr<INoiseModel> model = nullptr);

        void   Clear();
        void   AddRule(const RangeNoiseRule& rule);
        double Apply(double value) const;

    private:
        std::vector<RangeNoiseRule>  rules_;
        std::unique_ptr<INoiseModel> model_;
    };

    // -----------------------------------------------------------------------
    // IMU / Odometry 用パイプライン（軸ごとに独立したノイズ）
    // -----------------------------------------------------------------------
    struct AxisValue
    {
        double x {0.0};
        double y {0.0};
        double z {0.0};
    };

    class AxisNoisePipeline
    {
    public:
        explicit AxisNoisePipeline(const AxisNoiseParams& params,
                                   double dt_sec = 0.001);

        AxisValue Apply(const AxisValue& value) const;
        void      Reset();

    private:
        AxisNoiseParams                      params_;
        std::unique_ptr<INoiseModel>         model_x_;
        std::unique_ptr<INoiseModel>         model_y_;
        std::unique_ptr<INoiseModel>         model_z_;
    };

} // namespace hako::robots::sensor::noise