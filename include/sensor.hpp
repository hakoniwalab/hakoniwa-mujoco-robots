#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "primitive_types.hpp"

namespace hako::robots::sensor
{
    struct MessageHeader
    {
        double stamp_sec {0.0};
        std::string frame_id {};
    };

    struct Quaternion
    {
        double x {0.0};
        double y {0.0};
        double z {0.0};
        double w {1.0};
    };

    struct Pose3D
    {
        hako::robots::types::Position position {};
        Quaternion orientation {};
    };

    struct Twist3D
    {
        hako::robots::types::Vector3 linear {};
        hako::robots::types::Vector3 angular {};
    };

    struct OutputBinding
    {
        std::string name {};
        std::string pdu_name {};
        double update_rate_hz {10.0};
    };

    struct LinkBinding
    {
        std::string frame_id {};
        std::string parent_body {};
        std::string source_body {};
    };

    struct TransformBinding
    {
        std::string parent_frame_id {};
        std::string child_frame_id {};
        std::string source_body {};
    };

    struct NoiseModelConfig
    {
        std::string type {"none"};
        double mean {0.0};
        double stddev {0.0};
        double bias_mean {0.0};
        double bias_stddev {0.0};
        double dynamic_bias_stddev {0.0};
        double dynamic_bias_correlation_time {0.0};
        double precision {0.0};
    };

    struct AxisNoiseConfig
    {
        NoiseModelConfig x {};
        NoiseModelConfig y {};
        NoiseModelConfig z {};
    };


    struct DetectionDistance
    {
        double min {0.0};
        double max {0.0};
    };

    struct BlindPaddingRange
    {
        int size {0};
        double value {0.0};
        bool enabled {false};
    };

    struct AngleRange
    {
        double min_deg {0.0};
        double max_deg {0.0};
        bool ascending_order_of_data {true};
        double resolution_deg {1.0};
        int scan_frequency_hz {10};
        BlindPaddingRange blind_padding {};
    };

    struct DistanceAccuracy
    {
        DetectionDistance range {};
        bool distance_dependent {false};
        double percentage {0.0};
        double stddev {0.0};
        std::string noise_distribution {"Gaussian"};
    };

    struct LiDAR2DConfig
    {
        std::string frame_id {"laser"};
        DetectionDistance detection_distance {};
        AngleRange angle_range {};
        std::vector<DistanceAccuracy> distance_accuracy {};
        double yaw_bias_deg {0.0};
        double origin_offset_m {0.0};
    };

    struct LaserScanFrame
    {
        std::string frame_id {"laser"};
        float angle_min {0.0F};
        float angle_max {0.0F};
        float angle_increment {0.0F};
        float time_increment {0.0F};
        float scan_time {0.0F};
        float range_min {0.0F};
        float range_max {0.0F};
        std::vector<float> ranges {};
        std::vector<float> intensities {};
    };

    class ISensor
    {
    public:
        virtual ~ISensor() = default;
        virtual void Reset() = 0;
        virtual double GetUpdatePeriodSec() const = 0;
        virtual bool ShouldUpdate(double delta_sec) = 0;
    };

    class IStatePublisher
    {
    public:
        virtual ~IStatePublisher() = default;
        virtual void Reset() = 0;
        virtual double GetUpdatePeriodSec() const = 0;
        virtual bool ShouldUpdate(double delta_sec) = 0;
    };

}
