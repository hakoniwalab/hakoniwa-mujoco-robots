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

    struct JointBinding
    {
        std::string name {};
        std::string mjcf_joint {};
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

    struct ImuNoiseConfig
    {
        AxisNoiseConfig angular_velocity {};
        AxisNoiseConfig linear_acceleration {};
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

    struct ImuConfig
    {
        OutputBinding output {};
        LinkBinding link {};
        std::string mode {"ground_truth"};
        ImuNoiseConfig noise {};
    };

    struct ImuFrame
    {
        MessageHeader header {};
        Quaternion orientation {};
        hako::robots::types::Vector3 angular_velocity {};
        hako::robots::types::Vector3 linear_acceleration {};
    };

    struct JointStateConfig
    {
        OutputBinding output {};
        std::vector<JointBinding> joints {};
    };

    struct JointStateFrame
    {
        MessageHeader header {};
        std::vector<std::string> names {};
        std::vector<double> position {};
        std::vector<double> velocity {};
        std::vector<double> effort {};
    };

    struct OdometryConfig
    {
        OutputBinding output {};
        std::string frame_id {"odom"};
        std::string child_frame_id {"base_footprint"};
        std::string source_body {};
        std::string mode {"ground_truth"};
    };

    struct OdometryFrame
    {
        MessageHeader header {};
        std::string child_frame_id {"base_footprint"};
        Pose3D pose {};
        Twist3D twist {};
    };

    struct TfConfig
    {
        OutputBinding output {};
        std::vector<TransformBinding> transforms {};
    };

    struct TransformFrame
    {
        MessageHeader header {};
        std::string child_frame_id {};
        Pose3D transform {};
    };

    struct TfFrame
    {
        std::vector<TransformFrame> transforms {};
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

    class ILidarSensor2D : public ISensor
    {
    public:
        virtual ~ILidarSensor2D() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual void SetRuntimeOptions(double yaw_bias_deg, double origin_offset_m) = 0;
        virtual const LiDAR2DConfig& GetConfig() const = 0;

        // Capture one full scan frame according to the current config.
        virtual void Scan(LaserScanFrame& out) = 0;
    };

    class IImuSensor : public ISensor
    {
    public:
        virtual ~IImuSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const ImuConfig& GetConfig() const = 0;
        virtual void Read(ImuFrame& out) = 0;
    };

    class IJointStateSensor : public ISensor
    {
    public:
        virtual ~IJointStateSensor() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const JointStateConfig& GetConfig() const = 0;
        virtual void Read(JointStateFrame& out) = 0;
    };

    class IOdometryPublisher : public IStatePublisher
    {
    public:
        virtual ~IOdometryPublisher() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const OdometryConfig& GetConfig() const = 0;
        virtual void Build(OdometryFrame& out) = 0;
    };

    class ITfPublisher : public IStatePublisher
    {
    public:
        virtual ~ITfPublisher() = default;

        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual const TfConfig& GetConfig() const = 0;
        virtual void Build(TfFrame& out) = 0;
    };
}
