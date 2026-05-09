#pragma once

#include <memory>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/common/update_scheduler.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::ultrasonic
{
    /**
     * @brief Distance interval in meters.
     *
     * This structure is used for both the sensor's measurable detection range
     * and the distance interval where a specific accuracy/noise model applies.
     *
     * Unit:
     * - min: meter [m]
     * - max: meter [m]
     *
     * Expected constraints:
     * - min >= 0.0
     * - max > min
     */
    struct DistanceRange
    {
        double min {0.0};
        double max {0.0};
    };

    /**
     * @brief Distance accuracy/noise model for a specific range interval.
     *
     * The ultrasonic sensor may use different noise parameters depending on
     * the measured distance. For example, near range and far range can have
     * different standard deviations.
     *
     * Unit:
     * - range: meter [m]
     * - stddev: meter [m]
     * - precision: meter [m]
     *
     * Semantics:
     * - range:
     *     Distance interval where this accuracy model is applied.
     *     It should normally be inside UltrasonicConfig::detection_distance.
     *
     * - stddev:
     *     Standard deviation of distance noise.
     *     For gaussian noise, this is sigma in meters.
     *
     * - precision:
     *     Optional quantization step of the measured range.
     *     For example, 0.001 means 1 mm resolution.
     *     A value of 0.0 means no explicit quantization.
     *
     * - noise_distribution:
     *     Name of the noise distribution.
     *     Recommended values:
     *       - "gaussian"
     *       - "uniform"
     *       - "none"
     */
    struct DistanceAccuracy
    {
        DistanceRange range {};
        double stddev {0.0};
        double precision {0.0};
        std::string noise_distribution {"gaussian"};
    };

    /**
     * @brief Ultrasonic detection cone model.
     *
     * The ultrasonic sensor is modeled as a cone-shaped detection volume.
     * The runtime may approximate this cone using one or more ray casts.
     *
     * Unit:
     * - horizontal: radian [rad]
     * - vertical: radian [rad]
     * - ray_count: number of rays
     *
     * Semantics:
     * - horizontal:
     *     Full horizontal field-of-view angle in radians.
     *     This is not a half angle.
     *
     * - vertical:
     *     Full vertical field-of-view angle in radians.
     *     This is not a half angle.
     *
     * - ray_count:
     *     Number of rays used to approximate the cone.
     *
     * Suggested runtime behavior:
     * - ray_count == 1:
     *     Use a single forward ray.
     *
     * - ray_count > 1:
     *     Cast multiple rays inside the cone and use the nearest valid hit.
     *
     * Example:
     * - A sensor with ±35 degrees FOV should use:
     *     horizontal = 70 deg = 1.221730476 rad
     *     vertical   = 70 deg = 1.221730476 rad
     */
    struct Cone
    {
        double horizontal {0.0};
        double vertical {0.0};
        int ray_count {1};
    };

    /**
     * @brief Runtime binding information for resolving the sensor in the simulation world.
     *
     * This structure connects the abstract ultrasonic sensor profile to
     * runtime objects such as MJCF bodies or sites.
     *
     * Semantics:
     * - config_style:
     *     Configuration style identifier.
     *     Default: "hakoniwa-sdf-like"
     *
     * - runtime_source:
     *     Runtime model source.
     *     Default: "mjcf"
     *
     * - parent_body:
     *     Parent body name in the runtime model.
     *     This is typically the robot body to which the sensor is attached.
     *
     * - source_site:
     *     Optional site name used as the sensor origin and orientation.
     *     If empty, the implementation may fall back to sensor_body_name or
     *     another runtime-specific frame resolution rule.
     *
     * Note:
     * - For MuJoCo-based sensors, using a site as the sensor origin is recommended.
     */
    struct RuntimeBinding
    {
        std::string config_style {"hakoniwa-sdf-like"};
        std::string runtime_source {"mjcf"};
        std::string parent_body {"base_link"};
        std::string source_site {};
    };

    /**
     * @brief Ultrasonic sensor configuration.
     *
     * This configuration corresponds to the Hakoniwa ultrasonic sensor JSON profile.
     * It describes a single-value ultrasonic / sonar range sensor.
     *
     * The sensor output is one range value per measurement.
     * It does not output a LiDAR-like scan array.
     *
     * Fields:
     * - frame_id:
     *     Logical sensor frame name.
     *     This should be used as the published frame_id in sensor output.
     *
     * - detection_distance:
     *     Valid measurable distance range in meters.
     *
     * - distance_accuracy:
     *     List of distance accuracy/noise models.
     *     Each entry applies to a specific distance range.
     *
     * - cone:
     *     Detection cone definition and ray approximation count.
     *
     * - update_rate:
     *     Sensor update rate in Hz.
     *
     * - runtime_binding:
     *     Optional binding information for resolving the sensor origin in the runtime model.
     *
     * Unit:
     * - detection_distance: meter [m]
     * - cone angles: radian [rad]
     * - update_rate: hertz [Hz]
     *
     * Expected constraints:
     * - detection_distance.max > detection_distance.min
     * - cone.horizontal > 0.0
     * - cone.vertical > 0.0
     * - cone.ray_count >= 1
     * - update_rate > 0.0
     */
    struct UltrasonicConfig
    {
        std::string frame_id {"ultrasonic"};
        DistanceRange detection_distance {};
        std::vector<DistanceAccuracy> distance_accuracy {};
        Cone cone {};
        double update_rate {10.0};
        RuntimeBinding runtime_binding {};
    };

    /**
     * @brief Status of an ultrasonic measurement.
     *
     * Semantics:
     * - OK:
     *     A valid object was detected within the configured detection distance.
     *
     * - NO_HIT:
     *     No object was detected within detection_distance.max.
     *     The reported range is typically detection_distance.max.
     *
     * - BELOW_MIN_RANGE:
     *     A hit was detected closer than detection_distance.min.
     *     The reported range may be clamped to detection_distance.min.
     *
     * - INVALID:
     *     Measurement is not initialized or could not be produced.
     */
    enum class UltrasonicStatus
    {
        OK,
        NO_HIT,
        BELOW_MIN_RANGE,
        INVALID
    };

    /**
     * @brief Output frame of one ultrasonic range measurement.
     *
     * This structure represents one sensor update.
     *
     * Fields:
     * - frame_id:
     *     Frame name of the measurement.
     *     Usually copied from UltrasonicConfig::frame_id.
     *
     * - range:
     *     Measured distance in meters.
     *
     * - variance:
     *     Measurement variance in square meters [m^2].
     *     For gaussian noise, this is typically stddev * stddev.
     *
     * - status:
     *     Measurement status.
     *
     * Unit:
     * - range: meter [m]
     * - variance: meter squared [m^2]
     *
     * Recommended no-hit behavior:
     * - status = NO_HIT
     * - range = detection_distance.max
     */
    struct UltrasonicFrame
    {
        std::string frame_id {"ultrasonic"};
        double range {0.0};
        double variance {0.0};
        UltrasonicStatus status {UltrasonicStatus::INVALID};
    };

    /**
     * @brief Interface for an ultrasonic range sensor.
     *
     * This interface represents a single-value ultrasonic / sonar sensor.
     * The sensor measures one distance value per update according to the
     * current UltrasonicConfig.
     *
     * Implementations are responsible for:
     * - loading configuration,
     * - resolving the sensor origin in the simulation world,
     * - applying update scheduling,
     * - performing ray-based or cone-based measurement,
     * - applying distance noise and precision,
     * - returning an UltrasonicFrame.
     */
    class IUltrasonicSensor : public ISensor
    {
    public:
        virtual ~IUltrasonicSensor() = default;

        /**
         * @brief Load ultrasonic sensor configuration from a JSON file.
         *
         * @param config_path Path to the ultrasonic sensor profile JSON.
         * @return true if the configuration was loaded successfully.
         * @return false if loading or validation failed.
         */
        virtual bool LoadConfig(const std::string& config_path) = 0;

        /**
         * @brief Get the current ultrasonic sensor configuration.
         *
         * @return Const reference to the current UltrasonicConfig.
         */
        virtual const UltrasonicConfig& GetConfig() const = 0;

        /**
         * @brief Capture one ultrasonic range measurement.
         *
         * The output frame should contain a single range value and status.
         *
         * Expected behavior:
         * - If a valid hit is found:
         *     out.status = UltrasonicStatus::OK
         *
         * - If no hit is found:
         *     out.status = UltrasonicStatus::NO_HIT
         *     out.range = config.detection_distance.max
         *
         * - If the hit is below the minimum measurable range:
         *     out.status = UltrasonicStatus::BELOW_MIN_RANGE
         *
         * @param out Output frame to be filled by the implementation.
         */
        virtual void Measure(UltrasonicFrame& out) = 0;
    };

    /**
     * @brief Default ultrasonic sensor implementation.
     *
     * This implementation uses the Hakoniwa physics world abstraction to
     * measure distance from a sensor body or site.
     *
     * The detection cone may be approximated by multiple ray casts depending
     * on UltrasonicConfig::cone.ray_count.
     *
     * The class also owns:
     * - update scheduler,
     * - loaded configuration,
     * - noise pipeline,
     * - runtime body references.
     */
    class UltrasonicSensor : public IUltrasonicSensor
    {
    public:
        /**
         * @brief Construct an ultrasonic sensor.
         *
         * @param world Physics world used for ray casting and body lookup.
         * @param sensor_body_name Name of the sensor body or fallback runtime frame.
         * @param exclude_body_name Body name to exclude from ray hit detection.
         *
         * Note:
         * - sensor_body_name is used as a fallback when RuntimeBinding::source_site
         *   is not specified or cannot be resolved.
         * - exclude_body_name is typically the robot's own base body, so the sensor
         *   does not detect its own body.
         */
        UltrasonicSensor(
            std::shared_ptr<hako::robots::physics::IWorld> world,
            std::string sensor_body_name = "ultrasonic_sensor",
            std::string exclude_body_name = "base_footprint");

        /**
         * @brief Load ultrasonic sensor configuration from JSON.
         *
         * This should initialize:
         * - config_,
         * - scheduler_ using config_.update_rate,
         * - noise_pipeline_ using config_.distance_accuracy.
         *
         * @param config_path Path to the ultrasonic sensor profile JSON.
         * @return true if configuration loading succeeded.
         * @return false otherwise.
         */
        bool LoadConfig(const std::string& config_path) override;

        /**
         * @brief Get the current ultrasonic sensor configuration.
         *
         * @return Const reference to config_.
         */
        const UltrasonicConfig& GetConfig() const override;

        /**
         * @brief Reset internal sensor state.
         *
         * Expected behavior:
         * - reset update scheduler,
         * - clear or reinitialize runtime state if necessary,
         * - keep the loaded configuration unless implementation explicitly reloads it.
         */
        void Reset() override;

        /**
         * @brief Get sensor update period in seconds.
         *
         * This is normally computed from UltrasonicConfig::update_rate.
         *
         * Formula:
         * - period_sec = 1.0 / update_rate
         *
         * @return Update period in seconds.
         */
        double GetUpdatePeriodSec() const override;

        /**
         * @brief Check whether the sensor should produce a new measurement.
         *
         * @param delta_sec Elapsed simulation time since the last scheduler update, in seconds.
         * @return true if the sensor should update now.
         * @return false otherwise.
         */
        bool ShouldUpdate(double delta_sec) override;

        /**
         * @brief Capture one ultrasonic range measurement.
         *
         * Expected runtime algorithm:
         * 1. Resolve sensor origin and orientation.
         * 2. Generate one or more ray directions inside the configured cone.
         * 3. Cast rays into the physics world.
         * 4. Select the nearest valid hit.
         * 5. Apply detection distance limits.
         * 6. Apply configured noise and precision.
         * 7. Fill UltrasonicFrame.
         *
         * @param out Output frame to be filled.
         */
        void Measure(UltrasonicFrame& out) override;

    private:
        /**
         * @brief Physics world used by this sensor.
         */
        std::shared_ptr<hako::robots::physics::IWorld> world_;

        /**
         * @brief Runtime rigid body associated with the sensor.
         *
         * This may be resolved from sensor_body_name_ or runtime binding information.
         */
        std::shared_ptr<hako::robots::physics::IRigidBody> sensor_body_;

        /**
         * @brief Sensor body name or fallback runtime frame name.
         */
        std::string sensor_body_name_;

        /**
         * @brief Body name excluded from ray hit detection.
         *
         * Typically used to avoid detecting the robot's own base body.
         */
        std::string exclude_body_name_;

        /**
         * @brief Loaded ultrasonic sensor configuration.
         */
        UltrasonicConfig config_ {};

        /**
         * @brief Update scheduler for enforcing config_.update_rate.
         */
        common::UpdateScheduler scheduler_ {};

        /**
         * @brief Noise pipeline applied to measured range values.
         */
        noise::RangeNoisePipeline noise_pipeline_;
    };
}