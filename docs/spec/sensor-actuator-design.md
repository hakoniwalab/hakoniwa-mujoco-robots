# Sensor/Actuator PDU Design

This note describes the current design boundary between MuJoCo sensor/actuator
implementations and Hakoniwa PDU I/O.

Configuration schemas are documented separately in
`sensor-actuator-config-schema.md`.

## Goals

- Keep MuJoCo sensor/actuator implementations independent from PDU read/write
  details where practical.
- Put Hakoniwa PDU data conversion in one predictable place.
- Keep application code simple by wiring sensors, actuators, converters, and
  endpoints through small adapters.
- Organize PDU support by ROS message package name.

## Directory Layout

PDU support is split into two layers under `include/hakoniwa/pdu/`.

```text
include/hakoniwa/pdu/
  converter/
    <ros_package>/<message>.hpp
  adapter/
    <ros_package>/<message>.hpp
```

Examples:

```text
include/hakoniwa/pdu/converter/sensor_msgs/image.hpp
include/hakoniwa/pdu/adapter/sensor_msgs/image.hpp
include/hakoniwa/pdu/converter/std_msgs/color_rgba.hpp
include/hakoniwa/pdu/adapter/std_msgs/color_rgba.hpp
```

The package directory name should match the ROS message package, such as
`sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`, or `tf2_msgs`.

## Responsibilities

### Sensor/Actuator Layer

Sensor and actuator classes expose domain-level data structures and behavior.
They should not own Hakoniwa endpoint I/O.

Examples:

- `ImageFrame`
- `DepthFrame`
- `RGBAColor`
- `UltrasonicFrame`
- `JointActuatorTarget`
- `Tb3Command`

This layer may provide sensor-native convenience helpers when they are not PDU
specific. For example, `TryExtractRGBAColor()` extracts a normalized color from
an `ImageFrame`, and `ICameraSensor::CaptureAsRGBA()` captures the camera and
returns that internal color type.

### Converter Layer

Converters translate between domain-level data and Hakoniwa C++ PDU types.
They do not read from or write to endpoints.

Examples:

- `ImageFrame` to `HakoCpp_Image`
- `DepthFrame` to `HakoCpp_Image`
- `CameraConfig` to `HakoCpp_CameraInfo`
- `UltrasonicFrame` to `HakoCpp_Range`
- `RGBAColor` to `HakoCpp_ColorRGBA`
- `JointActuatorTarget` from `HakoCpp_Float64`

Format handling, validation, timestamp conversion, byte layout, and ROS message
field mapping belong here.

### Adapter Layer

Adapters own Hakoniwa endpoint access via `TypedEndpoint`.

Adapters should be thin:

- receive or send PDU data
- call the appropriate converter
- optionally apply received data to an actuator or command target

Adapters should not duplicate conversion logic.

## Data Ownership

For actuator command PDUs, the current assumption is that a PDU has one logical
owner. Actuator adapters therefore focus on receiving command data and applying
it to actuators. They should not also publish the same command PDU unless a
specific use case requires a separate output channel.

## Adding A New Sensor PDU

1. Define or reuse a sensor-domain data type under the relevant sensor module.
2. Add `include/hakoniwa/pdu/converter/<package>/<message>.hpp`.
3. Add `include/hakoniwa/pdu/adapter/<package>/<message>.hpp`.
4. Keep validation and field mapping in the converter.
5. Keep endpoint I/O in the adapter.
6. Add focused tests for conversion behavior.
7. Wire the adapter from the application layer or example code.

## Adding A New Actuator PDU

1. Define an actuator-domain command/target type.
2. Add a converter for the incoming PDU message type.
3. Add an adapter that receives the PDU and applies the target to the actuator.
4. Avoid putting PDU ownership or endpoint behavior inside the actuator itself.

## Current PDU Mappings

Current sensor/actuator mappings include:

- `sensor_msgs/Image`
- `sensor_msgs/CameraInfo`
- `sensor_msgs/Range`
- `sensor_msgs/Imu`
- `sensor_msgs/LaserScan`
- `sensor_msgs/JointState`
- `nav_msgs/Odometry`
- `tf2_msgs/TFMessage`
- `geometry_msgs/Twist`
- `std_msgs/Float64`
- `std_msgs/ColorRGBA`

## Compatibility Notes

Legacy camera PDU helper files were removed because they duplicated the new
converter responsibility:

- `include/sensors/camera/camera_pdu_converter.hpp`
- `src/sensors/camera/camera_pdu_converter.cpp`

Tests should include the new converter headers directly.

## Known Exceptions

- `forklift_operation_adapter.hpp` still uses an older adapter style and is not
  part of the current sensor/actuator cleanup.
- `ImpulseCollision` and some disturbance/rigid-body PDU paths still have direct
  PDU coupling. They should be handled separately if those areas are brought
  under the same design.
- `std_msgs/ColorRGBA` converter and adapter support exists, but application or
  config wiring should be handled as a separate task when a concrete output PDU
  channel is needed.
