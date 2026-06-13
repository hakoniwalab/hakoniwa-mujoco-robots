# Sensor/Actuator Config Schemas

This document explains the JSON config schemas used by Hakoniwa MuJoCo sensor
and actuator components.

The schema files live under:

```text
config/sensors/schema/
config/actuator/schema/
```

Sample configs live under:

```text
config/sensors/
config/actuator/
```

## Config Categories

There are two main categories.

For user-facing explanations, it is useful to split each config into three
conceptual containers:

- `spec`: the physical or behavioral specification
- `mjcf_binding`: names of MJCF objects used by the runtime
- `pdu_config`: PDU channel and communication-rate settings

Current JSON files do not always use those exact container keys. Many `spec`
fields are top-level fields, and `mjcf_binding` is currently represented by
`RuntimeBinding`.

### Sensor Profiles

Sensor profile configs describe a concrete sensor model or sensor behavior.
They are close to SDF-style sensor definitions, but they are resolved for this
MuJoCo runtime.

Examples:

- camera image size, format, clip range, field of view
- ultrasonic detection range and cone shape
- 2D LiDAR scan angle and distance accuracy
- GPS, contact, force/torque profile settings

### PDU Output Configs

PDU output configs describe how MuJoCo/runtime data should be exposed as named
sensor outputs and PDU channels.

Examples:

- IMU output from a MuJoCo body
- joint state output from MJCF joints
- odometry output from a source body
- TF output from source bodies

These configs normally include `type`, `name`, `pdu_name`, and
`update_rate_hz`.

### Actuator Profiles

Actuator profile configs describe how an actuator command should be applied to
a MuJoCo joint or actuator.

Examples:

- target joint name
- control mode: `position`, `velocity`, or `torque`
- optional effort/velocity limits
- optional damping/friction metadata
- optional MJCF actuator binding

## Common Fields

### `$schema`

Points to the schema file used by tooling and editors. It is optional in some
schemas, but sample configs should include it.

Examples:

```json
"$schema": "../schema/camera.schema.json"
```

```json
"$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json"
```

### `frame_id`

ROS-compatible frame name for sensor output data.

Examples:

- `camera_rgb_frame`
- `camera_depth_frame`
- `laser`
- `imu_link`

### `update_rate` / `UpdateRate` / `update_rate_hz`

Frequency in Hz.

The repository currently has both SDF-like profile naming and PDU output
config naming:

- `update_rate`: camera-style profiles
- `UpdateRate`: ultrasonic-style profiles
- `update_rate_hz`: PDU output configs

Prefer matching the existing schema for the sensor type being edited.

### MJCF Binding (`RuntimeBinding`)

Optional block that connects a profile to MuJoCo object names. In user-facing
docs, this is the MJCF binding block.

Common fields:

- `config_style`: currently `hakoniwa-sdf-like`
- `runtime_source`: currently `mjcf`
- `parent_body`: parent MuJoCo body name
- `source_body`: source MuJoCo body name
- `source_site`: source MuJoCo site name
- `actuator_name`: MJCF actuator name for joint actuators

## Sensor Profile Schemas

### Camera

Schema:

```text
config/sensors/schema/camera.schema.json
```

Samples:

```text
config/sensors/camera/sample_camera.json
config/sensors/color_camera/simple-color-camera.json
```

Key fields:

- `frame_id`: output frame
- `update_rate`: capture rate in Hz
- `horizontal_fov`: horizontal field of view in radians
- `image.width`: image width in pixels
- `image.height`: image height in pixels
- `image.format`: `R8G8B8`, `B8G8R8`, or `L8`
- `clip.near`: near clip distance in meters
- `clip.far`: far clip distance in meters
- `noise`: optional Gaussian noise metadata

PDU mapping:

- `sensor_msgs/Image`
- `sensor_msgs/CameraInfo`
- optionally `std_msgs/ColorRGBA` when a pixel color is extracted

### Depth Camera

Schema:

```text
config/sensors/schema/depth-camera.schema.json
```

Sample:

```text
config/sensors/camera/sample_depth_camera.json
```

Key fields:

- same base fields as camera
- `image.format`: `DEPTH_F32_M` or `DEPTH_U16_MM`

Runtime representation:

- depth sensor data is stored internally as float meters
- `DEPTH_U16_MM` is handled as a downstream serialization/storage hint

PDU mapping:

- `sensor_msgs/Image`
- `sensor_msgs/CameraInfo`

### RGBD Camera

Schema:

```text
config/sensors/schema/rgbd-camera.schema.json
```

Sample:

```text
config/sensors/camera/sample_rgbd_camera.json
```

Key fields:

- `rgb`: standard camera profile
- `depth`: depth camera profile

PDU mapping:

- RGB stream: `sensor_msgs/Image`
- depth stream: `sensor_msgs/Image`
- camera info streams: `sensor_msgs/CameraInfo`

### Multi-camera

Schema:

```text
config/sensors/schema/multicamera.schema.json
```

Sample:

```text
config/sensors/camera/sample_multicamera.json
```

Key fields:

- `cameras[]`
- `cameras[].name`
- `cameras[].pose.position`
- `cameras[].pose.orientation`
- `cameras[].camera_profile`

The `camera_profile` may be a standard camera profile or an RGBD camera profile.

### Ultrasonic

Schema:

```text
config/sensors/schema/ultrasonic.schema.json
```

Sample:

```text
config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

Key fields:

- `spec.frame_id`
- `spec.RadiationType`: `ultrasound` or `infrared`
- `spec.DetectionDistance.Min`
- `spec.DetectionDistance.Max`
- `spec.DistanceAccuracy[]`
- `spec.Cone.Horizontal`
- `spec.Cone.Vertical`
- `spec.Cone.RayCount`
- `spec.UpdateRate`
- `mjcf_binding.source_site`
- `pdu_config.pdu_name`
- `pdu_config.update_rate_hz`
- `pdu_config.message_type`

PDU mapping:

- `sensor_msgs/Range`

### 2D LiDAR

Schema:

```text
config/sensors/schema/lidar-2d.schema.json
```

Samples:

```text
config/sensors/lidar/lds-01.json
config/sensors/lidar/lds-02.json
config/sensors/lidar/urg-04lx-ug01.json
```

Key fields:

- `frame_id`
- `DetectionDistance.Min`
- `DetectionDistance.Max`
- `DistanceAccuracy[]`
- `AngleRange.Min`
- `AngleRange.Max`
- `AngleRange.Resolution`
- `AngleRange.ScanFrequency`
- `AngleRange.AscendingOrderOfData`
- optional `RuntimeBinding`

PDU mapping:

- `sensor_msgs/LaserScan`

### GPS

Schema:

```text
config/sensors/schema/gps.schema.json
```

Sample:

```text
config/sensors/gps/sample_gps.json
```

Key fields:

- `frame_id`
- `update_rate`
- optional position/velocity noise blocks

### Contact

Schema:

```text
config/sensors/schema/contact.schema.json
```

Sample:

```text
config/sensors/contact/sample_contact.json
```

Key fields:

- `frame_id`
- `update_rate`
- `collision_name`

### Force/Torque

Schema:

```text
config/sensors/schema/force-torque.schema.json
```

Sample:

```text
config/sensors/force_torque/sample_force_torque.json
```

Key fields:

- `frame_id`
- `update_rate`
- `joint_name`
- `frame`: `parent`, `child`, or `sensor`
- `measure_direction`: `parent_to_child` or `child_to_parent`

## Runtime Output Schemas

### Common Output Binding

Schema:

```text
config/sensors/schema/common-output.schema.json
```

Shared fields:

- `type`: output type
- `name`: runtime output name
- `pdu_name`: Hakoniwa PDU channel name
- `update_rate_hz`: output rate in Hz

### IMU Output

Schema:

```text
config/sensors/schema/imu-output.schema.json
```

Sample:

```text
config/sensors/imu/tb3-imu.json
```

Key fields:

- common output fields
- `frame_id`
- `parent_body`
- `source_body`
- `mode`: currently `ground_truth`
- optional angular velocity and linear acceleration noise

PDU mapping:

- `sensor_msgs/Imu`

### Joint State Output

Schema:

```text
config/sensors/schema/joint-state-output.schema.json
```

Sample:

```text
config/sensors/joint_state/tb3-wheel-joint-states.json
```

Key fields:

- common output fields
- `joints[]`
- `joints[].name`: ROS joint name
- `joints[].mjcf_joint`: MJCF joint name

PDU mapping:

- `sensor_msgs/JointState`

### Odometry Output

Schema:

```text
config/sensors/schema/odometry-output.schema.json
```

Sample:

```text
config/sensors/odometry/tb3-ground-truth-odom.json
```

Key fields:

- common output fields
- `frame_id`
- `child_frame_id`
- `source_body`
- `mode`: `ground_truth` or `encoder`

PDU mapping:

- `nav_msgs/Odometry`

### TF Output

Schema:

```text
config/sensors/schema/tf-output.schema.json
```

Sample:

```text
config/sensors/tf/tb3-basic-tf.json
```

Key fields:

- common output fields
- `transforms[]`
- `transforms[].parent_frame_id`
- `transforms[].child_frame_id`
- `transforms[].source_body`

PDU mapping:

- `tf2_msgs/TFMessage`

### TB3 Basic Sensor Outputs

Schema:

```text
config/sensors/schema/tb3-basic-sensors.schema.json
```

This schema aggregates PDU output configs for the TurtleBot3 sample.

Key fields:

- `version`
- `config_style`: `hakoniwa-sdf-like`
- `runtime_source`: `mjcf`
- `model.name`
- `model.base_body`
- `outputs[]`: IMU, joint state, odometry, or TF output entries

## Actuator Schemas

### Joint Actuator

Schema:

```text
config/actuator/schema/joint-actuator.schema.json
```

Samples:

```text
config/actuator/joint/sample_joint_actuator.json
config/actuator/joint/sample_position_actuator.json
config/actuator/joint/sample_velocity_actuator.json
config/actuator/joint/tb3_left_wheel.json
config/actuator/joint/tb3_right_wheel.json
```

Key fields:

- `spec.joint_name`: target MJCF joint name
- `spec.type`: `position`, `velocity`, or `torque`
- `spec.limit.lower`
- `spec.limit.upper`
- `spec.limit.effort`
- `spec.limit.velocity`
- `spec.dynamics.damping`
- `spec.dynamics.friction`
- `mjcf_binding.actuator_name`

Backward-compatible top-level `joint_name` / `type` / `limit` / `dynamics` and
`RuntimeBinding.actuator_name` are still accepted by the schema and loader, but
new configs should use `spec` and `mjcf_binding`.

PDU mapping:

- command input is currently represented by `std_msgs/Float64`
- the adapter converts it to `JointActuatorTarget`

## Noise Schemas

Shared noise schemas:

```text
config/sensors/schema/noise-model.schema.json
config/sensors/schema/axis-noise.schema.json
```

Supported generic noise types:

- `none`
- `gaussian`
- `gaussian_quantized`

Common fields:

- `mean`
- `stddev`
- `bias_mean`
- `bias_stddev`
- `dynamic_bias_stddev`
- `dynamic_bias_correlation_time`
- `precision`

## Conventions

- Keep profile configs under `config/sensors/<sensor_type>/` or
  `config/actuator/<actuator_type>/`.
- Keep reusable schemas under `config/sensors/schema/` or
  `config/actuator/schema/`.
- Use lower snake case for new schema filenames.
- Match existing field names for the sensor family being edited.
- Add a sample config when adding a new schema.
- Update this document when adding or renaming a schema.

## Relation To PDU Design

These schemas describe configuration. They do not perform PDU conversion.

PDU conversion and endpoint I/O are documented separately in
[`sensor-actuator-design.md`](sensor-actuator-design.md).
