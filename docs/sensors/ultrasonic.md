# Hakoniwa Ultrasonic Sensor Schema

This document describes the JSON schema for Hakoniwa ultrasonic / sonar range sensor profiles.

The ultrasonic sensor profile is an SDF-like, MJCF-resolved configuration format used by Hakoniwa MuJoCo-based sensors.  
It models a single-value distance sensor such as LEGO SPIKE Prime Distance Sensor, generic ultrasonic sensors, or simple proximity sensors.

Unlike 2D LiDAR, this sensor does not output an array of scan points.  
It outputs one range value per update.

## Concept

An ultrasonic sensor is modeled as a cone-shaped detection volume.

Internally, the MuJoCo implementation may approximate this cone using one or more ray casts.  
The final output is a single distance value, typically the nearest valid hit inside the detection volume.

```text
sensor origin
    |
    |---- cone / ray casting volume ---->
    |
output: single range value [m]
````

## Top-level Fields

```json
{
  "$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json",
  "frame_id": "lego_spike_distance_sensor",
  "DetectionDistance": {
    "Min": 0.05,
    "Max": 2.0
  },
  "DistanceAccuracy": [
    {
      "Range": {
        "Min": 0.05,
        "Max": 2.0
      },
      "StdDev": 0.01,
      "Precision": 0.001,
      "NoiseDistribution": "gaussian"
    }
  ],
  "Cone": {
    "Horizontal": 1.221730476,
    "Vertical": 1.221730476,
    "RayCount": 9
  },
  "UpdateRate": 100.0
}
```

## Field Reference

### `$schema`

Type: `string`
Required: no
Unit: none

JSON Schema identifier.

Example:

```json
"$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json"
```

---

### `frame_id`

Type: `string`
Required: yes
Unit: none

Coordinate frame name of the ultrasonic sensor.

This should match the logical sensor frame used by Hakoniwa PDU, ROS-compatible messages, or the resolved MJCF site/body binding.

Example:

```json
"frame_id": "front_ultrasonic_link"
```

Recommended naming examples:

```text
front_ultrasonic_link
rear_ultrasonic_link
lego_spike_distance_sensor
spike_distance_sensor_link
```

---

### `DetectionDistance`

Type: `object`
Required: yes
Unit: meter `[m]`

Defines the measurable distance range of the sensor.

```json
"DetectionDistance": {
  "Min": 0.05,
  "Max": 2.0
}
```

#### `DetectionDistance.Min`

Type: `number`
Unit: meter `[m]`
Range: `>= 0`

Minimum valid measurable distance.

Values below this distance are considered invalid, clamped, or implementation-defined depending on the runtime behavior.

Example:

```json
"Min": 0.05
```

This means the sensor cannot reliably measure objects closer than 5 cm.

#### `DetectionDistance.Max`

Type: `number`
Unit: meter `[m]`
Range: `> 0`

Maximum valid measurable distance.

If no object is detected within the sensor volume, the runtime may return this value as the default no-hit distance.

Example:

```json
"Max": 2.0
```

This means the sensor can measure up to 2 meters.

---

### `DistanceAccuracy`

Type: `array`
Required: yes
Unit: meter `[m]`

Defines distance measurement noise and precision.

The array format allows different accuracy models for different distance ranges.

Example:

```json
"DistanceAccuracy": [
  {
    "Range": {
      "Min": 0.05,
      "Max": 2.0
    },
    "StdDev": 0.01,
    "Precision": 0.001,
    "NoiseDistribution": "gaussian"
  }
]
```

---

### `DistanceAccuracy[].Range`

Type: `object`
Required: yes
Unit: meter `[m]`

Defines the distance interval where this accuracy model is applied.

Example:

```json
"Range": {
  "Min": 0.05,
  "Max": 2.0
}
```

Normally this range should be within `DetectionDistance.Min` and `DetectionDistance.Max`.

---

### `DistanceAccuracy[].StdDev`

Type: `number`
Required: yes
Unit: meter `[m]`
Range: `>= 0`

Standard deviation of measurement noise.

Example:

```json
"StdDev": 0.01
```

This means Gaussian noise with a standard deviation of 1 cm.

For a simple ultrasonic sensor, typical values may be:

```text
0.005  = 5 mm
0.01   = 1 cm
0.02   = 2 cm
```

---

### `DistanceAccuracy[].Precision`

Type: `number`
Required: no
Unit: meter `[m]`
Range: `>= 0`

Measurement resolution or quantization step.

Example:

```json
"Precision": 0.001
```

This means the measured distance is quantized to 1 mm resolution.

If omitted, the runtime may use continuous floating-point output without quantization.

---

### `DistanceAccuracy[].NoiseDistribution`

Type: `string`
Required: yes
Unit: none

Noise distribution type.

Recommended values:

```text
gaussian
uniform
none
```

Current recommended default:

```json
"NoiseDistribution": "gaussian"
```

For initial Hakoniwa ultrasonic sensor implementation, `gaussian` is sufficient.

---

### `Cone`

Type: `object`
Required: yes
Unit: radians for angles

Defines the ultrasonic detection volume.

The sensor output is still a single range value, even when the detection volume is cone-shaped.

Example:

```json
"Cone": {
  "Horizontal": 1.221730476,
  "Vertical": 1.221730476,
  "RayCount": 9
}
```

---

### `Cone.Horizontal`

Type: `number`
Required: yes
Unit: radian `[rad]`
Range: `> 0`

Horizontal field of view of the ultrasonic cone.

This is the full angle, not half angle.

Example:

```json
"Horizontal": 1.221730476
```

This is approximately 70 degrees.

```text
70 deg = 1.221730476 rad
```

If a sensor datasheet says ±35 degrees, the schema value should be 70 degrees in radians.

---

### `Cone.Vertical`

Type: `number`
Required: yes
Unit: radian `[rad]`
Range: `> 0`

Vertical field of view of the ultrasonic cone.

This is also the full angle.

Example:

```json
"Vertical": 1.221730476
```

For many simple ultrasonic sensors, using the same value for horizontal and vertical FOV is acceptable.

---

### `Cone.RayCount`

Type: `integer`
Required: yes
Unit: count
Range: `>= 1`

Number of rays used to approximate the ultrasonic cone in the MuJoCo runtime.

Example:

```json
"RayCount": 9
```

Recommended values:

```text
1   center ray only
5   center + up/down/left/right
9   simple cone approximation
13+ higher fidelity approximation
```

Initial recommended value:

```json
"RayCount": 9
```

Runtime interpretation:

```text
RayCount == 1:
  Use a single forward ray.

RayCount > 1:
  Place multiple rays inside the cone.
  Return the nearest valid hit distance.
```

---

### `UpdateRate`

Type: `number`
Required: yes
Unit: hertz `[Hz]`
Range: `> 0`

Sensor update frequency.

Example:

```json
"UpdateRate": 100.0
```

This means the sensor produces a new measurement at 100 Hz.

If the physics simulation runs at 1000 Hz and the sensor update rate is 100 Hz, the sensor is updated once every 10 simulation steps.

```text
physics timestep: 0.001 sec
physics rate:     1000 Hz
sensor rate:      100 Hz
sensor period:    0.01 sec
```

---

## Optional Runtime Binding

A runtime implementation may support `RuntimeBinding` to resolve the sensor profile against MJCF bodies or sites.

Example:

```json
"RuntimeBinding": {
  "config_style": "hakoniwa-sdf-like",
  "runtime_source": "mjcf",
  "parent_body": "base_link",
  "source_site": "spike_distance_sensor_site"
}
```

### `RuntimeBinding.config_style`

Type: `string`
Required: no
Allowed value:

```text
hakoniwa-sdf-like
```

Indicates that the config follows Hakoniwa's SDF-like sensor model.

### `RuntimeBinding.runtime_source`

Type: `string`
Required: no
Allowed value:

```text
mjcf
```

Indicates that the runtime pose/source is resolved from MJCF.

### `RuntimeBinding.parent_body`

Type: `string`
Required: no

Name of the parent MJCF body.

Example:

```json
"parent_body": "base_link"
```

### `RuntimeBinding.source_body`

Type: `string`
Required: no

Name of the MJCF body used as the sensor source frame.

### `RuntimeBinding.source_site`

Type: `string`
Required: no

Name of the MJCF site used as the sensor origin and orientation.

For MuJoCo-based sensors, using `source_site` is recommended.

Example:

```json
"source_site": "front_ultrasonic_site"
```

### `RuntimeBinding.frame_id_override`

Type: `string`
Required: no

Overrides the top-level `frame_id` when publishing sensor data.

---

## Units Summary

| Field                          | Unit           |
| ------------------------------ | -------------- |
| `DetectionDistance.Min`        | meter `[m]`    |
| `DetectionDistance.Max`        | meter `[m]`    |
| `DistanceAccuracy[].Range.Min` | meter `[m]`    |
| `DistanceAccuracy[].Range.Max` | meter `[m]`    |
| `DistanceAccuracy[].StdDev`    | meter `[m]`    |
| `DistanceAccuracy[].Precision` | meter `[m]`    |
| `Cone.Horizontal`              | radian `[rad]` |
| `Cone.Vertical`                | radian `[rad]` |
| `Cone.RayCount`                | count          |
| `UpdateRate`                   | hertz `[Hz]`   |

---

## Runtime Behavior

The recommended default runtime behavior is:

```text
1. Resolve the sensor origin and direction from frame_id or RuntimeBinding.
2. Generate RayCount rays inside the cone.
3. Cast rays using the MuJoCo ray-casting API.
4. Select the nearest valid hit distance.
5. Clamp the result to DetectionDistance.Min and DetectionDistance.Max.
6. Apply noise based on DistanceAccuracy.
7. Quantize using Precision if specified.
8. Publish a single range value.
```

No-hit behavior:

```text
If no object is detected within DetectionDistance.Max,
return DetectionDistance.Max.
```

This keeps the first implementation simple and deterministic.

---

## Recommended PDU Mapping

The ultrasonic sensor should be mapped to a single-value range PDU.

Example conceptual PDU:

```text
hako_msgs/Range
  header
  radiation_type
  field_of_view
  min_range
  max_range
  range
  variance
  status
```

Recommended mapping:

| Schema field                 | PDU field         |
| ---------------------------- | ----------------- |
| `frame_id`                   | `header.frame_id` |
| `DetectionDistance.Min`      | `min_range`       |
| `DetectionDistance.Max`      | `max_range`       |
| `Cone.Horizontal` or max FOV | `field_of_view`   |
| measured distance            | `range`           |
| `StdDev * StdDev`            | `variance`        |

For ultrasonic sensors, `radiation_type` should be equivalent to:

```text
ULTRASOUND
```

or ROS-compatible:

```text
ULTRASOUND = 0
```

---

## LEGO SPIKE Distance Sensor Example

The LEGO SPIKE Prime Distance Sensor can be modeled as an ultrasonic range sensor.

Representative values:

```text
Detection range: 0.05 m to 2.0 m
Accuracy:        approximately ±0.02 m
Resolution:      0.001 m
Update rate:     100 Hz
Field of view:   ±35 deg
```

Since the schema uses full FOV in radians:

```text
±35 deg = 70 deg full angle
70 deg = 1.221730476 rad
```

Example profile:

```json
{
  "$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json",
  "frame_id": "lego_spike_distance_sensor",

  "DetectionDistance": {
    "Min": 0.05,
    "Max": 2.0
  },

  "DistanceAccuracy": [
    {
      "Range": {
        "Min": 0.05,
        "Max": 2.0
      },
      "StdDev": 0.01,
      "Precision": 0.001,
      "NoiseDistribution": "gaussian"
    }
  ],

  "Cone": {
    "Horizontal": 1.221730476,
    "Vertical": 1.221730476,
    "RayCount": 9
  },

  "UpdateRate": 100.0
}
```

For a stricter datasheet-like model, `StdDev` may be set to `0.02`.

```json
"StdDev": 0.02
```

For a smoother simulation model, `StdDev: 0.01` is recommended.

---

## Design Notes

This schema intentionally keeps the ultrasonic sensor model simple.

The following behaviors are treated as runtime policy, not schema fields:

```text
ray placement pattern
no-hit value
clamping behavior
nearest-hit selection
material reflection model
outlier model
```

The initial Hakoniwa implementation should use fixed defaults:

```text
ray placement:        simple cone grid
hit selection:        nearest valid hit
no-hit value:         DetectionDistance.Max
noise model:          DistanceAccuracy
output type:          single range value
```

This keeps the schema compact while leaving room for future runtime improvements.

Future extensions may add:

```text
material reflection coefficient
angle-dependent reflection loss
dropout probability
outlier probability
multi-echo behavior
temperature-dependent speed of sound
```

These should not be added until there is a concrete simulation requirement.
