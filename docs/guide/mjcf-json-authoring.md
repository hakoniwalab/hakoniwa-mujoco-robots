# MJCF / JSON Authoring Guide

This guide is for users who need to create MuJoCo XML and Hakoniwa sensor /
actuator JSON configs before writing C++ integration code.

The goal is to make each file independently checkable first. If you jump
straight into C++ integration, XML loading errors, JSON mistakes, runtime name
binding errors, PDU issues, and control-loop timing issues are hard to separate.

## Recommended Order

1. Write the MuJoCo MJCF XML.
2. Check that MuJoCo can load the XML.
3. Write the sensor or actuator JSON config.
4. Check JSON syntax and schema.
5. Check that names in JSON match names in MJCF.
6. Run the closest small example or a minimal C++ `LoadConfig()` check.
7. Integrate the component into a robot sample.

## What Goes In MJCF

MJCF defines the physical world and MJCF object names. Hakoniwa
JSON and C++ code use those names to resolve bodies, sites, cameras, joints,
and actuators.

| Purpose | MJCF object | Referenced by |
| --- | --- | --- |
| Read robot pose | `<body name="...">` | body name |
| Read LiDAR pose | `<body name="base_scan">` or similar | sensor body name |
| Ultrasonic origin and direction | `<site name="front_ultrasonic_site">` | `mjcf_binding.source_site` |
| RGB / depth camera | `<camera name="color_camera">` | camera name |
| Joint state | `<joint name="...">` | `joints[].mjcf_joint` |
| Joint actuator | `<position>`, `<velocity>`, `<motor>` | `mjcf_binding.actuator_name` |

Choose names that describe their role, such as `base_link`, `base_scan`,
`front_ultrasonic_site`, `left_wheel_joint`, or `left_wheel_velocity`.

## Sensor MJCF Rules Of Thumb

For sensors, the first question is where the sensor reads from.

The current TB3 LiDAR integration uses a body such as `base_scan`. That body
provides the scan origin and yaw.

For ultrasonic sensors, prefer a `site`. The source site's local `+X` direction
is the range direction.

For cameras, use MuJoCo `<camera name="...">`. The camera name passed to
`CameraSensor` must match the MJCF camera name.

## Actuator MJCF Rules Of Thumb

For a joint actuator, define both the joint and the matching MJCF actuator.
The JSON `spec.type` must match the MJCF actuator element.

```xml
<joint name="wheel_joint" type="hinge" axis="0 1 0"/>
<velocity name="wheel_velocity" joint="wheel_joint" kv="3"/>
```

Matching JSON:

```json
{
  "$schema": "../../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "wheel_joint",
    "type": "velocity"
  },
  "mjcf_binding": {
    "actuator_name": "wheel_velocity"
  }
}
```

`spec.joint_name` is the joint name. `mjcf_binding.actuator_name` is the
actuator name. Treating them as the same thing is a common source of confusion.

## What Goes In JSON

JSON defines the sensor or actuator parameters and the binding to MJCF runtime
objects.

Sensor profile example:

```json
{
  "$schema": "../schema/ultrasonic.schema.json",
  "frame_id": "front_ultrasonic",
  "DetectionDistance": {
    "Min": 0.05,
    "Max": 2.0
  },
  "UpdateRate": 20,
  "RuntimeBinding": {
    "source_site": "front_ultrasonic_site"
  }
}
```

Runtime output binding example:

```json
{
  "$schema": "../schema/joint-state-output.schema.json",
  "type": "joint_state",
  "name": "wheel_joint_states",
  "pdu_name": "joint_states",
  "update_rate_hz": 20,
  "joints": [
    {
      "name": "left_wheel_joint",
      "mjcf_joint": "left_wheel_joint"
    }
  ]
}
```

`frame_id` is the logical frame name used in ROS-compatible PDU messages.
`RuntimeBinding` and `mjcf_joint` refer to concrete MJCF object names.

## Standalone Checks

Run the lightweight validator:

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  --json config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

It checks:

- JSON parsing.
- Local `$schema` path existence.
- JSON Schema validation when the optional `jsonschema` package is installed.
- MJCF loading when the optional Python `mujoco` package is installed.

Install optional packages for stricter checks:

```bash
python3 -m pip install jsonschema mujoco
```

If an optional package is missing, the validator prints `WARN` and skips that
specific check.

## Name Binding Checks

Schema validation checks the JSON shape. It does not prove that a referenced
MJCF object exists. Runtime loading still needs to resolve the binding.

Check these names carefully:

- `source_site` matches an MJCF `<site name="...">`.
- camera name matches an MJCF `<camera name="...">`.
- `mjcf_joint` matches an MJCF `<joint name="...">`.
- `mjcf_binding.actuator_name` matches a `<position>`, `<velocity>`, or
  `<motor>` actuator name.
- actuator JSON `spec.type` matches the MJCF actuator type.

## Before C++ Integration

Before wiring a new robot loop, run the closest example with your model or
config where possible.

Ultrasonic:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example \
  models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

RGB camera:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example \
  models/sensors/color_camera/color-camera-sample.xml \
  config/sensors/color_camera/simple-color-camera.json \
  ./camera_color_sample.png
```

Joint actuator:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

## C++ Integration Entry Point

After file-level checks pass:

1. Load the MJCF with `WorldImpl`.
2. Create the sensor or actuator class.
3. Call `LoadConfig()`.
4. Call one sensor/action method such as `Measure()`, `Capture()`, `Build()`,
   or `SetTarget()`.
5. Add PDU adapters only after the MuJoCo object names and JSON config resolve.

PDU wiring can come last. First prove that the MJCF objects and JSON config are
valid together.

## Common Failures

- The XML is well-formed but not loadable as a MuJoCo model.
- JSON names do not match MJCF body/site/camera/joint/actuator names.
- `frame_id` is confused with an MJCF object name.
- Actuator JSON `spec.type` does not match the MJCF actuator type.
- Sensor direction is different from the assumed local axis. Ultrasonic uses
  site local `+X`; LiDAR depends on the integration's reference body yaw.
- JSON Schema validation passes, but MJCF binding fails. Schema validation
  does not inspect the MJCF model.
