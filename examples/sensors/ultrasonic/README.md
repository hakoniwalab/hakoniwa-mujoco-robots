# Ultrasonic Sensor Example

This example demonstrates a Hakoniwa ultrasonic range sensor running on MuJoCo.

It is intended as a small, user-facing example for understanding how the Ultrasonic Sensor API is used with a concrete MuJoCo model and JSON config.

The example shows how to:

- load a minimal MuJoCo model
- bind an ultrasonic sensor to a MuJoCo `site`
- create `UltrasonicSensor`
- apply the JSON config with `UltrasonicSensor::LoadConfig()`
- measure range with `UltrasonicSensor::Measure()`
- move the robot body interactively
- visualize the measured ray in the MuJoCo viewer
- convert the internal ultrasonic frame to `sensor_msgs/Range`

This is an interactive example. Automated checks for the same basic behavior are provided by the sensor unit tests.

## Files

```text
examples/sensors/ultrasonic/
  README.md
  ultrasonic-example.cpp
  ultrasonic-hakoniwa-asset.cpp
  read_range.py
  support/ultrasonic_example_support.hpp
  support/ultrasonic_example_support.cpp

models/sensors/ultrasonic/
  ultrasonic-sensor-test.xml

config/sensors/ultrasonic/
  lego-spike-distance-sensor.json

config/
  ultrasonic-pdudef-compact.json
  ultrasonic-pdutypes.json
  endpoint/ultrasonic_endpoint.json
  endpoint/comm/shm_ultrasonic_comm.json

tests/sensors/ultrasonic/unit/
  ultrasonic_config_loader_test.cpp
  ultrasonic_range_pdu_converter_test.cpp
  ultrasonic_measurement_test.cpp
```

Read these first:

- [`ultrasonic-example.cpp`](./ultrasonic-example.cpp): the Ultrasonic Sensor API usage
- [`ultrasonic-hakoniwa-asset.cpp`](./ultrasonic-hakoniwa-asset.cpp): publishes measured ranges as Hakoniwa PDU
- [`read_range.py`](./read_range.py): Python Hakoniwa asset that receives and prints the range PDU
- [`lego-spike-distance-sensor.json`](../../../config/sensors/ultrasonic/lego-spike-distance-sensor.json): the sensor config
- [`ultrasonic-sensor-test.xml`](../../../models/sensors/ultrasonic/ultrasonic-sensor-test.xml): the MuJoCo model, sensor site, and obstacles

## Ultrasonic Sensor API

The example uses the range sensor through `UltrasonicSensor`.

```cpp
auto world = std::make_shared<hako::robots::physics::impl::WorldImpl>();
world->loadModel(model_path);

UltrasonicSensor sensor(
    world,
    "front_ultrasonic_site",
    "base_footprint");

sensor.LoadConfig(config_path);

UltrasonicFrame frame {};
sensor.Measure(frame);
```

The full version is in [`ultrasonic-example.cpp`](./ultrasonic-example.cpp). The surrounding code only prepares the MuJoCo model, viewer, keyboard controls, robot movement, and debug ray drawing.

## Model

The example uses this MJCF model:

```text
models/sensors/ultrasonic/ultrasonic-sensor-test.xml
```

The model contains:

- a movable robot body named `base_footprint`
- a free joint named `base_freejoint`
- a sensor site named `front_ultrasonic_site`
- a front wall
- a diagonal obstacle

The ultrasonic sensor uses the local `+X` axis of the source site as its measurement direction.

In other words, the ray starts at `front_ultrasonic_site` and points along the site's local forward direction.

## Sensor Config

The example uses this sensor profile:

```text
config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

The profile currently represents a simple LEGO SPIKE-like distance sensor profile for Hakoniwa testing.

Important fields:

```json
{
  "spec": {
    "frame_id": "spike_distance_sensor_link",
    "DetectionDistance": {
      "Min": 0.05,
      "Max": 2.0
    },
    "DistanceAccuracy": [
      {
        "StdDev": 0.0,
        "Precision": 0.0,
        "NoiseDistribution": "none"
      }
    ],
    "Cone": {
      "Horizontal": 0.0,
      "Vertical": 0.0,
      "RayCount": 1
    },
    "RadiationType": "ultrasound",
    "UpdateRate": 100.0
  },
  "mjcf_binding": {
    "source_site": "front_ultrasonic_site"
  },
  "pdu_config": {
    "pdu_name": "range",
    "update_rate_hz": 100.0,
    "message_type": "sensor_msgs/Range"
  }
}
```

`spec` is the physical sensor specification. `mjcf_binding` names the MJCF site
used as the sensor origin and direction. `pdu_config` records the intended
Hakoniwa PDU output settings. For this example, noise is disabled and
`RayCount` is `1`, so the measurement is deterministic and easy to verify.

## Expected Distances

The front wall is located at `x = 1.0`.

The wall half-size along the X axis is `0.02`, so the front surface of the wall is at:

```text
x = 1.0 - 0.02 = 0.98
```

The sensor site is initially located at:

```text
x = 0.12
```

Therefore, the expected initial surface distance is:

```text
0.98 - 0.12 = 0.86 m
```

When the robot moves forward by `0.05 m`, the expected range becomes:

```text
0.86 - 0.05 = 0.81 m
```

The unit test also verifies a diagonal obstacle hit and a no-hit case.

## Build

From the repository root:

```bash
./build.bash
```

Or, if CMake has already been configured:

```bash
cmake --build src/cmake-build
```

The example target is:

```text
ultrasonic-example
```

To build the Hakoniwa PDU publisher example only:

```bash
cmake --build src/cmake-build --target ultrasonic-hakoniwa-asset
```

## Run

From the repository root:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

You can also pass a model path and config path explicitly:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example \
  models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

## Controls

The example accepts simple keyboard commands from standard input.

```text
i : move forward  (+X)
k : move backward (-X)
j : move left     (+Y)
l : move right    (-Y)
s : sense and print ultrasonic range
h : help
q : quit
```

Moving with `i`, `k`, `j`, or `l` updates the robot position and refreshes the ultrasonic measurement.

Pressing `s` explicitly measures the current range and prints the latest result.

## Hakoniwa PDU Publisher

The Hakoniwa publisher example opens a MuJoCo viewer and publishes
`sensor_msgs/Range` as a Hakoniwa PDU.

Terminal 1: C++ publisher

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-hakoniwa-asset
```

Terminal 2: Python reader

```bash
python3 examples/sensors/ultrasonic/read_range.py
```

If this prints `protocol 'shm' requested but Hakoniwa core support is disabled`,
the Python `hakoniwa-pdu-endpoint` runtime being loaded was built without
Hakoniwa core support. Rebuild `hakoniwa-pdu-endpoint` with
`HAKO_PDU_ENDPOINT_ENABLE_HAKONIWA_CORE=ON`, or set
`HAKO_PDU_ENDPOINT_SHARED_LIB` to a core-enabled runtime library before running
the reader.

Terminal 3: start trigger

```bash
hako-cmd start
```

The publisher uses:

```cpp
hako::robots::pdu::adapter::sensor_msgs::RangePduAdapter
```

and sends values with:

```cpp
range_adapter->send(sensor.GetConfig(), frame);
```

Measurement and publishing are gated by `UltrasonicSensor::ShouldUpdate()`,
so the runtime cadence follows `spec.UpdateRate`.

The default PDU key is a PDU robot name plus a channel name:

```text
PduKey("UltrasonicAsset", "range")
```

`UltrasonicAsset` is the PDU robot name from
`ultrasonic-pdudef-compact.json`, and `range` is the channel name from
`ultrasonic-pdutypes.json`.

The Python reader prints lines like:

```text
range=0.860 m min=0.050 max=2.000 fov=0.000 radiation_type=0
```

## Viewer

The MuJoCo viewer displays the model and the latest measured ray.

After the first measurement:

- green ray means hit
- red ray means no-hit

The ray is drawn from the sensor site to the measured range endpoint.

Mathematically, the endpoint is:

```text
to = site_position + world_forward * range
```

where `world_forward` is the source site's local `+X` direction transformed into world coordinates.

This debug visualization is useful for ultrasonic sensors and can also be reused for 2D LiDAR ray visualization.

## Example Session

```text
Hakoniwa Ultrasonic Sensor Example
model : models/sensors/ultrasonic/ultrasonic-sensor-test.xml
config: config/sensors/ultrasonic/lego-spike-distance-sensor.json
site  : front_ultrasonic_site

Controls:
  i : move forward  (+X)
  k : move backward (-X)
  j : move left     (+Y)
  l : move right    (-Y)
  s : sense and print ultrasonic range
  h : help
  q : quit

base_pos=(0.000, 0.000, 0.100)
> s
range=0.860 m, status=OK, variance=0.000e+00
> i
range=0.810 m, status=OK, variance=0.000e+00
moved: x += 0.050, base_pos=(0.050, 0.000, 0.100)
> j
range=0.810 m, status=OK, variance=0.000e+00
moved: y += 0.050, base_pos=(0.050, 0.050, 0.100)
```

## PDU Mapping

The ultrasonic sensor keeps an internal frame with simulation-friendly diagnostic data:

```text
UltrasonicFrame
  frame_id
  range
  variance
  status
```

For external communication, it can be converted to the ROS-compatible Hakoniwa PDU type:

```text
sensor_msgs/Range
  std_msgs/Header header
  uint8 radiation_type
  float32 field_of_view
  float32 min_range
  float32 max_range
  float32 range
```

Mapping:

```text
UltrasonicConfig.frame_id              -> Range.header.frame_id
UltrasonicConfig.radiation_type        -> Range.radiation_type
UltrasonicConfig.cone.horizontal       -> Range.field_of_view
UltrasonicConfig.detection_distance.min -> Range.min_range
UltrasonicConfig.detection_distance.max -> Range.max_range
UltrasonicFrame.range                  -> Range.range
```

`variance` and `status` are internal diagnostic fields and are not part of ROS `sensor_msgs/Range`.

## Tests

Sensor unit tests can be built with:

```bash
cmake -S src -B src/cmake-build \
  -DHAKO_USE_THIRDPARTY_HAKONIWA=ON \
  -DHAKO_BUILD_SENSOR_TESTS=ON \
  -DHAKO_BUILD_CAMERA_SMOKE_TESTS=OFF \
  -DUSE_VIEWER=OFF

cmake --build src/cmake-build --target sensor_unit_tests
```

Run all sensor unit tests:

```bash
cmake --build src/cmake-build --target run_sensor_unit_tests
```

Ultrasonic-specific tests are grouped under:

```text
ultrasonic_unit_tests
```

They verify:

- config loading
- `sensor_msgs/Range` PDU conversion
- deterministic measurement values with noise disabled

The deterministic measurement test checks:

```text
initial front wall range        = 0.86 m
front wall range after x move   = 0.81 m
diagonal obstacle range         = 0.18 m
no-hit range                    = 2.00 m with NO_HIT status
```

## Current Scope

Included:

- MuJoCo model loading
- ultrasonic sensor config loading
- source site lookup
- keyboard-based base movement
- deterministic range measurement
- MuJoCo viewer ray overlay
- `sensor_msgs/Range` PDU conversion
- Hakoniwa PDU publisher example
- Python Hakoniwa PDU reader example
- automated unit tests for config, PDU conversion, and measurement values

Not included yet:

- live Hakoniwa endpoint publish from the example
- yaw rotation controls
- multi-sensor setup
- full 2D LiDAR ray overlay

These can be added incrementally now that the sensor, viewer, PDU, example, and test paths are connected.
