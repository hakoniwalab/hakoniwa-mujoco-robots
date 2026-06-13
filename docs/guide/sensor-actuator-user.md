# Sensor And Actuator User Guide

This guide is the practical entry point for using Hakoniwa MuJoCo sensors and
actuators. It focuses on what to run, which files to edit, and how data moves
between MuJoCo and Hakoniwa PDU.

If you are writing MJCF XML and JSON from scratch, start with
[`mjcf-json-authoring.md`](mjcf-json-authoring.md).
For the shared JSON structure and common fields, read
[`json-config.md`](json-config.md).

For lower-level design boundaries, see
[`sensor-actuator-design.md`](../spec/sensor-actuator-design.md). For JSON field
details, see [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md).

## Quick Map

```text
MuJoCo model          models/
Sensor profiles      config/sensors/
Actuator profiles    config/actuator/
C++ components       src/sensors/, src/actuator/
PDU converters       include/hakoniwa/pdu/converter/
PDU endpoint I/O     include/hakoniwa/pdu/adapter/
Small examples       examples/sensors/, examples/actuators/
```

Use the examples first when learning a component. Use the TB3 and forklift
samples when you need a full robot loop.

## Data Flow

Sensor output normally flows in this direction:

```text
MuJoCo world -> sensor component -> frame struct -> PDU converter -> PDU adapter -> Hakoniwa endpoint
```

Actuator input normally flows in this direction:

```text
Hakoniwa endpoint -> PDU adapter -> command/target struct -> actuator component -> MuJoCo ctrl[]
```

The sensor and actuator components should stay usable without Hakoniwa endpoint
I/O. Endpoint access belongs in the adapter layer.

## Run The Small Examples

Build everything:

```bash
./build.bash
```

Run the ultrasonic range sensor example:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

Run the RGB camera PNG capture example:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

Run the joint actuator example:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

These examples are viewer-oriented and are meant for interactive inspection.
Do not treat them as headless checks unless the example README says so.

## Sensor Components

| Component | Main config | Main output | Example / sample |
| --- | --- | --- | --- |
| 2D LiDAR | `config/sensors/lidar/*.json` | `sensor_msgs/LaserScan` | TB3 demo |
| IMU | `config/sensors/imu/tb3-imu.json` | `sensor_msgs/Imu` | TB3 demo |
| Joint state | `config/sensors/joint_state/tb3-wheel-joint-states.json` | `sensor_msgs/JointState` | TB3 demo |
| Odometry | `config/sensors/odometry/tb3-ground-truth-odom.json` | `nav_msgs/Odometry` | TB3 demo |
| TF | `config/sensors/tf/tb3-basic-tf.json` | `tf2_msgs/TFMessage` | TB3 demo |
| Ultrasonic | `config/sensors/ultrasonic/lego-spike-distance-sensor.json` | `sensor_msgs/Range` | `examples/sensors/ultrasonic/` |
| RGB camera | `config/sensors/color_camera/simple-color-camera.json` | `sensor_msgs/Image` | `examples/sensors/color_camera/` |
| Depth / RGBD camera | `config/sensors/camera/*.json` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | sensor tests |

For sensor profile fields, start with the sample config closest to your sensor
and then check the matching schema under `config/sensors/schema/`.

## Actuator Components

The current reusable actuator path is the MuJoCo joint actuator:

```text
src/actuator/joint_actuator_impl.hpp
config/actuator/joint/*.json
examples/actuators/joint/
```

`JointActuatorImpl` resolves a MuJoCo actuator from JSON and writes targets to
`mjData::ctrl[]`. The actual control behavior comes from the MJCF actuator type:

| JSON `spec.type` | Expected MJCF actuator | Meaning of `ctrl[]` |
| --- | --- | --- |
| `position` | `<position>` | target joint position |
| `velocity` | `<velocity>` | target joint velocity |
| `torque` | `<motor>` | applied effort / torque command |

The JSON `spec.type` and MJCF actuator type must match. A config for `velocity` should
not point at a MuJoCo `<position>` actuator.

## Add A Sensor To A Robot Sample

1. Add or reuse the MuJoCo body, site, camera, joint, or frame that the sensor
   reads from.
2. Add a JSON profile under `config/sensors/<sensor_type>/`.
3. Load the profile in the robot/application class.
4. Build the sensor frame from MuJoCo state.
5. Convert and publish the frame through the matching PDU adapter.
6. Add the output PDU to the robot PDU definition when the data must be visible
   to Python or another Hakoniwa asset.

Use TB3 as the reference integration:

```text
src/main_for_sample/tb3/main.cpp
src/robots/tb3/tb3_robot.cpp
config/tb3-pdudef-compact.json
config/tb3-pdutypes.json
```

## Add A Joint Actuator

1. Define the MuJoCo joint and matching MJCF actuator in the model.
2. Add a JSON binding under `config/actuator/joint/`.
3. Create the actuator with `world->createJointActuator()`.
4. Call `LoadConfig()` once after loading the model.
5. Call `SetTarget()` each control step.
6. If the target comes from PDU, receive it through the matching adapter instead
   of reading endpoint data inside the actuator.

Minimal C++ usage:

```cpp
auto actuator = world->createJointActuator();
actuator->LoadConfig("config/actuator/joint/sample_velocity_actuator.json");
actuator->SetTarget(target_velocity);
```

## Common Checks

- Run `./doctor.bash` before diagnosing build or runtime problems.
- If Python tools fail to decode data, confirm the Python process and C++
  process are using the same PDU definition JSON.
- If a sensor publishes no data, check the update rate and whether the source
  body/site/camera name exists in the MJCF model.
- If a joint actuator does nothing, check that `mjcf_binding.actuator_name`
  matches a MuJoCo actuator name, not just a joint name.
  The old `RuntimeBinding.actuator_name` key is still accepted for compatibility.
- If a joint actuator reports a type mismatch, make the JSON `spec.type` match the
  MJCF actuator element: `<position>`, `<velocity>`, or `<motor>`.
- Viewer examples need a GUI/OpenGL context. For headless checks, use CMake
  configure, focused unit tests, or non-viewer targets.

## Where To Read Next

- [`mjcf-json-authoring.md`](mjcf-json-authoring.md)
- [`json-config.md`](json-config.md)
- [`examples/sensors/README.md`](../../examples/sensors/README.md)
- [`examples/actuators/README.md`](../../examples/actuators/README.md)
- [`examples/sensors/ultrasonic/README.md`](../../examples/sensors/ultrasonic/README.md)
- [`examples/sensors/color_camera/README.md`](../../examples/sensors/color_camera/README.md)
- [`examples/actuators/joint/README.md`](../../examples/actuators/joint/README.md)
- [`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md)
- [`sensor-actuator-design.md`](../spec/sensor-actuator-design.md)
