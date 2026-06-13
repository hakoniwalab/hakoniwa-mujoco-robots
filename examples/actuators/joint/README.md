# Joint Actuator Example

This example opens a MuJoCo viewer and shows the MJCF-native way to use joint actuators:

- `<position>` actuator: `ctrl[]` is the target joint position
- `<velocity>` actuator: `ctrl[]` is the target joint velocity
- keyboard input changes the position and velocity targets while the viewer is running

`JointActuatorImpl` does not implement a software position or velocity controller in this example. MuJoCo does that because the MJCF actuator type is `<position>` or `<velocity>`.

## Files

```text
examples/actuators/joint/
  README.md
  joint-actuator-example.cpp
  joint-actuator-hakoniwa-asset.cpp
  joint_actuator_sender.py

models/actuators/joint/
  position-velocity-actuator-sample.xml

config/actuator/joint/
  sample_position_actuator.json
  sample_velocity_actuator.json

config/
  joint-actuator-pdudef-compact.json
  joint-actuator-pdutypes.json
  endpoint/joint_actuator_endpoint.json
```

Read these first:

- [`joint-actuator-example.cpp`](./joint-actuator-example.cpp): the Joint Actuator API usage
- [`joint-actuator-hakoniwa-asset.cpp`](./joint-actuator-hakoniwa-asset.cpp): Hakoniwa PDU command receiver with a MuJoCo viewer
- [`joint_actuator_sender.py`](./joint_actuator_sender.py): Python Hakoniwa asset that sends `std_msgs/Float64` commands
- [`position-velocity-actuator-sample.xml`](../../../models/actuators/joint/position-velocity-actuator-sample.xml): the MJCF `<position>` and `<velocity>` actuators
- [`sample_position_actuator.json`](../../../config/actuator/joint/sample_position_actuator.json): JSON binding for the position actuator
- [`sample_velocity_actuator.json`](../../../config/actuator/joint/sample_velocity_actuator.json): JSON binding for the velocity actuator

## Joint Actuator API

The example uses the actuator through `IJointActuator`.

```cpp
auto world = std::make_shared<WorldImpl>();
world->loadModel(model_path);

auto position_actuator = world->createJointActuator();
position_actuator->LoadConfig(position_config_path);
position_actuator->SetTarget(position_target);

auto velocity_actuator = world->createJointActuator();
velocity_actuator->LoadConfig(velocity_config_path);
velocity_actuator->SetTarget(velocity_target);
```

The config resolves the MuJoCo actuator by name:

```json
{
  "spec": {
    "joint_name": "position_hinge",
    "type": "position",
    "limit": {
      "lower": -0.8,
      "upper": 0.8
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "actuator_name": "position_servo"
  }
}
```

`spec.joint_name` is the MJCF joint name. `mjcf_binding.actuator_name` is the
MJCF actuator name. The older top-level `joint_name` / `type` and
`RuntimeBinding` format is still accepted for compatibility, but new configs
should use `spec` and `mjcf_binding`.

The model defines the actual actuator behavior:

```xml
<position name="position_servo" joint="position_hinge" kp="6" dampratio="1.0"/>
<velocity name="velocity_servo" joint="velocity_hinge" kv="3"/>
```

## Build

From the repository root:

```bash
./build.bash
```

Or, if CMake has already been configured:

```bash
cmake --build src/cmake-build --target joint-actuator-example
```

The Hakoniwa PDU example target is:

```bash
cmake --build src/cmake-build --target joint-actuator-hakoniwa-asset
```

## Run

From the repository root:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

The MuJoCo viewer opens and the terminal prints the requested position target, measured position angle, requested velocity target, and measured joint velocity.

## Controls

```text
a / d  : decrease / increase position target
j / l  : decrease / increase velocity target
Space  : stop velocity target
r      : reset simulation state and targets
p      : pause / resume physics
h      : show help
q / Esc: quit
```

Use the mouse to rotate and zoom the viewer.

## Hakoniwa PDU Example

The Hakoniwa version receives position and velocity commands from PDU channels:

```text
JointActuatorAsset/position_target
JointActuatorAsset/velocity_target
```

Run it with three terminals:

```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-hakoniwa-asset
```

```bash
python3 examples/actuators/joint/joint_actuator_sender.py
```

```bash
hako-cmd start
```

The C++ process owns the MuJoCo viewer. The Python sender writes
`std_msgs/Float64` command PDUs. See
[`docs/tutorial/joint-actuator-hakoniwa-ja.md`](../../../docs/tutorial/joint-actuator-hakoniwa-ja.md)
for the full walkthrough.
