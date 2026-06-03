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

models/actuators/joint/
  position-velocity-actuator-sample.xml

config/actuator/joint/
  sample_position_actuator.json
  sample_velocity_actuator.json
```

Read these first:

- [`joint-actuator-example.cpp`](./joint-actuator-example.cpp): the Joint Actuator API usage
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
  "joint_name": "position_hinge",
  "type": "position",
  "RuntimeBinding": {
    "actuator_name": "position_servo"
  }
}
```

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
