# Ultrasonic Sensor Example

This example demonstrates the Hakoniwa MuJoCo ultrasonic sensor with a minimal MJCF model.

The purpose of this example is to manually verify that the ultrasonic sensor can:

- load a MuJoCo model
- resolve a sensor site
- move the robot body with keyboard commands
- measure distance from the sensor site
- show how the measured range changes as the robot moves

This is an interactive example, not a strict automated test.

## Files

```text
examples/sensors/ultrasonic/
  README.md
  ultrasonic-example.cpp

models/sensors/ultrasonic/
  ultrasonic-sensor-test.xml

config/sensors/ultrasonic/
  lego-spike-distance-sensor.json
````

## Model

The example uses the following MJCF model:

```text
models/sensors/ultrasonic/ultrasonic-sensor-test.xml
```

The model contains:

* a movable base body named `base_footprint`
* a sensor site named `front_ultrasonic_site`
* a front wall
* a diagonal obstacle for cone-ray behavior checks

The ultrasonic sensor uses the local `+X` axis of the source site as the measurement direction.

## Sensor Config

The example uses the following ultrasonic sensor profile:

```text
config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

This profile represents a simple LEGO SPIKE-like distance sensor.

## Expected Initial Distance

The front wall is located at `x = 1.0`.

The wall half-size along the X axis is `0.02`, so the front surface of the wall is at:

```text
x = 1.0 - 0.02 = 0.98
```

The sensor site is located at:

```text
x = 0.12
```

Therefore, the expected initial surface distance is approximately:

```text
0.98 - 0.12 = 0.86 m
```

Small differences are acceptable depending on raycast behavior, geometry resolution, and future sensor runtime policies.

## Controls

The example accepts simple keyboard commands from standard input.

```text
i : move forward  (+X)
k : move backward (-X)
j : move left     (+Y)
l : move right    (-Y)
s : sense and print ultrasonic range
q : quit
```

The initial implementation uses translation only.
Yaw rotation is intentionally omitted to keep the first example focused on position and distance behavior.

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
  s : sense
  q : quit

> s
range: 0.860 m

> i
moved: x += 0.050, base_pos=(0.050, 0.000, 0.100)

> s
range: 0.810 m

> k
moved: x -= 0.050, base_pos=(0.000, 0.000, 0.100)

> j
moved: y += 0.050, base_pos=(0.000, 0.050, 0.100)

> s
range: 0.862 m
```

## Build

From the repository root:

```bash
./build.bash
```

Or build directly with CMake:

```bash
cmake --build src/cmake-build
```

## Run

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

## Current Scope

This example is intentionally minimal.

Included:

* MuJoCo model loading
* ultrasonic sensor config loading
* source site lookup
* keyboard-based base movement
* manual sensing with `s`
* range output to standard output

Not included in the first version:

* Hakoniwa PDU publish
* viewer integration
* automatic assertions
* CI execution
* yaw rotation
* multi-sensor setup

Those features can be added later after the basic sensor behavior is stable.

## Relationship to Tests

This example is a manual usage example.

After the behavior is stabilized, the same logic can be promoted into automated smoke tests under:

```text
tests/sensors/ultrasonic/
```

A future smoke test should verify at least:

* model load succeeds
* `front_ultrasonic_site` exists
* initial range is approximately `0.86 m`
* moving forward decreases the measured range
* moving backward increases the measured range
* self geometry is not detected as the nearest hit

