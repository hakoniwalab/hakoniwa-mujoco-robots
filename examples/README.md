# Examples

This directory contains small, user-facing examples for trying individual Hakoniwa MuJoCo features without running the full robot demos.

## Sensor Examples

Sensor examples live under:

```text
examples/sensors/
```

Available examples:

- [Ultrasonic Sensor Example](sensors/ultrasonic/README.md)
  - minimal MuJoCo ultrasonic range sensor
  - interactive keyboard movement
  - measured ray visualization in the MuJoCo viewer
  - `sensor_msgs/Range` PDU conversion path
  - Hakoniwa PDU publisher and Python reader examples
- [Color Camera Sensor Example](sensors/color_camera/README.md)
  - minimal MuJoCo RGB camera sensor
  - red / green / blue panels in a small scene
  - MuJoCo viewer with `s` key PNG capture from the viewer or terminal
  - `i/k/j/l` camera movement
  - left / center / right RGB sample printout
  - PNG output for quick visual inspection

These examples are intended to complement the larger TurtleBot3 and forklift demos described in the top-level README.

## Actuator Examples

Actuator examples live under:

```text
examples/actuators/
```

Available examples:

- [Joint Actuator Example](actuators/joint/README.md)
  - minimal MuJoCo `<position>` actuator
  - minimal MuJoCo `<velocity>` actuator
  - MuJoCo viewer with keyboard target control
  - JSON config loaded through `JointActuatorImpl`
  - `SetTarget()` writes targets to MuJoCo `ctrl[]`
