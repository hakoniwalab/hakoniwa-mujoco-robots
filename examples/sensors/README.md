# Sensor Examples

This directory contains small examples for Hakoniwa MuJoCo sensor components.

## Ultrasonic

See:

```text
examples/sensors/ultrasonic/README.md
```

The ultrasonic example demonstrates:

- loading a minimal MuJoCo model
- binding a range sensor to a MuJoCo `site`
- moving the robot body interactively with `i/k/j/l`
- measuring range with `s`
- visualizing the measured ray in the MuJoCo viewer
- converting the internal ultrasonic frame to `sensor_msgs/Range`

Run from the repository root:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

## Color Camera

See:

```text
examples/sensors/color_camera/README.md
```

The color camera example demonstrates:

- loading a minimal MuJoCo scene with red / green / blue panels
- opening a MuJoCo viewer and capturing an RGB camera frame with `s` from the viewer or terminal
- moving the camera body with `i/k/j/l`
- printing sample RGB values from the captured image
- writing the result to a PNG file

Run from the repository root:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```
