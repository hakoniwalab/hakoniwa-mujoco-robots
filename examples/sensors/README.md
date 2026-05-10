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
