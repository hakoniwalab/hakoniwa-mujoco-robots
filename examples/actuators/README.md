# Actuator Examples

Actuator examples live under:

```text
examples/actuators/
```

Available examples:

- [Joint Actuator Example](joint/README.md)
  - minimal MuJoCo `<position>` actuator
  - minimal MuJoCo `<velocity>` actuator
  - MuJoCo viewer with keyboard target control
  - JSON config loaded through `JointActuatorImpl`
  - `SetTarget()` writes targets to MuJoCo `ctrl[]`
  - Hakoniwa PDU command receiver and Python command sender examples
- [AgileX Tracer Rover Samples](agilex_tracer/README.md)
  - generated rover MJCF from `hakoniwa-mbody-registry`
  - left/right wheel velocity actuators loaded from JSON
  - shared differential-drive kinematics helper
  - MuJoCo-only `--check` smoke
  - Hakoniwa `geometry_msgs/Twist` command receiver and Python sender
- [Unitree Go1 Joint I/O Example](unitree_go1/README.md)
  - vendored MuJoCo Menagerie MJCF and mesh assets
  - 12 position actuators bound by stable MJCF names
  - Menagerie `home` keyframe ctrl values
  - MuJoCo-only `--check` smoke and viewer perturbation controls
  - Hakoniwa `std_msgs/Float64MultiArray` joint command receiver
  - Python 12-joint target sender and `sensor_msgs/JointState` readback
