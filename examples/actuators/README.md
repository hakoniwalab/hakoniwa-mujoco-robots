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
