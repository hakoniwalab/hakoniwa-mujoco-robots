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
