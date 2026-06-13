# JSON Config Guide

This guide explains the JSON config structure used in this repository. Before
adding a sensor or actuator, understand which JSON file defines behavior, which
one binds to MJCF names, and which one exposes data over Hakoniwa PDU.

For detailed schemas, see
[`sensor-actuator-config-schema.md`](../spec/sensor-actuator-config-schema.md). For MJCF
name binding, see [`mjcf-json-authoring.md`](mjcf-json-authoring.md).

## Config Categories

| Category | Location | Role | Example |
| --- | --- | --- | --- |
| PDU robot definition | `config/*-compact.json` | Maps robot names to PDU type lists | `config/tb3-pdudef-compact.json` |
| PDU type list | `config/*pdutypes*.json` | Defines PDU channel names, types, and sizes | `config/tb3-pdutypes.json` |
| endpoint config | `config/endpoint/*.json` | Selects PDU definition, cache, and comm files for an endpoint | `config/endpoint/tb3_sim_endpoint.json` |
| sensor profile | `config/sensors/<type>/*.json` | Defines sensor behavior or sensor specification | `config/sensors/lidar/lds-02.json` |
| PDU output config | `config/sensors/<type>/*.json` | Defines how MuJoCo state is published as PDU output | `config/sensors/imu/tb3-imu.json` |
| actuator profile | `config/actuator/<type>/*.json` | Defines how a command target is applied to a MuJoCo actuator | `config/actuator/joint/tb3_left_wheel.json` |
| schema | `config/**/schema/*.schema.json` | Defines valid JSON shape | `config/sensors/schema/lidar-2d.schema.json` |

Most users start with sensor profiles, PDU output configs, and actuator
profiles. PDU definition and endpoint config matter when Python tools or other
Hakoniwa assets need to exchange data.

## Think In Three Config Containers

Sensor and actuator configs are easier to understand if you separate them into
three conceptual containers.

| Container | Role | Typical fields |
| --- | --- | --- |
| `spec` | Sensor or actuator specification | `DetectionDistance`, `AngleRange`, `DistanceAccuracy`, `type`, `limit` |
| `mjcf_binding` | Binding to MJCF object names | `source_body`, `source_site`, `actuator_name`, `mjcf_joint` |
| `pdu_config` | Hakoniwa PDU input/output config | `pdu_name`, `update_rate_hz`, PDU message type |

Conceptually:

```json
{
  "spec": {
    "frame_id": "laser",
    "DetectionDistance": {
      "Min": 160,
      "Max": 8000
    }
  },
  "mjcf_binding": {
    "source_body": "base_scan"
  },
  "pdu_config": {
    "pdu_name": "laser_scan",
    "update_rate_hz": 5
  }
}
```

Some older JSON files are not fully nested this way. For compatibility, some
`spec` fields may still appear as top-level fields, and `mjcf_binding` may be
represented by the `RuntimeBinding` key.

```json
{
  "frame_id": "laser",
  "DetectionDistance": {
    "Min": 160,
    "Max": 8000
  },
  "RuntimeBinding": {
    "source_body": "base_scan"
  }
}
```

This guide uses `spec` / `mjcf_binding` / `pdu_config` for explanation and also
mentions backward-compatible JSON key names where they still matter.

## Current Common Shape

Many config files look roughly like this:

```json
{
  "$schema": "../schema/example.schema.json",
  "type": "example",
  "name": "example_name",
  "frame_id": "example_frame",
  "pdu_name": "example_pdu",
  "update_rate_hz": 20,
  "RuntimeBinding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "source_body": "example_body",
    "source_site": "example_site"
  }
}
```

Not every config has every field. Use the closest sample and matching schema as
the source of truth.

## Common Fields

### `$schema`

Identifies the schema used by editors and validation tools.

```json
"$schema": "../schema/lidar-2d.schema.json"
```

Some existing files use remote-looking schema IDs:

```json
"$schema": "https://hakoniwa.dev/schemas/ultrasonic.schema.json"
```

`tools/validate_assets.py` maps `https://hakoniwa.dev/schemas/<name>.schema.json`
to the local schema files in this repository.

### `type`

Identifies the PDU output config or actuator profile type.

```json
"type": "joint_state"
```

```json
"type": "velocity"
```

Some sensor profiles do not have `type`; their sensor kind is implied by the
schema and sensor-specific fields.

### `name`

Human-readable application config name.

### `frame_id`

Logical frame name stored in ROS-compatible PDU messages. It is not necessarily
an MJCF body or site name.

### `pdu_name`

Hakoniwa PDU channel name. The same name must exist in the relevant
`config/*pdutypes*.json` file.

### `update_rate_hz`

Output publish rate for PDU output configs. Sensor profiles may use
`update_rate` or `UpdateRate`; follow the existing schema for that sensor type.

## `mjcf_binding` / MJCF Binding (`RuntimeBinding`)

`RuntimeBinding` connects a JSON config to concrete MJCF object names. For user
documentation, think of it as the `mjcf_binding` block.

```json
"RuntimeBinding": {
  "config_style": "hakoniwa-sdf-like",
  "runtime_source": "mjcf",
  "source_site": "front_ultrasonic_site"
}
```

These are MuJoCo XML names, not PDU names.

| Field | Meaning | Example |
| --- | --- | --- |
| `config_style` | Config convention, currently `hakoniwa-sdf-like` | `hakoniwa-sdf-like` |
| `runtime_source` | Binding source, usually `mjcf` | `mjcf` |
| `parent_body` | Parent body name | `base_link` |
| `source_body` | Body used as sensor/state source | `base_scan` |
| `source_site` | Site used as sensor origin and orientation | `front_ultrasonic_site` |
| `actuator_name` | MJCF actuator name | `left_wheel_velocity` |
| `frame_id_override` | Optional published frame override | `front_ultrasonic` |

Schema validation cannot prove that a `RuntimeBinding` target exists in MJCF.
That is checked when the model and config are loaded by runtime code.

## `spec` Vs `pdu_config`

`spec` describes sensor or actuator behavior:

```json
{
  "frame_id": "laser",
  "DetectionDistance": {
    "Min": 160,
    "Max": 8000
  },
  "AngleRange": {
    "Min": -180.0,
    "Max": 180.0,
    "Resolution": 1.0,
    "ScanFrequency": 5
  }
}
```

`pdu_config` describes how the component connects to Hakoniwa PDU. For sensors
it usually describes published MuJoCo state. For actuators it describes the
command PDU channel to receive:

```json
{
  "spec": {
    "type": "joint_state",
    "name": "tb3_wheel_joint_states",
    "joints": [
      { "name": "left_wheel_joint" }
    ]
  },
  "mjcf_binding": {
    "joints": [
      {
        "name": "left_wheel_joint",
        "mjcf_joint": "wheel_left_joint"
      }
    ]
  },
  "pdu_config": {
    "pdu_name": "joint_states",
    "update_rate_hz": 20,
    "message_type": "sensor_msgs/JointState"
  }
}
```

Some components, such as LiDAR and ultrasonic, keep `spec` and `mjcf_binding`
(`RuntimeBinding`) inside the sensor profile. Others, such as IMU, joint state,
odometry, and TF, are defined as PDU output configs.

## Actuator Profile

An actuator profile defines how a target is written to a MuJoCo actuator.

```json
{
  "$schema": "../schema/joint-actuator.schema.json",
  "spec": {
    "joint_name": "left_wheel_joint",
    "type": "velocity",
    "limit": {
      "velocity": 12.0
    }
  },
  "mjcf_binding": {
    "config_style": "hakoniwa-sdf-like",
    "runtime_source": "mjcf",
    "actuator_name": "left_wheel_velocity"
  }
}
```

`spec.joint_name` is the MJCF joint name. `mjcf_binding.actuator_name` is the
MJCF actuator name. `spec.type` must match the MJCF actuator kind.
Backward-compatible top-level `joint_name` / `type` and `RuntimeBinding` are
still accepted, but new configs should use `spec` and `mjcf_binding`.

| JSON `spec.type` | MJCF |
| --- | --- |
| `position` | `<position>` |
| `velocity` | `<velocity>` |
| `torque` | `<motor>` |

## PDU Definition And Endpoint Config

`config/tb3-pdudef-compact.json` maps a robot name to a PDU type list.

```json
{
  "paths": [
    {
      "id": "tb3-endpoint",
      "path": "tb3-pdutypes.json"
    }
  ],
  "robots": [
    {
      "name": "TB3",
      "pdutypes_id": "tb3-endpoint"
    }
  ]
}
```

`config/tb3-pdutypes.json` defines channels:

```json
{
  "name": "laser_scan",
  "type": "sensor_msgs/LaserScan",
  "pdu_size": 8192
}
```

Endpoint config selects the definition, cache, and comm files:

```json
{
  "name": "tb3_sim_endpoint",
  "pdu_def_path": "../tb3-pdudef-compact.json",
  "cache": "cache/buffer.json",
  "comm": "comm/shm_sim_comm.json"
}
```

## Validation

Run:

```bash
python3 tools/validate_assets.py \
  --json config/sensors/lidar/lds-02.json \
  --json config/actuator/joint/sample_velocity_actuator.json
```

With MJCF:

```bash
python3 tools/validate_assets.py \
  --mjcf models/sensors/ultrasonic/ultrasonic-sensor-test.xml \
  --json config/sensors/ultrasonic/lego-spike-distance-sensor.json
```

This checks JSON parsing, JSON Schema validation, and MJCF loading when the
optional Python packages are installed. It does not prove that a PDU exists in a
running endpoint or that a binding points to the intended physical object.

## Beginner Workflow

1. Copy the closest existing JSON.
2. Keep `$schema`.
3. Keep track of whether you are editing `spec`, `mjcf_binding`, or `pdu_config`.
4. Decide `name`, `frame_id`, and `pdu_name` separately.
5. Match `mjcf_binding` and `mjcf_joint` to MJCF names.
6. Run `tools/validate_assets.py`.
7. Run the closest example until `LoadConfig()` succeeds.
8. Change PDU definition and endpoint config only when communication is needed.
