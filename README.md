# hakoniwa-mujoco-robots

English | [日本語](README-ja.md)

## TL;DR
- This repository provides MuJoCo-based robot simulation assets for Hakoniwa.
- It supports ROS/URDF-derived robot models, including TurtleBot3 Burger.
- It connects C++ MuJoCo simulators and Python controllers / visualizers through Hakoniwa PDU.
- It includes a TurtleBot3 sample with gamepad control and 2D LiDAR `LaserScan`-compatible PDU output.
- It includes configurable LiDAR noise profiles such as `LDS-01`-like and `URG-04LX-UG01`-equivalent settings.
- It includes forklift samples with context save/restore and RD-light handoff as advanced examples.
- Compact JSON is the default for both C++ and Python (`hakoniwa-pdu >= 1.3.7`).

## Demo Videos
- TurtleBot3 + 2D LiDAR / sensor noise demo:
  - [![Watch the demo](https://img.youtube.com/vi/B5h-KKH4tpg/hqdefault.jpg)](https://www.youtube.com/watch?v=B5h-KKH4tpg)
- Runtime handoff demo (RD-light, dual forklift assets):
  - [![Watch the demo](https://img.youtube.com/vi/xaJJ1wEgNR8/hqdefault.jpg)](https://www.youtube.com/watch?v=xaJJ1wEgNR8)

### TurtleBot3 + 2D LiDAR Demo Notes

This demo shows a TurtleBot3 Burger running on MuJoCo with a 2D LiDAR simulation connected through Hakoniwa PDU.

The point is not only to visualize LiDAR point clouds, but also to reproduce differences in sensor noise characteristics.

- With TurtleBot3's standard `LDS-01`, the point cloud is visibly noisier
- With the `URG-04LX-UG01`-like profile, obstacle contours appear much clearer

One important Sim2Real theme in Hakoniwa is to represent the practical difference users experience when changing sensors, not just robot motion alone.

Configuration:
- ROS-derived TurtleBot3 model executed on MuJoCo
- 2D LiDAR simulated by raycast using the selected sensor profile
- Scan frequency, angular range, angular resolution, and noise model are loaded from JSON
- Output published as `LaserScan` PDU on Hakoniwa
- Point cloud visualized by a Python visualizer
- Noise differences reproduced for `LDS-01` and `URG-04LX-UG01`-equivalent settings

### Forklift RD-light Handoff Demo

This is an advanced experimental demo.  
It runs two MuJoCo forklift assets and performs single-node ownership handoff with context save/restore.

- RD-light is an **advanced / experimental** handoff demo
- It is not RD-full
- See the later RD-light section and [rd-design.md](rd-design.md) for details

---

## What This Repository Provides

Included:
- MuJoCo robot models
- Hakoniwa-integrated C++ simulators
- Python controllers
- Python visualizers
- PDU configs
- sensor configs
- forklift context save/restore
- RD-light as an advanced demo

Directory map:
- `models/`: MuJoCo XML models
- `config/`: PDU JSON configs
- `config/sensors/`: LiDAR / sensor spec JSON
- `src/`: C++ simulator implementation
- `python/`: Python controllers / visualizers
- `docker/`: Docker scripts
- `logs/`: generated logs
- `tmp/`: generated state files

---

## Architecture

Hakoniwa PDU is the hub, and MuJoCo (C++) communicates with Python controllers / visualizers over that contract.

- **Hakoniwa**: synchronization and PDU runtime
- **MuJoCo C++ Asset**: physics stepping + PDU read/write
- **Python Controller / Visualizer**: input and inspection tools
- **PDU JSON**: contract of channels/types/sizes

```text
+-----------------------------+      PDU (shared contract)      +----------------------+
| Python Controller / Viewer  |  <----------------------------> | MuJoCo C++ Simulator |
| (gamepad / visualizer)      |                                  | (tb3_sim / forklift) |
+--------------+--------------+                                  +----------+-----------+
               |                                                            |
               |                        Hakoniwa runtime                     |
               +------------------------(sync / mmap / PDU)-----------------+
```

---

## Quick Start: TurtleBot3 + 2D LiDAR

This is the shortest path to verify **MuJoCo + Hakoniwa + TurtleBot3 + gamepad + LiDAR**.

Prepare 4 terminals.

1. simulator
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim
```

2. gamepad controller
```bash
python python/tb3_gamepad.py
```

3. LiDAR visualizer
```bash
python python/lidar_visualizer.py
```

4. start trigger
```bash
hako-cmd start
```

To switch LiDAR spec:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

## Quick Start: Forklift

This is the shortest path for the forklift unit sample.

Prepare 3 terminals.

1. simulator
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

2. Python controller
```bash
python -m python.forklift_simple_auto config/forklift-unit-compact.json \
  --forward-distance 2.0 --backward-distance 2.0 --move-speed 0.7
```

3. start trigger
```bash
hako-cmd start
```

For compatibility, `controll.bash` is temporarily kept and internally calls `control.bash`.

---

## Prerequisites

### 1) Install hakoniwa-core-pro (required)

```bash
git clone --recursive https://github.com/hakoniwalab/hakoniwa-core-pro.git
cd hakoniwa-core-pro
bash build.bash
bash install.bash
```

Set paths if needed:

Linux:
```bash
export PATH=/usr/local/hakoniwa/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$LD_LIBRARY_PATH
```

macOS:
```bash
export PATH=/usr/local/hakoniwa/bin:$PATH
export DYLD_LIBRARY_PATH=/usr/local/hakoniwa/lib:$DYLD_LIBRARY_PATH
```

### 2) OS notes

- macOS: `brew install glfw`
- Ubuntu:
```bash
sudo apt-get update
sudo apt-get install -y libgl1 libgl1-mesa-dri libglx-mesa0 mesa-utils libglfw3-dev
```

---

## Setup

```bash
git clone https://github.com/hakoniwalab/hakoniwa-mujoco-robots.git
cd hakoniwa-mujoco-robots
git submodule update --init --recursive
./build.bash
```

- MuJoCo version is managed by `MUJOCO_VERSION.txt`.
- Clean build:
```bash
./build.bash clean
```

### Windows (MSVC + PowerShell)

If `hakoniwa-core-pro` and `hakoniwa-pdu-endpoint` are already installed on Windows, pass their install roots to `build-win.ps1`.

```powershell
.\build-win.ps1 -Clean `
  -BuildDirName build-win `
  -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install `
  -HakoniwaPduEndpointRoot C:\project\hakoniwa-pdu-endpoint\install `
  -ExtraPrefixPaths C:\project\vcpkg\installed\x64-windows `
  -ToolchainFile C:\project\vcpkg\scripts\buildsystems\vcpkg.cmake
```

Notes:
- The script configures with `-S src`, matching the existing Unix build layout.
- `HakoniwaCoreRoot` is forwarded to `HAKONIWA_INSTALL_PREFIX`.
- `HakoniwaPduEndpointRoot` is forwarded to `HAKONIWA_PDU_ENDPOINT_PREFIX`.
- `ExtraPrefixPaths` is optional and can be used for packages such as `glfw3`.

## Detailed Run Commands

### C++ samples

- Forklift:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

- Forklift unit (no cargo, useful for auto-control tests):
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

- TurtleBot3 (Hakoniwa asset + endpoint gamepad + 2D LiDAR):
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim
```

- TurtleBot3 (switch LiDAR spec):
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

### Python samples

- Minimal auto control:
```bash
python -m python.forklift_simple_auto config/custom-compact.json
```

- For unit model:
```bash
python -m python.forklift_simple_auto config/forklift-unit-compact.json --forward-distance 1.5 --backward-distance 1.5 --move-speed 0.7
```

- API control sample:
```bash
python -m python.forklift_api_control config/safety-forklift-pdu-compact.json config/monitor_camera_config.json
```

- Gamepad sample:
```bash
python -m python.forklift_gamepad config/custom-compact.json
```

- TurtleBot3 gamepad control:
```bash
python python/tb3_gamepad.py
```

- LiDAR visualizer:
```bash
python python/lidar_visualizer.py
```

---

## TurtleBot3 2D LiDAR

The TurtleBot3 Burger sample includes a MuJoCo-based 2D LiDAR implementation.

- 360-degree raycast
- scan-frame generation based on the selected sensor profile
  - e.g. 10 Hz / 100 ms for `urg-04lx-ug01.json`
  - e.g. 5 Hz / 200 ms for `lds-01.json` and `lds-02.json`
- `LaserScan`-compatible PDU published on Hakoniwa
- Point cloud inspection via Python visualizer

To avoid MuJoCo ray self / near-body interference around the LiDAR mount, the implementation detects self-geometry hits and retries raycasting just beyond the hit point. This avoids relying on a large fixed origin offset and keeps close obstacle perception more natural.

## Sensor Noise Profiles

LiDAR behavior can be switched by sensor config JSON.

- `config/sensors/lidar/lds-01.json`
  - noisy profile close to TurtleBot3 standard `LDS-01`
  - range: 0.12-3.5 m
  - scan: 5 Hz, 1.0 deg resolution
  - spec: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/

- `config/sensors/lidar/lds-02.json`
  - longer-range profile close to TurtleBot3 `LDS-02`
  - range: 0.16-8.0 m
  - scan: 5 Hz, 1.0 deg resolution
  - spec: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/

- `config/sensors/lidar/urg-04lx-ug01.json`
  - cleaner profile based on Hokuyo `URG-04LX-UG01`
  - range: 0.02-5.56 m
  - scan: 10 Hz, 0.3515625 deg resolution
  - useful for comparing clearer obstacle contours against LDS-style profiles

Switch example:
```bash
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-01.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/lds-02.json
./src/cmake-build/main_for_sample/tb3/tb3_sim config/sensors/lidar/urg-04lx-ug01.json
```

This is intended to capture a practical Sim2Real point: changing sensors changes perception quality.

## Docker (Ubuntu 24.04)

Create image:
```bash
bash docker/create-image.bash
```

Run:
```bash
bash docker/run.bash
```

Build in container:
```bash
bash build.bash
```

Notes:
- Ubuntu + Docker: GUI supported
- macOS + Docker: treat as **headless recommended**
```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

---

## Advanced: Forklift / Context Save-Restore

The forklift sample includes advanced MuJoCo context save/restore support.  
It is positioned as an advanced sample for long-running experiments, stop/resume workflows, failure recovery, and future handoff experiments.

## Advanced: Context Save/Restore (MuJoCo)

### Goal

- Long-run continuity
- Better stop/resume operations
- Failure recovery support
- Prerequisite for future RD ownership handoff

### Boundary design (saved scope)

Saved via `HakoniwaMujocoContext` (`include/hakoniwa_mujoco_context.hpp`):
- Forklift subtree physical state (not full-world)
- Control state (`phase`, `target_v`, `target_yaw`, `target_lift`, `step`)

### Adopted context spec (implemented)

Saved content:
- `ForkliftState`
  - Forklift-subtree `qpos[]`
  - Forklift-subtree `qvel[]`
  - Forklift-subtree `qacc[]`
  - Forklift-subtree `qacc_warmstart[]`
  - Forklift-subtree `qfrc_applied[]`
  - Forklift-subtree `xfrc_applied[]`
  - Forklift actuators `ctrl[]` (`left_motor`, `right_motor`, `lift_motor`)
  - `act[]` (`mjData.act`)
  - (compat/debug snapshot) base/lift pose/vel fields
- `ControlState`
  - `phase`
  - `target_linear_velocity`
  - `target_yaw_rate`
  - `target_lift_z`
  - `sim_step`
  - PID internal states (`lift`, `drive_v`, `drive_w`)

Save format and behavior:
- state file format: `v8` (reads `v7` / `v6` / `v5` / `v4` / `v3` / `v2` / `v1` for backward compatibility)
- autosave interval: `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS` (default `1000`)
- save path: `HAKO_FORKLIFT_STATE_FILE` (default under `./tmp/`)
- v8 design intent: expand saved boundary to forklift-subtree dynamics + actuator/PID internal state, based on observed divergence when fork/lift context was insufficient.

Restore behavior (`main_unit.cpp`):
- apply physics state first, then restore lift target
- apply restored `target_linear_velocity` / `target_yaw_rate`
- keep `phase=2` latch in-session to avoid unintended phase flip

### Boundary design (not saved)

- External object states (cargo/shelf/etc.)
- Internal states of external processes (e.g., Python internal state)

This is intentionally **not** a full-world snapshot.

### Save / Restore

- Save:
  - periodic autosave
  - save on shutdown
- Restore:
  - if state file exists, restore
  - otherwise start fresh

Compatibility:
- state restore assumes **the same model XML** and **the same MuJoCo version** (`MUJOCO_VERSION.txt`).

Environment variables:
- `HAKO_FORKLIFT_STATE_FILE`
- `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`
- `HAKO_FORKLIFT_MOTION_GAIN`
- `HAKO_FORKLIFT_TRACE_FILE` (default: `./logs/forklift-unit-trace.csv`)
- `HAKO_FORKLIFT_TRACE_EVERY_STEPS` (default: `10`)

Example:
```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=1000 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

### phase handling

- `phase=2` (return path) is latched in-session
- restored target values are applied for stable post-resume behavior

### Log checks

- `logs/forklift-unit-run.log`: C++ run log
- `logs/control-run.log`: Python run log
- `logs/forklift-unit-recovery.log`: audit log (`START/AUTOSAVE/END`)
- `logs/forklift-unit-trace.csv`: objective continuity trace (time series)

Success signals (Phase1 resume / `phase=2` latch check):
- `START restored=yes ... phase=2 ...`
- `Resume control phase=2 ...`
- `AUTOSAVE` still shows `phase=2` after resume
Note: `phase=2` here is the forklift control return-path flag, not FAQ “Phase2” (complex contact/cargo/shelf scope).

Logs are append mode (`tee -a`).
Use `START restored=no/yes` to separate first and second runs.

### Continuity graph (objective check)

Generate continuity plots from trace CSV:

```bash
python -m python.plot_forklift_continuity \
  --csv logs/forklift-unit-trace.csv \
  --output logs/forklift-unit-continuity.png \
  --window-sec 8
```

This overlays pre-restart and post-restore sessions for:
- `pos_x`
- `body_vx` (with `target_v`)
- `yaw`
- `lift_z`
- `phase`

How to read:
- `pos_x`: post-restore should start near the pre-restart tail value.
- `body_vx` vs `target_v`: transient mismatch is acceptable, but convergence should be fast.
- `phase`: should continue from restored phase (no unintended reset).
- `yaw` / `lift_z`: small discontinuity is acceptable; persistent drift indicates restore mismatch.

Pass criteria (practical):
- `START restored=yes` appears in `logs/forklift-unit-recovery.log`.
- In the first few hundred milliseconds after restart, trajectories trend toward the pre-restart curve.
- `phase` continuity is preserved through resume and subsequent autosave logs.

Acceptance (Phase1, `sim_step` aligned, strict):
- `mean(|Δbody_vx|) <= 1e-3`
- `max(|Δbody_vx|) <= 1e-2`
- `max(|Δpos_x|) <= 1e-3`
- `phase` continuity: identical
- `max(|Δlift_z|) <= 1e-4`
Note: official evidence happened to show zero diff; thresholds are kept as engineering tolerance for reproducibility across environment/runtime noise.

Evaluation protocol:
- Official comparison window: `4010..4860`
- Alignment: `sim_step` (baseline vs resumed)
- Trace source: `logs/forklift-unit-trace.csv` (column definitions apply)
- `Δx` definition: `(baseline - resumed)` and thresholds apply to `|Δx|`
- Trace sampling: `HAKO_FORKLIFT_TRACE_EVERY_STEPS=10` (official evidence condition)
- Metrics include the immediate post-restore window (stricter; operational resume procedure is part of the spec)

Robustness note:
- `python.plot_forklift_continuity` skips partially written CSV rows (e.g., interrupted append on `Ctrl+C`).

### Measured restore evidence

Confirmed in `forklift_unit` restart tests:
- Date: 2026-02-23
- MuJoCo: v3.5.0 (from `MUJOCO_VERSION.txt`)
- State format: `v8`

Observed (latest run):
- `logs/forklift-unit-recovery.log`: `START restored=yes ... step=4000 ... target_v=0.700000 ... target_lift=0.171200 ...`
- `logs/forklift-unit-run.log`: `Resume control phase=1 target(v,yaw,lift)=(0.7, -0, 0.1712) step=4000`
- Numeric continuity check on common steps (`4010..4860`, 86 points):
  - `delta_vx mean/min/max = 0.0 / 0.0 / 0.0`
  - `delta_target_lift mean/min/max = 0.0 / 0.0 / 0.0`

This confirms step-aligned continuity for current scope (forklift subtree + control state).

Official resume evidence package:
- `evidence/official-resume-2026-02-23-v8/`
- Includes copied raw logs + plots + `summary.txt`

Position evidence:
![Resume Position Overlay](evidence/official-resume-2026-02-23-v8/position_overlay.png)

Velocity evidence:
![Resume Velocity Overlay](evidence/official-resume-2026-02-23-v8/velocity_overlay.png)

Acceleration evidence (finite difference of `body_vx`):
![Resume Acceleration Overlay](evidence/official-resume-2026-02-23-v8/acceleration_overlay.png)

---

## Integration Test (forklift_unit)

Purpose:
- Python control can be re-run with the same args
- C++ side can save/restore
- phase continuity (especially phase-2 resume) can be verified

3 terminals:

1. sim
```bash
bash forklift-unit.bash
```

2. control
```bash
FORWARD_GOAL_X=5.0 HOME_GOAL_X=0.0 GOAL_TOLERANCE=0.03 bash control.bash
```

3. start
```bash
hako-cmd start
```

Resume test:
1. stop sim with `Ctrl+C`
2. restart sim with same command
3. re-run control with same args
4. verify `restored=yes` and `phase=2` in logs

---

## Evidence Workflow (Phase1)

Recommended sequence:
1. Run baseline (no restart), generate plot, move artifacts.
2. Run resume test (stop/restart), generate plot, move artifacts.
3. Compare both evidence folders.

Move logs/plots from `logs/` to `evidence/<case_name>/`:

```bash
bash evidence/move-logs-to-evidence.bash phase1-baseline-01
bash evidence/move-logs-to-evidence.bash phase1-resume-01
```

Each evidence folder stores:
- `control-run.log`
- `forklift-unit-run.log`
- `forklift-unit-recovery.log`
- `forklift-unit-trace.csv`
- `forklift-unit-continuity.png`
- `meta.txt` (capture timestamp + MuJoCo version)

Graph interpretation is documented in:
- `Context Save/Restore -> Continuity graph (objective check)`
  - `How to read`
  - `Pass criteria (practical)`

---

## Advanced: RD-light Handoff

RD-light is an advanced / experimental single-node ownership handoff demo for forklift assets.  
It is not RD-full, and final control-plane semantics remain out of scope here.

### Position in RD Architecture (Current)

This repository does not implement RD control-plane logic itself.
It provides the **data-plane physics execution base** required for RD.

Responsibilities:
- `hakoniwa-rd-core`
  - Ownership transitions (Owner/NonOwner)
  - Epoch/commit-point control
  - Bridge rewiring and control-plane consistency
- `hakoniwa-mujoco-robots`
  - High-fidelity EU execution with MuJoCo
  - PDU I/O
  - Context save/restore (continuity prerequisite)

Key boundaries:
- This repository is a **prerequisite implementation**, not complete RD.
- Ownership transition and commit-point decisions remain RD-core responsibility.
- This repository currently has no RD control API (ownership transfer request/accept).
  Integration is planned in the roadmap via context handoff design.

### Guarantees / Non-goals / Interfaces

Guarantees (this repository):
- Data Plane execution for MuJoCo EU in Hakoniwa
- RD-light minimal handoff on single node:
  - ownership release / activation
  - RuntimeContext save/restore continuity
  - single-owner operation
  - standby non-interference operation

Non-goals (out of scope in this repository):
- RD-full Control Plane semantics finalization
- commit-point meaning finalization and global decision authority
- epoch consistency guarantee across distributed nodes
- `d_max` guarantee and drift repair
- bridge rewiring completion confirmation

Interfaces:
- PDU interface (RuntimeStatus / RuntimeContext / robot PDUs)
- Callback boundary for context save/restore payload
- Asset-level ownership status (`owner=yes/no`) and handoff logging

### RD-light (Implemented Here)

This repository includes **RD-light**, a lightweight single-node ownership handoff implementation for experiments.  
It is not a replacement for the full RD control plane; it is a practical asset-side prerequisite implementation.

- Purpose:
  - switch ownership between two independent MuJoCo assets
  - hand off state via `RuntimeStatus` / `RuntimeContext`
  - validate motion continuity after handoff (data-plane continuity)
- Scope:
  - single-node experimental setup
  - final commit-point semantics and distributed rewiring remain in `hakoniwa-rd-core`
- Entry points:
  - `forklift-1.bash` (initial owner)
  - `forklift-2.bash` (initial standby)
- Design details:
  - [`rd-design.md`](rd-design.md)
  - Quick links:
    - [State transition rules](rd-design.md#rd-state-transition-rules)
    - [Context specification](rd-design.md#rd-context-specification)
    - [Switch trigger](rd-design.md#rd-switch-trigger)
    - [Stabilization features](rd-design.md#rd-stabilization-features)
    - [Failure behavior](rd-design.md#rd-failure-behavior)
    - [Class design](rd-design.md#14-クラス設計実装反映)
    - [TODO](rd-design.md#rd-todo)

### Safe Handoff Condition (User Responsibility)

Handoff timing is a user / upper-layer responsibility.
RD-light does not automatically certify physical safety at arbitrary handoff instants.

The position of this project is explicit.  
**The responsibility for how far to push high-fidelity complex physics belongs to the user (scenario designer / upper-layer control)**,  
while this repository provides the Data Plane and handoff mechanism.  
Therefore, **preconditions before entering truly critical simulation phases (handoff boundary, velocity constraints, contact-state constraints, etc.) must be defined by the user side**.

MUST NOT:
- do not handoff during active contact/collision
- do not handoff immediately before predicted collision
- do not handoff during grasp/constraint-active states
- do not handoff when strong external-object contact constraints are active

SHOULD:
- handoff in free space
- handoff under stable motion (small acceleration / bounded angular rate / low contact count)
- use explicit safe boundary design in scenario definition

Failure semantics when violated:
- physical continuity is **not guaranteed**
- divergence / bounce / sudden transients are within supported failure behavior

Current demo interpretation:
- the 1m boundary is a **safe-boundary design example** for handoff timing.
- this is a scenario-level policy, not an automatic RD-full safety proof.

### Design Principle: Explicitly Decide What Not to Guarantee

This repository does not attempt to guarantee full physical continuity for every possible situation.  
Instead, it assumes **ownership handoff before entering high-risk contact/collision zones**.

- Expanding guarantees into contact-heavy edge cases too early causes implementation and operational complexity to explode.
- Therefore, boundaries are fixed first: what is guaranteed here, and what is delegated to upper-layer control/operations.
- This is not a weakness; it is an intentional boundary design for scalable distributed simulation.

## Advanced Reading: Hakoniwa Core Summary

Read this repository with the following 3 layers in mind:

- `hakoniwa-core-cpp` (simulation hub core)
  - Shared memory (`mmap`) and synchronization core
  - `hako-master` manages runtime state and PDU areas
- `hakoniwa-core-pro` (runtime/API layer)
  - Asset APIs, command tools, and execution control
  - Typical APIs: `hako_conductor_start()`, `hako_asset_register()`, `hako_asset_start()`, `hako_asset_pdu_read()`, `hako_asset_pdu_write()`
- `hakoniwa-pdu-registry` (PDU type/size artifacts)
  - Generated type/offset/size/converter artifacts derived from ROS msg
  - Binary layout: `[MetaData(24B)] + [BaseData] + [HeapData]`
  - MetaData is fixed-length 24B in the current PDU spec (ref: [`hakoniwa-pdu-registry` README](thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/README.md))

Minimum knowledge for reading samples:
- `pdu_size` is total PDU buffer size (including metadata), not raw type size.
  - Example: `Int32` payload is 8B, but config uses `24 + 8 = 32`.
- Config format:
  - C++/Python: compact (`pdudef + pdutypes`)
  - Minimum Python package: `hakoniwa-pdu >= 1.3.7`
- Runtime shared files are typically under `/var/lib/hakoniwa/mmap`.

Recommended reading order:
1. `src/main_for_sample/forklift/main_unit.cpp`
2. `python/forklift_simple_auto.py`
3. `config/forklift-unit*.json` and `config/safety-forklift-pdu*.json`

## Advanced Reading: Config Rule

### compact only (must read)

- C++ simulator / Python controller: **compact JSON**
  - e.g. `forklift-unit-compact.json`, `custom-compact.json`, `safety-forklift-pdu-compact.json`
- Python runtime requirement: `hakoniwa-pdu >= 1.3.7`

If runtime `hakoniwa-pdu` is older, startup may succeed but PDU resolution can fail (`channel=-1`, `size=-1`).

---

## FAQ

### Q1. Does this repository implement RD itself?
A. RD-light is implemented, RD-full is not.
Implemented here: single-node minimal handoff in the Data Plane (ownership release/activation + context handoff + single owner operation).
Not implemented here: RD-full Control Plane semantics (commit-point finalization authority, distributed epoch guarantees, `d_max` guarantee, bridge rewiring completion confirmation).

### Q2. How does commit-point relate to MuJoCo save?
A. Currently they are not directly linked.
Save is operational autosave.
Future hook point is explicit: commit-point reached in RD control-plane -> invoke context-save API at the data-plane boundary.
So current behavior is practical continuity support, not final commit-point semantics.

### Q3. How is this aligned with `d_max`?
A. This repository focuses on local physics execution.
`d_max` guarantee belongs to RD semantics in upper layers; this repository does not provide `d_max` guarantee by itself.

### Q4. Why not full-world snapshot?
A. Intentional phased scope.
Current saved scope is forklift subtree + control state (with selected MuJoCo dynamics buffers).
External objects are future extension.

### Q5. Do you save MuJoCo solver internal state?
A. Partially.
Saved data includes forklift-subtree `qpos` / `qvel` / `qacc`, `qacc_warmstart`, `qfrc_applied`, `xfrc_applied`, actuator `ctrl`, `act`, and control state.
This is broader than minimal pose-only save, but still not a full-world MuJoCo snapshot.
These buffers were chosen to stabilize contact/actuation resumption without targeting a full MuJoCo internal snapshot.

### Q6. Is physical continuity perfectly guaranteed?
A. No.
Target is semantic continuity, not strict micro-level physical continuity.

### Q7. Can restore work across model/version changes?
A. Not guaranteed.
Restore assumes same model XML and same MuJoCo version.

### Q8. Is legacy/compact coexistence a design issue?
A. Current policy is compact-only. Legacy examples/configs were removed from this repository.

### Q9. How is this different from HLA/FMI positioning?
A. This design centers on explicit PDU contracts, EU-level ownership, and commit-point semantics.
Its positioning is different from master-algorithm-centric synchronization styles.

### Q10. How is input handled at restore boundary?
A. Controller runs as a Hakoniwa asset (tick-synchronized), and input is applied immediately at restore boundary.
So there is no hold-time tuning operation in the current flow.

### Q11. Why is there no Phase2 evidence (cargo/shelf/complex contact)?
A. The current objective is to validate **ExecutionUnit continuity (Phase1)** as an RD prerequisite.
This repository provides data-plane continuity implementation; without RD control-plane coupling (commit-point-triggered save, ownership handoff rule, epoch-consistent handoff timing), Phase2 evidence has low return relative to required effort.

Phase2 (cargo/shelf/complex contact) will be executed after:
- commit-point-triggered context-save hook from RD control-plane
- handoff rule finalization (which state timing is authoritative at transfer)
- boundary finalization for external-object context scope

Therefore, Phase1 is treated as the official evidence in the current scope, and Phase2 is explicitly out of scope for now.

Note: Phase1 evidence targets semantic continuity under the defined scope; it is not a claim of full-world physical determinism.

### Q12. Is this a "glass castle"? (Only valid in a special setup?)
A. This concern is valid.  
This repository is not a universal "restore everything" solution; it is a **scoped Data Plane implementation**.

- Validity conditions:
  - single-owner operation
  - Safe Handoff Condition (avoid handoff during contact, near-collision, or active constraints)
  - same model XML / same MuJoCo version
- What it provides:
  - ownership handoff + continuity within that scope (Phase1)
- What is still non-guaranteed:
  - general continuity with contact/external objects (Phase2)

So this is not "works only by accident"; it is a staged design with explicit applicability boundaries.

This FAQ reflects the current implementation scope.
For final semantics and distributed extensions, see [Hakoniwa Design Docs](https://github.com/hakoniwalab/hakoniwa-design-docs).

---

## Samples

- `src/main_for_sample/forklift/main.cpp`: forklift basic integration
- `src/main_for_sample/forklift/main_unit.cpp`: unit model verification
- `src/main_for_sample/tb3/main.cpp`: TurtleBot3 sample (Hakoniwa asset / endpoint / 2D LiDAR)
- `python/tb3_gamepad.py`: TurtleBot3 Python controller asset (PS4/DualSense)
- `python/lidar_visualizer.py`: generic LiDAR visualizer (world view)
- `config/sensors/lidar/lds-01.json`: TurtleBot3 LDS-01-like noisy LiDAR profile
- `config/sensors/lidar/lds-02.json`: TurtleBot3 LDS-02-like longer-range LiDAR profile
- `config/sensors/lidar/urg-04lx-ug01.json`: Hokuyo URG-04LX-UG01-like cleaner LiDAR profile

---

## Roadmap

- Windows run flow (build/run/log)
- Keep compact-only runtime checks and diagnostics (`hakoniwa-pdu` version / PDU resolution)
- Operational hardening of Python controller asset mode (tick-synchronized path)
- Expand saved scope (cargo/shelf/etc.)
- Automated restore consistency checks (log verification scripts)
- Safe handoff diagnosability (optional logs: reason=`contact_active` / `near_collision` / `constraint_active`)
- RD integration for context handoff design

---

## License

MIT License
