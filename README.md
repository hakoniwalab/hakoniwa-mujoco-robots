# hakoniwa-mujoco-robots

English | [日本語](README-ja.md)

## TL;DR
- This repository provides MuJoCo-based physics assets for Hakoniwa.
- It connects a C++ MuJoCo simulator and Python controllers via PDU contracts.
- It includes context save/restore for forklift state + control state.
- Current migration rule: C++ uses compact JSON, Python uses legacy JSON.
- Fast start uses 3 terminals: simulator, controller, and `hako-cmd start`.

## Demo Video
- Runtime handoff demo (RD-lite, dual forklift assets):
  - [![Watch the demo](https://img.youtube.com/vi/xaJJ1wEgNR8/hqdefault.jpg)](https://www.youtube.com/watch?v=xaJJ1wEgNR8)

### What This Video Demonstrates
- The two MuJoCo viewers are two asset instances of the same logical EU (forklift).
- The **1m forward point is defined as the world boundary**, and used as the handoff point.
- This is **not** a reinforcement-learning demo; it is an **RD-lite ownership handoff demo**.
- Normally, only one side is owner (active control + PDU publish), while the other side is standby.
- At the switching threshold, owner updates `RuntimeStatus/RuntimeContext`; standby restores context and becomes owner.
- Standby is shown as semi-transparent and non-interfering; after handoff, roles are swapped.
- The two sides are **separate MuJoCo physics assets** (not a single shared simulator instance).
- The key point is **seamless execution-right delegation** between independent assets while preserving motion continuity.

### How to Read the Logs
- `ownership release requested`: current owner requested handoff
- `ownership activated`: peer restored context and became owner
- `status step=... owner=yes/no`: local ownership state at that step
- `dist_to_release`, `dist_to_home`: distance metrics for switch conditions

### Scope of This Claim
- This demo does **not** claim full RD control-plane implementation.
- It demonstrates the MuJoCo asset-side prerequisite (data-plane continuity with context handoff).

---

## Why

Hakoniwa is a PDU-based simulation platform for multi-process, multi-language, and distributed setups.
This repository integrates MuJoCo as a high-fidelity physics asset inside that ecosystem.

Main goals:
- Provide a high-fidelity physics execution layer in Hakoniwa.
- Keep Python control logic decoupled from C++ simulation.
- Support experiment continuity (save/restore), as a prerequisite for future RD integration.

Why save/restore matters:
- Resume long-running experiments.
- Continue after manual stop/restart.
- Recover from failures.
- Prepare for future ownership transfer workflows.

---

## What

Included:
- MuJoCo models (forklift, rover)
- Hakoniwa-integrated C++ samples
- Python controller samples
- Docker environment (Ubuntu 24.04)
- Forklift context save/restore (state file + audit logs)

Directory map:
- `models/`: MuJoCo XML models
- `config/`: PDU JSON configs
- `src/`: C++ simulator implementation
- `python/`: Python controllers
- `docker/`: Docker scripts
- `logs/`: generated logs
- `tmp/`: generated state files

---

## Architecture

Hakoniwa works as the hub, and MuJoCo (C++) and Python controllers communicate over PDU.

- **Hakoniwa**: synchronization and PDU runtime
- **MuJoCo C++ Asset**: physics stepping + PDU read/write
- **Python Controller**: target-value control logic
- **PDU JSON**: contract of channels/types/sizes

```text
+-------------------+         PDU (shared contract)         +----------------------+
| Python Controller |  <---------------------------------->  | MuJoCo C++ Simulator |
| (forklift_* .py)  |                                        | (forklift*_sim)      |
+---------+---------+                                        +----------+-----------+
          |                                                            |
          |                   Hakoniwa runtime                         |
          +--------------------(sync / mmap / PDU)---------------------+
```

---

## Hakoniwa Core Summary (Reading Guide)

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
- Migration-period dual config:
  - C++: compact (`pdudef + pdutypes`)
  - Python: legacy (`pdudef`)
- Runtime shared files are typically under `/var/lib/hakoniwa/mmap`.

Recommended reading order:
1. `src/main_for_sample/forklift/main_unit.cpp`
2. `python/forklift_simple_auto.py`
3. `config/forklift-unit*.json` and `config/safety-forklift-pdu*.json`

---

## Position in RD Architecture (Current)

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

### RD Summary (ExecutionUnit / Ownership / commit-point / d_max)

RD is a control model for safe execution ownership transfer of an ExecutionUnit (EU) in distributed runs.
An ExecutionUnit is a logical unit; an ExecutionUnitInstance is its concrete runtime instance per node.
Ownership must remain unique at all times.

Switching is explicit and state-driven. Epoch identifies generations, and next owner activation is coordinated after bridge rewiring is confirmed.
A commit-point is a semantic boundary for responsibility/causality, not a physical simultaneous-start point.

Time consistency assumes bounded drift and uses design limits (`d_max`, or `2*d_max` in distributed paths).
Automatic repair for `d_max` violation and failure recovery is out of scope and handled operationally.

**Note:** RD provides bounded-drift semantics, but automatic repair/failure recovery beyond `d_max` is out of scope.

This README is implementation/operations guidance (**Informative**).
Final RD semantics are defined in (**Normative**):
- **Normative**: final specification of semantics
- **Informative**: implementation/operations guidance (this README)
- [Hakoniwa Design Docs](https://github.com/hakoniwalab/hakoniwa-design-docs)
- [Core Functions (JA)](https://github.com/hakoniwalab/hakoniwa-design-docs/blob/main/src/architecture/core-functions-ja.md)
- [Glossary (JA)](https://github.com/hakoniwalab/hakoniwa-design-docs/blob/main/src/glossary-ja.md)

### RD-Light (Implemented Here)

This repository includes **RD-Light**, a lightweight single-node ownership handoff implementation for experiments.  
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

---

## Migration Rule (Important)

### legacy vs compact (must read)

- C++ simulator: **compact JSON**
  - e.g. `forklift-unit-compact.json`, `safety-forklift-pdu-compact.json`
- Python controller: **legacy JSON**
  - e.g. `forklift-unit.json`, `custom.json`, `safety-forklift-pdu.json`

This is a migration-stage coexistence. Mixing wrong formats often causes "it starts but does not work" behavior.

---

## Prerequisites

## 1) Install hakoniwa-core-pro (required)

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

## 2) OS notes

- macOS: `brew install glfw`
- Ubuntu:
```bash
sudo apt-get update
sudo apt-get install -y libgl1 libgl1-mesa-dri libglx-mesa0 mesa-utils libglfw3-dev
```

---

## Setup

```bash
git clone https://github.com/toppers/hakoniwa-mujoco-robots.git
cd hakoniwa-mujoco-robots
git submodule update --init --recursive
./build.bash
```

- MuJoCo version is managed by `MUJOCO_VERSION.txt`.
- Clean build:
```bash
./build.bash clean
```

---

## Quick Start

Use host execution as the shortest path.
Prepare 3 terminals.

1. Simulator
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

2. Python controller (legacy)
```bash
python -m python.forklift_simple_auto config/forklift-unit.json \
  --forward-distance 2.0 --backward-distance 2.0 --move-speed 0.7
```

3. Start trigger
```bash
hako-cmd start
```

For compatibility, `controll.bash` is temporarily kept and internally calls `control.bash`.

---

## How (Detailed Run Commands)

## C++ samples

- Forklift:
```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

- Forklift unit (no cargo, useful for auto-control tests):
```bash
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

- Rover:
```bash
./src/cmake-build/main_for_sample/rover/rover_sim
```

## Python samples

- Minimal auto control:
```bash
python -m python.forklift_simple_auto config/custom.json
```

- For unit model (legacy):
```bash
python -m python.forklift_simple_auto config/forklift-unit.json --forward-distance 1.5 --backward-distance 1.5 --move-speed 0.7
```

- API control sample:
```bash
python -m python.forklift_api_control config/safety-forklift-pdu.json config/monitor_camera_config.json
```

- Gamepad sample:
```bash
python -m python.forklift_gamepad config/custom.json
```

---

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

## Context Save/Restore (MuJoCo)

## Goal

- Long-run continuity
- Better stop/resume operations
- Failure recovery support
- Prerequisite for future RD ownership handoff

## Boundary design (saved scope)

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

## Boundary design (not saved)

- External object states (cargo/shelf/etc.)
- Internal states of external processes (e.g., Python internal state)

This is intentionally **not** a full-world snapshot.

## Save / Restore

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
- `HAKO_FORKLIFT_RESUME_CMD_HOLD_SEC` (default: `2.0`, ignore all external commands during this fixed window after resume)

Example:
```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=1000 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

## phase handling

- `phase=2` (return path) is latched in-session
- restored target values are applied for stable post-resume behavior

## Log checks

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
- Metrics include the `RESUME_CMD_HOLD_SEC` window (stricter; operational resume procedure is part of the spec)

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

## FAQ

### Q1. Does this repository implement RD itself?
A. No.
It does not implement RD control-plane logic (ownership transition/Epoch control).
It provides physical ExecutionUnit continuity needed by RD.
Ownership transition and commit-point are responsibilities of `hakoniwa-rd-core`.

### Q2. How does commit-point relate to MuJoCo save?
A. Currently they are not directly linked.
Save is operational autosave.
Future hook point is explicit: commit-point reached in RD control-plane -> invoke context-save API at the data-plane boundary.

### Q3. How is this aligned with `d_max`?
A. This repository focuses on local physics execution.
`d_max` guarantee belongs to RD semantics; this repository is the data-plane EU implementation running under that model.

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
A. It is a migration-stage constraint.
Roadmap targets unification toward compact.

### Q9. How is this different from HLA/FMI positioning?
A. This design centers on explicit PDU contracts, EU-level ownership, and commit-point semantics.
Its positioning is different from master-algorithm-centric synchronization styles.

### Q10. Is `HAKO_FORKLIFT_RESUME_CMD_HOLD_SEC` a workaround?
A. It is an intentional disturbance-shield window right after resume.
Immediately after restore, command re-send and internal state settling can overlap; this short window suppresses external command noise to stabilize the resume boundary.
With future commit-point-coupled save/restore, this window can be reduced or removed.

### Q11. Why is there no Phase2 evidence (cargo/shelf/complex contact)?
A. The current objective is to validate **ExecutionUnit continuity (Phase1)** as an RD prerequisite.
This repository provides data-plane continuity implementation; without RD control-plane coupling (commit-point-triggered save, ownership handoff rule, epoch-consistent handoff timing), Phase2 evidence has low return relative to required effort.

Phase2 (cargo/shelf/complex contact) will be executed after:
- commit-point-triggered context-save hook from RD control-plane
- handoff rule finalization (which state timing is authoritative at transfer)
- boundary finalization for external-object context scope

Therefore, Phase1 is treated as the official evidence in the current scope, and Phase2 is explicitly out of scope for now.

Note: Phase1 evidence targets semantic continuity under the defined scope; it is not a claim of full-world physical determinism.

This FAQ reflects the current implementation scope.
For final semantics and distributed extensions, see [Hakoniwa Design Docs](https://github.com/hakoniwalab/hakoniwa-design-docs).

---

## Samples

- `src/main_for_sample/forklift/main.cpp`: forklift basic integration
- `src/main_for_sample/forklift/main_unit.cpp`: unit model verification
- `src/main_for_sample/rover/main.cpp`: rover sample

---

## Roadmap

- Windows run flow (build/run/log)
- Python-side compact format support (remove legacy dependency)
- Expand saved scope (cargo/shelf/etc.)
- Automated restore consistency checks (log verification scripts)
- RD integration for context handoff design

---

## License

MIT License
