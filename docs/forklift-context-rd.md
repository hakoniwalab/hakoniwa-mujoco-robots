# Forklift Context Save/Restore And RD-light

This document collects the advanced forklift notes that are too detailed for the top-level README.

The forklift sample includes MuJoCo context save/restore support. It is intended for long-running experiments, stop/resume workflows, failure recovery, and RD-light handoff experiments.

## Context Save/Restore

Goals:

- Continue long-running experiments after a restart.
- Improve stop/resume operation.
- Provide failure-recovery support.
- Provide a prerequisite for future RD ownership handoff.

Saved scope:

- Forklift subtree physical state, not the full world.
- Control state: `phase`, `target_v`, `target_yaw`, `target_lift`, `step`.
- Forklift actuator `ctrl[]` values.
- `mjData.act`.
- PID internal states for lift and drive control.

Not saved:

- Cargo, shelves, and other external object states.
- Internal state of external processes such as Python controllers.

Restore assumes the same model XML and the same MuJoCo version. The active MuJoCo version is managed by [`MUJOCO_VERSION.txt`](../MUJOCO_VERSION.txt).

## State Format

The current state file format is `v8`.

The implementation reads older formats for compatibility, but new evidence should be captured with the current format.

The `v8` boundary includes forklift-subtree dynamics plus actuator and PID internal state. This was expanded after earlier restore attempts showed divergence when fork/lift context was too narrow.

## Environment Variables

- `HAKO_FORKLIFT_STATE_FILE`: state file path.
- `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`: autosave interval in simulation steps.
- `HAKO_FORKLIFT_MOTION_GAIN`: forklift motion gain.
- `HAKO_FORKLIFT_TRACE_FILE`: trace CSV path, default `./logs/forklift-unit-trace.csv`.
- `HAKO_FORKLIFT_TRACE_EVERY_STEPS`: trace sampling interval, default `10`.
- `HAKO_CONTROLLER_MODE`: `asset` or `external`; `control.bash` defaults to `asset`.
- `HAKO_CONTROLLER_ASSET_NAME`: controller asset name.
- `HAKO_CONTROLLER_DELTA_USEC`: controller tick period.

Example:

```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=1000 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

## Logs

- `logs/forklift-unit-run.log`: C++ run log.
- `logs/control-run.log`: Python controller log.
- `logs/forklift-unit-recovery.log`: audit log with `START`, `AUTOSAVE`, and `END`.
- `logs/forklift-unit-trace.csv`: objective continuity trace.

Useful success signals:

- `START restored=yes`
- `Resume control phase=2`
- `AUTOSAVE` continues to show the restored phase after resume.

## Continuity Check

Generate plots from the trace CSV:

```bash
python -m python.plot_forklift_continuity \
  --csv logs/forklift-unit-trace.csv \
  --output logs/forklift-unit-continuity.png \
  --window-sec 8
```

The plot overlays pre-restart and post-restore sessions for `pos_x`, `body_vx`, `yaw`, `lift_z`, and `phase`.

Practical pass criteria:

- `START restored=yes` appears in `logs/forklift-unit-recovery.log`.
- In the first few hundred milliseconds after restart, trajectories trend toward the pre-restart curve.
- `phase` continuity is preserved through resume and later autosave logs.

Strict Phase1 acceptance criteria used by the evidence workflow:

- `mean(|delta body_vx|) <= 1e-3`
- `max(|delta body_vx|) <= 1e-2`
- `max(|delta pos_x|) <= 1e-3`
- `phase` continuity: identical
- `max(|delta lift_z|) <= 1e-4`

## Measured Restore Evidence

Confirmed in `forklift_unit` restart tests:

- Date: 2026-02-23
- MuJoCo: v3.5.0 at the time of this evidence capture
- State format: `v8`

The current MuJoCo version used by this repository is managed separately in [`MUJOCO_VERSION.txt`](../MUJOCO_VERSION.txt).

Observed:

- `logs/forklift-unit-recovery.log`: `START restored=yes ... step=4000 ... target_v=0.700000 ... target_lift=0.171200 ...`
- `logs/forklift-unit-run.log`: `Resume control phase=1 target(v,yaw,lift)=(0.7, -0, 0.1712) step=4000`
- Numeric continuity check on common steps `4010..4860` showed zero difference for the recorded velocity and lift-target metrics in that evidence run.

Evidence package:

- `evidence/official-resume-2026-02-23-v8/`

## Integration Test

Use three terminals.

1. Simulator:

```bash
bash forklift-unit.bash
```

2. Controller:

```bash
FORWARD_GOAL_X=5.0 HOME_GOAL_X=0.0 GOAL_TOLERANCE=0.03 bash control.bash
```

3. Start trigger:

```bash
hako-cmd start
```

Resume test:

1. Stop the simulator with `Ctrl+C`.
2. Restart the simulator with the same command.
3. Re-run the controller with the same arguments.
4. Verify `restored=yes` and the expected `phase` in logs.

## Evidence Workflow

Recommended sequence:

1. Run a baseline without restart, generate plots, and move artifacts.
2. Run a stop/restart resume test, generate plots, and move artifacts.
3. Compare both evidence folders.

Move logs and plots:

```bash
bash evidence/move-logs-to-evidence.bash phase1-baseline-01
bash evidence/move-logs-to-evidence.bash phase1-resume-01
```

Each evidence folder stores copied logs, trace CSV, continuity plots, and `meta.txt`.

## RD-light Handoff

RD-light is an advanced single-node ownership handoff demo for forklift assets. It is not RD-full.

This repository provides the data-plane physics execution base required for RD:

- High-fidelity execution with MuJoCo.
- PDU I/O.
- Context save/restore as a continuity prerequisite.

Out of scope here:

- RD-full control-plane semantics.
- Global commit-point finalization.
- Distributed epoch guarantees.
- `d_max` guarantees and drift repair.
- Bridge rewiring completion confirmation.

Entry points:

- `forklift-1.bash`: initial owner.
- `forklift-2.bash`: initial standby.

More detailed RD transition notes are in [`rd-design.md`](../rd-design.md).
