# AGENTS.md

Guidance for coding agents working in this repository.

## Project Shape

This repository provides MuJoCo-based robot simulation assets for Hakoniwa.

Main areas:

- `src/`: C++ simulator implementation.
- `src/sensors/`: reusable sensor components and PDU converters.
- `src/actuator/`: joint actuator implementation.
- `examples/`: small standalone examples for sensors and actuators.
- `python/`: Python controllers and visualizers.
- `config/`: PDU, sensor, and actuator JSON configs.
- `models/`: MuJoCo XML models.
- `docs/`: advanced notes that should not crowd the top-level README.

## First Commands

Run these before making broad assumptions about the local machine:

```bash
./doctor.bash
```

Use `PYTHON_CMD=/path/to/python ./doctor.bash` when a specific Python environment should be checked.

If the doctor reports missing prerequisites, fix those first or clearly mention the failure in your final note.

Configure/build uses `src/cmake-build`:

```bash
cmake -S src -B src/cmake-build
cmake --build src/cmake-build --target forklift_sim forklift_unit_sim tb3_sim joint-actuator-example color-camera-example ultrasonic-example -j4
```

For a quick syntax check of shell changes:

```bash
bash -n doctor.bash
```

Always run:

```bash
git diff --check
```

## Dependency Notes

- MuJoCo version is controlled by `MUJOCO_VERSION.txt`.
- Python tools require `hakoniwa-pdu >= 1.6.1`.
- `hakoniwa-core-pro` and `hakoniwa-pdu-endpoint` are expected to be installed C++ packages unless `HAKO_USE_THIRDPARTY_HAKONIWA=ON` is used.
- `glfw3` is needed for viewer-enabled builds.

## Tests And Manual Checks

Viewer examples are interactive and may need a GUI:

- `examples/sensors/ultrasonic`
- `examples/sensors/color_camera`
- `examples/actuators/joint`

Do not claim viewer behavior was verified unless you actually ran the viewer or inspected the produced artifact, such as a PNG.

Headless-safe checks include:

- `bash -n doctor.bash`
- `git diff --check`
- CMake configure
- non-interactive target builds
- sensor unit test targets when configured

## Documentation Expectations

When adding or changing user-facing behavior:

- Update the closest README.
- Keep top-level README focused on first-run setup, examples, and troubleshooting.
- Move long design notes to `docs/`.
- Keep examples discoverable from `examples/README.md`.
- If a command output path changes, document the default path.

## Coding Style

- Prefer existing local patterns over new abstractions.
- Keep example code readable as API usage, with support code separated only when it hides setup noise.
- Use `rg` for searches.
- Use `apply_patch` for manual edits.
- Do not remove or revert user changes unless explicitly asked.

## Common Pitfalls

- `pip` and `python` can point to different environments. Use `python -m pip`, and use `PYTHON_CMD` with `doctor.bash` when needed.
- CMake package roots should point to install prefixes, not build directories.
- Docker on macOS should be treated as headless.
- RD-light and context save/restore are advanced topics; keep them out of first-run instructions.
