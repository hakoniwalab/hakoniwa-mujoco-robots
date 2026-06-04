# Contributing

Thanks for improving `hakoniwa-mujoco-robots`.

This repository combines C++ MuJoCo simulators, Python tools, Hakoniwa PDU configs, and interactive examples. Please keep changes easy to reproduce.

## Before You Start

From the repository root:

```bash
./doctor.bash
```

Fix any `FAIL` items before building. If you cannot fix one locally, mention it in your issue or pull request.

## Build

Default build:

```bash
./build.bash
```

Representative target build:

```bash
cmake -S src -B src/cmake-build
cmake --build src/cmake-build --target forklift_sim forklift_unit_sim tb3_sim joint-actuator-example color-camera-example ultrasonic-example -j4
```

## Checks Before A Pull Request

Run:

```bash
bash -n doctor.bash
git diff --check
```

When relevant, also run the target you changed. Examples:

```bash
cmake --build src/cmake-build --target tb3_sim
cmake --build src/cmake-build --target color-camera-example
cmake --build src/cmake-build --target joint-actuator-example
```

Viewer-based behavior is manual. If you verify a viewer example, say exactly what you ran and what you observed.

## Documentation

Update documentation when changing:

- commands
- required dependencies
- default output paths
- JSON config shape
- examples
- public sensor or actuator APIs

Keep the top-level README focused on first-run setup, common troubleshooting, and example entry points. Put long design notes in `docs/`.

## Issue Reports

For setup or build failures, include:

- OS and architecture
- `./doctor.bash` output
- command that failed
- first relevant error block
- whether you use system packages, Homebrew, vcpkg, or custom install prefixes

For viewer issues, include:

- GUI environment
- MuJoCo version from `MUJOCO_VERSION.txt`
- whether the issue also happens with a standalone example

## Pull Requests

Good pull requests include:

- a short summary
- why the change is needed
- commands used for verification
- manual viewer checks, if any
- README/docs updates, if relevant

Keep unrelated refactors out of feature or fix PRs.
