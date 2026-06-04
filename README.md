# hakoniwa-mujoco-robots

English | [日本語](README-ja.md)

## TL;DR
- This repository provides MuJoCo-based robot simulation assets for Hakoniwa.
- It supports ROS/URDF-derived robot models, including TurtleBot3 Burger.
- It connects C++ MuJoCo simulators and Python controllers / visualizers through Hakoniwa PDU.
- It includes a TurtleBot3 sample with gamepad control and 2D LiDAR `LaserScan`-compatible PDU output.
- It includes configurable LiDAR noise profiles such as `LDS-01`-like and `URG-04LX-UG01`-equivalent settings.
- It includes forklift samples with context save/restore and RD-light handoff as advanced examples.
- Compact JSON is the default for both C++ and Python (`hakoniwa-pdu >= 1.6.1`).

## Start Here

Use these entry points first. The rest of this README includes advanced notes and experimental background.

| Goal | Read / Run |
| --- | --- |
| Build the repository | [Prerequisites](#prerequisites), then [Setup](#setup) |
| Diagnose the local environment | [`./doctor.bash`](#environment-diagnostics) |
| Run TurtleBot3 with gamepad and LiDAR | [Quick Start: TurtleBot3 + 2D LiDAR](#quick-start-turtlebot3--2d-lidar) |
| Run the forklift sample | [Quick Start: Forklift](#quick-start-forklift) |
| Try small sensor examples | [examples/sensors/README.md](examples/sensors/README.md) |
| Try color-camera PNG capture | [examples/sensors/color_camera/README.md](examples/sensors/color_camera/README.md) |
| Try MJCF position / velocity actuators | [examples/actuators/joint/README.md](examples/actuators/joint/README.md) |
| Read RD-light / context save-restore notes | [docs/forklift-context-rd.md](docs/forklift-context-rd.md) |

Current standalone examples:

```text
examples/sensors/ultrasonic/        ultrasonic range sensor + viewer ray
examples/sensors/color_camera/      RGB camera sensor + PNG capture
examples/actuators/joint/           MuJoCo position / velocity joint actuators
```

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
- See [docs/forklift-context-rd.md](docs/forklift-context-rd.md) and [rd-design.md](rd-design.md) for details

---

## What This Repository Provides

Included:
- MuJoCo robot models
- Hakoniwa-integrated C++ simulators
- Python controllers
- Python visualizers
- PDU configs
- sensor configs
- reusable sensor components under `src/sensors/`
- reusable actuator components under `src/actuator/`
- standalone feature examples under `examples/`
- forklift context save/restore
- RD-light as an advanced demo

Directory map:
- `models/`: MuJoCo XML models
- `config/`: PDU JSON configs
- `config/sensors/`: LiDAR / sensor spec JSON
- `config/actuator/`: actuator binding JSON
- `src/`: C++ simulator implementation
- `src/sensors/`: sensor implementations and PDU converters
- `src/actuator/`: joint actuator implementation
- `python/`: Python controllers / visualizers
- `examples/`: small standalone examples for individual features
- `tests/sensors/`: focused sensor unit / smoke tests
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

### 2) Install hakoniwa-pdu-endpoint (required)

This repository links against the installed C++ `hakoniwa-pdu-endpoint` package.

```bash
git clone https://github.com/hakoniwalab/hakoniwa-pdu-endpoint.git
cd hakoniwa-pdu-endpoint
bash build.bash
sudo bash install.bash
```

If you install it outside `/usr/local/hakoniwa`, set:

```bash
export HAKONIWA_PDU_ENDPOINT_ROOT=/path/to/hakoniwa-pdu-endpoint/install
```

If Hakoniwa core is also outside `/usr/local/hakoniwa`, set:

```bash
export HAKONIWA_CORE_ROOT=/path/to/hakoniwa-core-pro/install
```

### 3) Install hakoniwa-pdu Python package (required for Python tools)

Python controllers and visualizers under `python/` use the `hakoniwa_pdu` module.

```bash
python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

Check the installed version:

```bash
python -m pip show hakoniwa-pdu
```

This is required for commands such as:

```bash
python python/tb3_gamepad.py
python -m python.forklift_simple_auto config/forklift-unit-compact.json
python python/lidar_visualizer.py
```

### 4) OS notes

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
./doctor.bash
./build.bash
```

- MuJoCo version is managed by `MUJOCO_VERSION.txt`.
- `./build.bash` runs a preflight check before CMake and reports missing prerequisites such as `hakoniwa-core-pro`, `hakoniwa-pdu-endpoint`, or `glfw3`.
- If the preflight check blocks a custom setup temporarily, use `HAKO_SKIP_PREFLIGHT=1 ./build.bash`.
- Clean build:
```bash
./build.bash clean
```

## Environment Diagnostics

Run this before `./build.bash` when setting up a new machine:

```bash
./doctor.bash
```

The doctor command checks local prerequisites without building:

- CMake and Git
- `hakoniwa-core-pro` package config
- `hakoniwa-pdu-endpoint` package config
- `hakoniwa-pdu >= 1.6.1` for Python tools
- `glfw3`
- `MUJOCO_VERSION.txt`

Fix reported failures first, then run `./build.bash`.

### Windows (MSVC + PowerShell)

The Windows build is intended to run after `hakoniwa-core-pro` and `hakoniwa-pdu-endpoint` have already been built and installed on Windows.

Prerequisites:
- `hakoniwa-core-pro` is installed under a prefix such as `C:\project\hakoniwa-core-pro\install`
- `hakoniwa-pdu-endpoint` is installed under a prefix such as `C:\project\hakoniwa-pdu-endpoint\install`
- `vcpkg` packages for Windows are available under `C:\project\vcpkg\installed\x64-windows`
- Run the build from Windows PowerShell when possible

PowerShell:

```powershell
.\build-win.ps1 -Clean `
  -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install `
  -HakoniwaPduEndpointRoot C:\project\hakoniwa-pdu-endpoint\install `
  -ExtraPrefixPaths C:\project\vcpkg\installed\x64-windows
```

WSL / Git Bash wrapper:

```bash
./build-win.bash
```

Notes:
- `build-win.ps1` runs a preflight check before CMake and reports missing install roots, missing package config files, and missing `glfw3` / `vcpkg` prefix information.
- The script configures with `-S src`, matching the existing Unix build layout.
- `HakoniwaCoreRoot` is forwarded to `HAKONIWA_INSTALL_PREFIX`.
- `HakoniwaPduEndpointRoot` is forwarded to `HAKONIWA_PDU_ENDPOINT_PREFIX`.
- `ExtraPrefixPaths` is used to resolve Windows packages such as `glfw3` and transitive dependencies required by `hakoniwa_pdu_endpoint`.
- `HakoniwaPduEndpointRoot` must point to an installed prefix created by `cmake --install`, not a build directory such as `build-win` or `build-shared`. The `hakoniwa_pdu_endpointConfig.cmake` generated in the build tree is not directly consumable by this project.
- If you build `hakoniwa-pdu-endpoint` yourself, install it first:

```powershell
cmake --install C:\project\hakoniwa-pdu-endpoint\build-win --config Release --prefix C:\project\hakoniwa-pdu-endpoint\install
```

- Successful outputs are generated under `build-win/Release/mujoco-common.lib`.
- Successful outputs are generated under `build-win/sensors/Release/msensors.lib`.
- Successful outputs are generated under `build-win/main_for_sample/forklift/Release/forklift_sim.exe`.
- Successful outputs are generated under `build-win/main_for_sample/forklift/Release/forklift_unit_sim.exe`.
- Successful outputs are generated under `build-win/main_for_sample/tb3/Release/tb3_sim.exe`.
- On Windows, runtime DLLs such as `hakoniwa_pdu_endpoint.dll` are copied next to each `.exe` after build.
- If Windows reports `forklift_simulation_loop.obj: Permission denied`, that is a file lock issue, not a source code issue. Close `MSBuild.exe`, `cl.exe`, or `devenv.exe` and rebuild.
- The Python endpoint binding is built separately from the C++ samples. For `python/tb3_gamepad.py`, build the vendored `thirdparty/hakoniwa-pdu-endpoint` Python runtime with Hakoniwa core support enabled:

```powershell
cd .\thirdparty\hakoniwa-pdu-endpoint
.\build-python-win.ps1 `
  -Clean `
  -BuildNative `
  -BuildFfi `
  -EnableHakoniwaCore `
  -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install `
  -BuildDirName build-win `
  -Configuration Release `
  -PythonCommand python `
  -ToolchainFile C:\project\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -VcpkgTriplet x64-windows `
  -Platform x64
```

- `python/tb3_gamepad.py` prefers the vendored Python package under `thirdparty/hakoniwa-pdu-endpoint/python`, so it will use the locally built runtime instead of a stale site-packages installation when both are present.

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

### Standalone examples

- Ultrasonic sensor example:
```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

- Color camera sensor example (`i/k/j/l` moves the camera, `s` writes a PNG from the viewer or terminal):
```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```

- Joint actuator example (`a/d` changes position target, `j/l` changes velocity target in the MuJoCo viewer):
```bash
./src/cmake-build/examples/actuators/joint/joint-actuator-example
```

See [examples/README.md](examples/README.md) for the current example index.

---

## TurtleBot3 2D LiDAR

The TurtleBot3 Burger sample includes a MuJoCo-based 2D LiDAR implementation.

Note:
- `models/tb3/turtlebot3_burger_world.xml` currently uses primitive geoms for the body, wheels, and LiDAR housing instead of external mesh files.
- This avoids runtime failures caused by machine-specific absolute mesh paths when running on Windows.
- The wheel actuators are MuJoCo `<velocity>` actuators. Gamepad input is converted to left / right wheel angular velocity targets before being written through `JointActuatorImpl`.
- Linear velocity and yaw-rate targets are rate-limited before conversion to wheel angular velocities. Defaults include `HAKO_TB3_MAX_YAW_RATE=1.2`, `HAKO_TB3_MAX_LINEAR_ACCELERATION=0.1`, `HAKO_TB3_MAX_YAW_ACCELERATION=0.5`, and `HAKO_TB3_COMMAND_DEADZONE=0.1`.
- The rear caster uses low friction so it does not dominate the drive-wheel behavior.

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

## Camera Depth Status

Camera / depth / RGBD sensor components are available under `include/sensors/camera/` and `src/sensors/camera/`.

- Depth conversion currently assumes the MuJoCo offscreen path where `mjr_readPixels` returns an OpenGL-style depth buffer.
- The implementation converts depth with effective clip planes computed from `vis.map.znear/zfar * stat.extent`.
- Camera / depth / RGBD / multicamera profiles under `config/sensors/camera/*.json` can be loaded into C++ configs.
- The profile structure is documented in `config/sensors/schema/`, and the JSON loader reuses the existing `LoadConfig(config)` validation path.
- Camera unit tests live under `tests/sensors/camera/unit/` and cover config loader, PDU converter, and local depth encoding.
- These unit tests do not require an OpenGL render context and are intended to run in CI.
- This path has been smoke-tested with fixed-camera box scenes at `0.2`, `0.5`, `1.0`, `2.0`, `5.0`, and `9.0` meters.
- The smoke test also checks several image positions, multiple horizontal FOV settings, and clip-range NaN masking.
- Render smoke tests live under `tests/sensors/camera/smoke/` and require MuJoCo + OpenGL, so they are intended for local/manual runs or dedicated CI.
- It is still not claimed to be fully validated for arbitrary scenes such as oblique geometry, extreme camera setups, or alternative depth-map conventions.

## Sensor Components And Examples

Reusable sensor components live under `src/sensors/`, with JSON profiles under `config/sensors/` and schemas under `config/sensors/schema/`.

Current sensor areas:
- camera / depth / RGBD / multicamera
- color camera PNG example
- 2D LiDAR
- ultrasonic range
- IMU
- joint state
- odometry
- TF
- noise helpers
- debug ray visualization

The standalone examples are intentionally smaller than the TurtleBot3 and forklift demos. They are useful for checking one feature at a time:
- [examples/README.md](examples/README.md)
- [examples/sensors/README.md](examples/sensors/README.md)
- [examples/sensors/ultrasonic/README.md](examples/sensors/ultrasonic/README.md)
- [examples/sensors/color_camera/README.md](examples/sensors/color_camera/README.md)
- [examples/actuators/README.md](examples/actuators/README.md)
- [examples/actuators/joint/README.md](examples/actuators/joint/README.md)

Sensor unit tests are optional build targets:
```bash
cmake -S src -B src/cmake-build -DHAKO_BUILD_SENSOR_TESTS=ON
cmake --build src/cmake-build --target run_sensor_unit_tests
```

Camera render smoke tests require a MuJoCo / OpenGL runtime:
```bash
cmake -S src -B src/cmake-build -DHAKO_BUILD_CAMERA_SMOKE_TESTS=ON
cmake --build src/cmake-build --target camera_smoke_tests
```

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

## FAQ

### Q1. What should I run first?

Run the environment doctor first:

```bash
./doctor.bash
```

Fix any `FAIL` lines, then run:

```bash
./build.bash
```

### Q2. `hakoniwa-pdu` is installed, but Python still fails. Why?

`pip` and `python` may point to different Python installations. Use the same Python command that runs the script:

```bash
python -m pip show hakoniwa-pdu
python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

`./doctor.bash` prints the Python executable it checks.

If you want to check a specific Python environment, pass `PYTHON_CMD`:

```bash
PYTHON_CMD=/path/to/python ./doctor.bash
/path/to/python -m pip install --upgrade "hakoniwa-pdu>=1.6.1"
```

### Q3. CMake cannot find `hakoniwa-core-pro` or `hakoniwa-pdu-endpoint`.

Install both packages first, then re-run `./doctor.bash`.

If they are not installed under `/usr/local/hakoniwa`, set:

```bash
export HAKONIWA_CORE_ROOT=/path/to/hakoniwa-core-pro/install
export HAKONIWA_PDU_ENDPOINT_ROOT=/path/to/hakoniwa-pdu-endpoint/install
```

### Q4. CMake cannot find `glfw3`.

Install GLFW:

```bash
brew install glfw
```

Ubuntu:

```bash
sudo apt-get install -y libglfw3-dev
```

### Q5. Where does the color-camera example write PNG files?

By default, it writes `./camera_color_sample.png` relative to the directory where you run the command. You can pass an explicit output path as the third argument.

### Q6. Do I need the MuJoCo viewer for every example?

No. The main robot demos use the viewer by default, and the standalone sensor / actuator examples are designed for interactive viewer use. Some tests and config checks do not need a viewer.

For Docker on macOS, use headless mode:

```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

## Advanced Topics

Advanced forklift context save/restore and RD-light notes are split out of the top-level README.

- [Forklift Context Save/Restore And RD-light](docs/forklift-context-rd.md)
- [RD design notes](rd-design.md)

The top-level README keeps only the entry points, setup, and sample commands. Use the advanced documents when you need restore evidence, RD-light handoff details, or continuity evaluation workflow.

## Samples

- `src/main_for_sample/forklift/main.cpp`: forklift basic integration
- `src/main_for_sample/forklift/main_unit.cpp`: unit model verification
- `src/main_for_sample/tb3/main.cpp`: TurtleBot3 sample (Hakoniwa asset / endpoint / 2D LiDAR)
- `python/tb3_gamepad.py`: TurtleBot3 Python controller asset (PS4/DualSense)
- `python/lidar_visualizer.py`: generic LiDAR visualizer (world view)
- `examples/README.md`: standalone example index
- `examples/sensors/ultrasonic/README.md`: ultrasonic sensor example
- `examples/sensors/color_camera/README.md`: color camera PNG example
- `examples/actuators/joint/README.md`: MJCF-native position / velocity joint actuator example
- `src/sensors/`: reusable sensor components and PDU conversion helpers
- `config/sensors/lidar/lds-01.json`: TurtleBot3 LDS-01-like noisy LiDAR profile
- `config/sensors/lidar/lds-02.json`: TurtleBot3 LDS-02-like longer-range LiDAR profile
- `config/sensors/lidar/urg-04lx-ug01.json`: Hokuyo URG-04LX-UG01-like cleaner LiDAR profile
- `config/sensors/ultrasonic/lego-spike-distance-sensor.json`: ultrasonic range sensor profile used by the standalone example
- `config/sensors/color_camera/simple-color-camera.json`: color camera profile used by the PNG example

---

## Contributing

- Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening an issue or pull request.
- Coding agents should also read [AGENTS.md](AGENTS.md).
- For setup failures, include `./doctor.bash` output in the issue.
- For viewer examples, mention whether the behavior was manually checked.

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
