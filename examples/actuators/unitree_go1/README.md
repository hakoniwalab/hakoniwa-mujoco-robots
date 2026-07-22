# Unitree Go1 Joint I/O Examples

These samples load the vendored MuJoCo Menagerie Unitree Go1 `scene.xml`, bind
the 12 MJCF position actuators by name, apply the Menagerie `home` keyframe ctrl
values, and optionally apply a small bounded perturbation to two diagonal thigh
joints.

The samples verify joint I/O only. They do not claim walking behavior.

## Build

```bash
cmake --build src/cmake-build --target unitree-go1-joint-io-example
cmake --build src/cmake-build --target unitree-go1-joint-hakoniwa-asset
python3 -m py_compile examples/actuators/unitree_go1/send_go1_joint_targets.py
python3 -m py_compile examples/actuators/unitree_go1/pose_bounce_go1.py
python3 -m py_compile examples/actuators/unitree_go1/walk_go1.py
```

## MuJoCo-Only Check

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-io-example --check
```

Expected signals:

- the model loads from `thirdparty/mujoco_menagerie/unitree_go1/scene.xml`
- `nu=12` and the 12 actuator names are printed
- `FR_thigh` and `RL_thigh` move after the small perturbation
- the final line is `check ok`

## MuJoCo-Only Viewer

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-io-example
```

Controls:

- `t`: toggle the small diagonal thigh perturbation
- `h`: return to home ctrl values
- `r`: reset simulation to the Menagerie home keyframe
- `p`: pause or resume physics
- `q` or `Esc`: quit

## Hakoniwa PDU Smoke

Use three terminals. Start the plant asset first, then the Python sender, then
start Hakoniwa time.

Terminal 1:

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-hakoniwa-asset --no-viewer
```

Terminal 2:

```bash
python3 examples/actuators/unitree_go1/send_go1_joint_targets.py --duration-sec 3 --amplitude 0.12
```

Terminal 3:

```bash
/usr/local/hakoniwa/bin/hako-cmd start
```

Expected plant signals:

- `asset=Go1JointAsset command_pdu=joint_position_targets joint_state_pdu=joint_states`
- the 12 actuator command order is printed
- after `hako-cmd start`, the plant logs `WAIT RUNNING` and `Unitree Go1 joint Hakoniwa asset started`
- `FR_thigh_target` and `RL_thigh_target` change, and `FR_thigh` / `RL_thigh` follow

Expected sender signals:

- `Go1 joint sender is registered.`
- after `hako-cmd start`, the sender logs `sent joint_position_targets`
- once the plant publishes state, the sender logs `joint_states FR_thigh:... RL_thigh:...`

The first read may report an invalid initial `joint_states` or
`Float64MultiArray` PDU before the publisher writes its first frame. The smoke
is valid when later command/state logs update and the sender exits normally.

## Pose Bounce Demo

`pose_bounce_go1.py` sends a jump-like posture sequence:
`home -> crouch -> extend -> recover -> home`. This is a pose demo, not
verified jumping or locomotion.

Terminal 1:

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-hakoniwa-asset --no-viewer
```

Terminal 2:

```bash
python3 examples/actuators/unitree_go1/pose_bounce_go1.py --cycles 2
```

Terminal 3:

```bash
/usr/local/hakoniwa/bin/hako-cmd start
```

Expected sender signals:

- `Go1 pose bounce sender is registered.`
- after `hako-cmd start`, phases such as `home`, `crouch`, `extend`, and `recover` are logged
- once the plant publishes state, the sender logs thigh/calf JointState readback

## Walk Pattern Demo

`walk_go1.py` sends experimental open-loop joint patterns through the same
12-joint PDU contract. The default `creep` profile is tuned for slow, stable
backward motion with four support-friendly leg phases; this direction was more
stable for the current Menagerie Go1 rigid-body/contact setup. `--profile trot`
keeps the showier diagonal-leg pattern, but it tips more easily. This is not a
verified walking controller.

Terminal 1:

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-hakoniwa-asset --no-viewer
```

Terminal 2:

```bash
python3 examples/actuators/unitree_go1/walk_go1.py --duration-sec 6
```

For a shorter smoke test:

```bash
python3 examples/actuators/unitree_go1/walk_go1.py --duration-sec 4
```

If the model tips in the viewer, use a smaller/slower pattern:

```bash
python3 examples/actuators/unitree_go1/walk_go1.py --duration-sec 6 --frequency-hz 0.45 --thigh-amp 0.07 --calf-lift 0.08 --hip-sway 0.0
```

To try the opposite sweep direction:

```bash
python3 examples/actuators/unitree_go1/walk_go1.py --duration-sec 6 --forward
```

For the previous diagonal-leg pattern:

```bash
python3 examples/actuators/unitree_go1/walk_go1.py --profile trot --duration-sec 3 --frequency-hz 0.8 --thigh-amp 0.16 --calf-lift 0.24
```

Terminal 3:

```bash
/usr/local/hakoniwa/bin/hako-cmd start
```

Expected sender signals:

- `Go1 walk sender is registered.`
- after `hako-cmd start`, phases such as `warmup`, `creep`, and `cooldown` are logged
- once the plant publishes state, the sender logs thigh JointState readback

Expected plant signals:

- `FR_thigh_target` and `RL_thigh_target` change during the pattern
- the base pose remains finite during the short smoke run; in headless and viewer testing, the default creep profile stayed near the home height and moved slowly backward
- one initial `Float64MultiArray` conversion warning can appear before the first command frame

## PDU Contract

- robot/asset name: `Go1JointAsset`
- command PDU: `joint_position_targets`
- command type: `std_msgs/Float64MultiArray`
- state PDU: `joint_states`
- state type: `sensor_msgs/JointState`
- command order:
  `FR_hip`, `FR_thigh`, `FR_calf`,
  `FL_hip`, `FL_thigh`, `FL_calf`,
  `RR_hip`, `RR_thigh`, `RR_calf`,
  `RL_hip`, `RL_thigh`, `RL_calf`

## Hakoniwa Viewer

The Hakoniwa asset opens the MuJoCo viewer by default:

```bash
./src/cmake-build/examples/actuators/unitree_go1/unitree-go1-joint-hakoniwa-asset
```

Use `--no-viewer` or `HAKO_GO1_JOINT_ENABLE_VIEWER=0` for headless smoke tests.

## License

The Go1 MJCF and mesh assets are vendored from MuJoCo Menagerie at
`thirdparty/mujoco_menagerie/unitree_go1`. Keep the model-local
`thirdparty/mujoco_menagerie/unitree_go1/LICENSE` file with redistributed model
artifacts.
