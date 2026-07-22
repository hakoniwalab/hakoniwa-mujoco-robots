# Shadow Hand Named-Actuator Control

This sample validates the MuJoCo Menagerie Shadow Hand model and exposes a
minimal Hakoniwa PDU control path for bounded open-close finger motion.

The goal is to define the command and observation spaces correctly before
writing a plant asset:

- command space: MuJoCo actuator names
- observation space: MuJoCo joint names
- tendon coupling: kept inside the MuJoCo model

Do not assume that the command dimension equals the observed joint count.

## Build

```bash
cmake -S src -B src/cmake-build
cmake --build src/cmake-build --target shadow-hand-actuator-map
cmake --build src/cmake-build --target shadow-hand-hakoniwa-asset
```

## Inspect

```bash
./src/cmake-build/examples/actuators/shadow_hand/shadow-hand-actuator-map
```

Expected signals:

- the model-local Apache-2.0 `LICENSE` is present
- `thirdparty/mujoco_menagerie/shadow_hand/scene_right.xml` loads
- the model summary includes `nu=20`, `njnt=25`, and `ntendon=4`
- the actuator command table lists 20 named position actuators
- four actuators target fixed tendons:
  - `rh_A_FFJ0 -> rh_FFJ0`
  - `rh_A_MFJ0 -> rh_MFJ0`
  - `rh_A_RFJ0 -> rh_RFJ0`
  - `rh_A_LFJ0 -> rh_LFJ0`
- each tendon couples two distal finger joints with coefficient 1

The scene also contains an unnamed freejoint for the free object. Exclude that
unnamed joint from the Shadow Hand `JointState` contract.

## MuJoCo-Only Check

```bash
./src/cmake-build/examples/actuators/shadow_hand/shadow-hand-actuator-map --check
```

The check applies a conservative open target, then a bounded close target, and
verifies that `qpos` / `qvel` stay finite.

Expected representative output:

```text
open  time=0.000 FFJ2=0.000 FFJ1=0.000 MFJ2=0.000 MFJ1=0.000 THJ2=0.000 THJ1=0.000
close time=1.000 FFJ2=0.857 FFJ1=0.865 MFJ2=0.857 MFJ1=0.865 THJ2=0.386 THJ1=0.860
check ok
```

## Command Contract Draft

The first Hakoniwa command PDU should use actuator ordering, not joint ordering:

```text
rh_A_WRJ2
rh_A_WRJ1
rh_A_THJ5
rh_A_THJ4
rh_A_THJ3
rh_A_THJ2
rh_A_THJ1
rh_A_FFJ4
rh_A_FFJ3
rh_A_FFJ0
rh_A_MFJ4
rh_A_MFJ3
rh_A_MFJ0
rh_A_RFJ4
rh_A_RFJ3
rh_A_RFJ0
rh_A_LFJ5
rh_A_LFJ4
rh_A_LFJ3
rh_A_LFJ0
```

The four `J0` commands are tendon actuators. The plant should set:

```cpp
data->ctrl[actuator_id] = target;
```

MuJoCo then applies the tendon coupling internally. Hakoniwa should not create
independent command fields for `rh_FFJ1`, `rh_FFJ2`, and equivalent distal
joint pairs.

## NamedActuatorImpl

`NamedActuatorImpl` is a common actuator helper for models whose control
contract is an actuator name rather than a joint name. It supports both direct
joint transmissions and tendon transmissions by binding:

```text
actuator_name -> actuator_id -> transmission target
```

This keeps the existing `JointActuatorImpl` behavior intact while giving
Shadow Hand and future tendon-driven models a cleaner command boundary.

## Hakoniwa PDU Smoke

The Hakoniwa command PDU is `std_msgs/Float64MultiArray` with 20 elements in
the actuator order shown above. The state PDU is `sensor_msgs/JointState` with
24 named hand joints. The unnamed freejoint for the scene object is intentionally
excluded.

Terminal 1:

```bash
./src/cmake-build/examples/actuators/shadow_hand/shadow-hand-hakoniwa-asset --no-viewer
```

Wait until the plant asset is registered and reaches:

```text
WAIT START
```

Terminal 2:

```bash
/Users/tmori/.pyenv/shims/python3.12 \
  examples/actuators/shadow_hand/send_shadow_hand_targets.py \
  --duration-sec 5
```

Wait until the sender is registered and reaches:

```text
WAIT START
```

Terminal 3:

```bash
/usr/local/hakoniwa/bin/hako-cmd start
```

Expected representative plant output:

```text
time=  1.002 FFJ0_target=0.738 FFJ2=0.295 FFJ1=0.347 THJ1_target=0.359 THJ1=0.316
time=  2.002 FFJ0_target=1.739 FFJ2=0.804 FFJ1=0.901 THJ1_target=0.845 THJ1=0.828
```

Expected representative sender output:

```text
sent actuator_position_targets FFJ0=0.875 MFJ0=0.875 THJ1=0.425
sent actuator_position_targets FFJ0=1.750 MFJ0=1.750 THJ1=0.850 | joint_states rh_THJ1=0.837 rh_FFJ2=0.802 rh_FFJ1=0.922 rh_MFJ2=0.802 rh_MFJ1=0.922
```

This confirms that a single tendon actuator command such as `rh_A_FFJ0`
changes the coupled observed joints `rh_FFJ2` and `rh_FFJ1`. It does not claim
stable grasping or closed-loop hand control.

If the Codex sandbox cannot access Hakoniwa shared memory, run the plant and
sender from a normal terminal. The runtime ordering remains: plant asset
registers, sender asset registers, then `hako-cmd start`.

## License

The Shadow Hand MJCF and mesh assets are vendored from MuJoCo Menagerie at
`thirdparty/mujoco_menagerie/shadow_hand`.

Keep the model-local `thirdparty/mujoco_menagerie/shadow_hand/LICENSE` file
with redistributed model artifacts. The local README states that the original
URDF and assets were provided by Shadow Robot Company under Apache-2.0.
