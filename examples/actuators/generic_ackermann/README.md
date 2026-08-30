# Generic Ackermann Hakoniwa Asset

This is one manifest-driven Hakoniwa/MuJoCo Ackermann asset controlled with a
PS5 DualSense controller. The same compiled C++ binary runs both the original
generic Golf Cart and Hunter V2; vehicle geometry, names, and control limits
come from the selected Ackermann Asset Package.

The joint structure and baseline geometry are informed by
[`srmainwaring/steer_bot`](https://github.com/srmainwaring/steer_bot),
commit `c86232658df84a8d3518b480a468e8a260754c9c` (BSD-3-Clause): two front
steering joints, two front rolling joints, and two driven rear wheel joints.
The upstream model is a simple box-and-cylinder teaching model, not the golf
cart mesh shown in the reference image.

## Control path

```text
PS5 DualSense
  -> hako_msgs/GameControllerOperation
  -> deadzone and speed/steering scaling
  -> Ackermann inner/outer front angles
  -> rear left/right wheel velocities
  -> MuJoCo joint actuators
```

The left stick controls steering. The right stick vertical axis controls
forward/reverse velocity. This intentionally reuses the established Hakoniwa
gamepad PDU so it works before `ackermann_msgs/AckermannDrive` is distributed in
all installed SDK packages.

## Build and headless smoke test

First regenerate the body from the sibling MBody registry. This is the same
Forge/runtime ownership pattern used by `hakoniwa-robot-arm-pack`:

```bash
python3.12 tools/recipe/generic_ackermann.py forge
python3.12 tools/recipe/generic_ackermann.py verify-forge
```

The physical source of truth is
`../hakoniwa-mbody-registry/bodies/generic_ackermann_golf_cart/config/`.
`recipes/generic_ackermann/asset-manifest.json` binds its generated model to
this runtime's PDU and actuator components.

```bash
python3.12 tools/recipe/generic_ackermann.py build
python3.12 tools/recipe/generic_ackermann.py smoke
```

The smoke test resolves the asset manifest, loads the generated model and four
runtime actuator bindings, and verifies idle stability, straight travel, and a
sustained Ackermann turn.

Run the exact same binary against Hunter:

```bash
python3.12 tools/recipe/hunter.py build
python3.12 tools/recipe/hunter.py smoke
```

See [`docs/ackermann-asset-package.md`](../../../docs/ackermann-asset-package.md)
for the schema and migration procedure.

Inspect the model without starting Hakoniwa:

```bash
./src/cmake-build/examples/actuators/generic_ackermann/generic-ackermann-hakoniwa-asset \
  --manifest recipes/generic_ackermann/asset-manifest.json \
  --view-model
```

## PS5 operation with Hakoniwa Launcher

Confirm that pygame can see the controller without connecting to Hakoniwa:

```bash
../hakoniwa-business-pack/work/foundation/install/python/bin/python3 \
  python/ackermann_gamepad.py --check-controller
```

Configure and build the tracked Recipe, then let Hakoniwa Launcher own process
ordering, PDU initialization, simulation start, and shutdown:

```bash
python3.12 tools/recipe/generic_ackermann.py forge
python3.12 tools/recipe/generic_ackermann.py verify-forge
python3.12 tools/recipe/generic_ackermann.py configure
python3.12 tools/recipe/generic_ackermann.py build
python3.12 tools/recipe/generic_ackermann.py doctor
python3.12 tools/recipe/generic_ackermann.py start
```

The plant starts before simulation time. The PS5 sender starts after the
Launcher has created the PDU region, avoiding startup races and stale PDU-size
reuse. Stop the complete session with:

```bash
python3.12 tools/recipe/generic_ackermann.py stop
```

Generated Launcher state and logs are under
`../hakoniwa-business-pack/work/recipes/generic-ackermann-ps5/`.

## Geometry and approximation

- wheelbase: 1.55 m
- track width: 1.04 m
- wheel radius: 0.25 m
- maximum center steering angle: 0.70 rad
- maximum inner-wheel joint angle: 0.90 rad
- maximum commanded speed: 3.5 m/s
- maximum commanded acceleration: 6.0 m/s²

The controller applies ideal planar Ackermann geometry. It does not model a
steering rack, tire slip curve, suspension, motor torque curve, or powertrain.
Those are explicit future fidelity layers, not hidden defaults.
