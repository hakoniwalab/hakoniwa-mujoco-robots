# Hunter V2 Model Forge validation

This Recipe is the external-source validation case for the MBody Ackermann
Forge. It deliberately stops at reproducible body generation; runtime PDU,
PS5 control, and vehicle tuning are separate acceptance stages.

```bash
cd ../hakoniwa-business-pack
python tools/recipe.py configure \
  --recipe ../hakoniwa-mujoco-robots/recipes/hunter/hunter-model-forge.yaml

cd ../hakoniwa-mujoco-robots
python3.12 tools/recipe/hunter.py forge
python3.12 tools/recipe/hunter.py verify-forge
python3.12 tools/recipe/hunter.py validate
python3.12 tools/recipe/hunter.py optimize
python3.12 tools/recipe/hunter.py doctor
```

The same compiled `generic-ackermann-hakoniwa-asset` also runs Hunter by
selecting the Hunter Asset Manifest and Ackermann runtime contract:

```bash
python3.12 tools/recipe/hunter.py configure
python3.12 tools/recipe/hunter.py build
python3.12 tools/recipe/hunter.py smoke
python3.12 tools/recipe/hunter.py start
python3.12 tools/recipe/hunter.py stop
```

No Hunter-specific C++ asset exists. The MJCF, four component configs, and
`ackermann-runtime.json` are the replaceable Asset Package.

The implementation and source of truth live in the sibling
`hakoniwa-mbody-registry`. The entrypoint remains here so downstream users and
CI do not need to reconstruct MBody tool invocation details.

The Business Pack configure step installs the pinned tool dependencies into
Foundation Python. The Recipe never selects or exposes an MBody-local virtual
environment.
