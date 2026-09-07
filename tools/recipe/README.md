# Recipe tool Foundation paths

The generic Ackermann tool (golf-cart and Hunter profiles) treats
`HAKONIWA_HOME` as the Foundation **install prefix**:

- Python: `$HAKONIWA_HOME/python/bin/python3`
- Libraries: `$HAKONIWA_HOME/lib`
- Core configuration: `$HAKONIWA_HOME/../config/cpp_core_config.json`

Without that variable, the existing sibling
`hakoniwa-business-pack/work/foundation/install` default is used.
This fix does not relocate Recipe outputs or native build directories.

Run the path regression checks without simulator dependencies:

```bash
python3 -m unittest discover -s tools/recipe -p test_foundation_prefix.py -v
```
