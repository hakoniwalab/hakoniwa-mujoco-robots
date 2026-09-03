import copy
import json
import unittest
from pathlib import Path

try:
    from jsonschema import Draft202012Validator
except ImportError:
    Draft202012Validator = None


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "config/assets/schema/hakoniwa-asset-manifest.schema.json"


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


@unittest.skipUnless(Draft202012Validator, "install jsonschema to validate schemas")
class AssetManifestSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.schema = load_json(SCHEMA_PATH)
        Draft202012Validator.check_schema(cls.schema)
        cls.validator = Draft202012Validator(cls.schema)
        cls.base_manifest = load_json(ROOT / "recipes/generic_ackermann/asset-manifest.json")

    def test_existing_manifests_remain_valid_without_schema_version(self):
        paths = [
            *sorted((ROOT / "config/assets").glob("*-asset.json")),
            *sorted((ROOT / "recipes").glob("*/asset-manifest.json")),
        ]
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(path=path):
                self.assertNotIn("schema_version", load_json(path))
                self.validator.validate(load_json(path))

    def test_controller_and_optional_v1_are_valid(self):
        manifest = copy.deepcopy(self.base_manifest)
        manifest["schema_version"] = 1
        manifest["components"].append(
            {
                "id": "trajectory_controller",
                "kind": "controller",
                "type": "joint_trajectory_controller",
                "config": "controller.json",
            }
        )
        self.validator.validate(manifest)

    def test_unknown_kind_is_rejected(self):
        manifest = copy.deepcopy(self.base_manifest)
        manifest["components"][0]["kind"] = "unknown"
        self.assertTrue(list(self.validator.iter_errors(manifest)))

    def test_unknown_schema_version_is_rejected(self):
        manifest = copy.deepcopy(self.base_manifest)
        manifest["schema_version"] = 2
        self.assertTrue(list(self.validator.iter_errors(manifest)))


if __name__ == "__main__":
    unittest.main()
