from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


MODULE_PATH = Path(__file__).with_name("hako.py")
SPEC = importlib.util.spec_from_file_location("hako_build_tool", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
HAKO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HAKO)
REPO_ROOT = MODULE_PATH.resolve().parents[1]


class ManifestTests(unittest.TestCase):
    def test_default_manifest_preserves_native_build_directory(self):
        cfg = HAKO.resolve_config(
            HAKO.load_simple_yaml(REPO_ROOT / "hakoniwa-build.yaml")
        )
        self.assertEqual(cfg, {"version": 1, "build": {"dir": "auto"}})
        self.assertIsNone(HAKO._resolved_build_dir(cfg["build"]["dir"], REPO_ROOT))

    def test_explicit_build_directory_is_repo_relative(self):
        resolved = HAKO._resolved_build_dir("out/mujoco", REPO_ROOT)
        self.assertEqual(resolved, (REPO_ROOT / "out" / "mujoco").resolve())

    def test_unknown_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "build.yaml"
            path.write_text(
                "version: 1\nbuild:\n  dir: auto\n  type: Release\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(HAKO.ConfigError, "unknown key"):
                HAKO.resolve_config(HAKO.load_simple_yaml(path))

    def test_default_manifest_is_repo_relative_and_explicit_config_is_cwd_relative(self):
        self.assertEqual(
            HAKO.resolve_manifest_path(None, REPO_ROOT),
            REPO_ROOT / "hakoniwa-build.yaml",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            work = Path(temp_dir)
            custom = work / "custom.yaml"
            custom.write_text("version: 1\nbuild:\n  dir: auto\n", encoding="utf-8")
            with patch("pathlib.Path.cwd", return_value=work):
                self.assertEqual(
                    HAKO.resolve_manifest_path("custom.yaml", REPO_ROOT),
                    custom.resolve(),
                )


class NativeMappingTests(unittest.TestCase):
    def test_posix_auto_keeps_existing_native_default(self):
        with patch.object(HAKO.sys, "platform", "linux"):
            command, env = HAKO.resolve_command("build", [], build_dir="auto")
        self.assertEqual(command, ["bash", str(REPO_ROOT / "build.bash")])
        self.assertNotIn("HAKO_BUILD_DIR", env)

    def test_posix_explicit_build_dir_maps_to_environment(self):
        expected = (REPO_ROOT / "out" / "mujoco").resolve()
        with patch.object(HAKO.sys, "platform", "linux"):
            command, env = HAKO.resolve_command("build", [], build_dir="out/mujoco")
        self.assertEqual(command, ["bash", str(REPO_ROOT / "build.bash")])
        self.assertEqual(env["HAKO_BUILD_DIR"], str(expected))

    def test_windows_explicit_build_dir_maps_to_native_argument(self):
        expected = (REPO_ROOT / "out" / "mujoco").resolve()
        with patch.object(HAKO.sys, "platform", "win32"), patch.object(
            HAKO, "_powershell", return_value="pwsh"
        ):
            command, _env = HAKO.resolve_command("build", [], build_dir="out/mujoco")
        self.assertEqual(
            command,
            [
                "pwsh",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(REPO_ROOT / "build-win.ps1"),
                "-BuildDirName",
                str(expected),
            ],
        )

    def test_doctor_keeps_existing_native_entry_points(self):
        with patch.object(HAKO.sys, "platform", "linux"):
            command, _env = HAKO.resolve_command("doctor", [], build_dir="auto")
        self.assertEqual(command, ["bash", str(REPO_ROOT / "doctor.bash")])

        with patch.object(HAKO.sys, "platform", "win32"), patch.object(
            HAKO, "_powershell", return_value="pwsh"
        ):
            command, _env = HAKO.resolve_command("doctor", [], build_dir="auto")
        self.assertEqual(
            command,
            [
                "pwsh",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(REPO_ROOT / "build-win.ps1"),
                "-DoctorOnly",
            ],
        )


if __name__ == "__main__":
    unittest.main()
