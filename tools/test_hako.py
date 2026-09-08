from __future__ import annotations

import importlib.util
import os
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

    def test_absolute_build_directory_is_rejected(self):
        with self.assertRaisesRegex(HAKO.ConfigError, "repository-relative"):
            HAKO.resolve_config(
                {"version": 1, "build": {"dir": str(REPO_ROOT / "out")}}
            )

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

    def test_config_option_stays_with_hako_and_native_args_follow_separator(self):
        args, native_args = HAKO.parse_cli(
            ["build", "--config", "custom.yaml", "--", "-G", "Ninja"]
        )
        self.assertEqual(args.command, "build")
        self.assertEqual(args.config, "custom.yaml")
        self.assertEqual(native_args, ["-G", "Ninja"])


class NativeMappingTests(unittest.TestCase):
    def test_cli_build_directory_overrides_manifest_on_both_platforms(self):
        with tempfile.TemporaryDirectory() as tmp:
            expected = str(Path(tmp).resolve() / "external")
            previous = Path.cwd()
            try:
                os.chdir(tmp)
                args, native = HAKO.parse_cli(["build", "--build-dir", "external", "--", "-G", "Ninja"])
                for platform in ("linux", "win32"):
                    with self.subTest(platform=platform), patch.object(HAKO.sys, "platform", platform), patch.object(HAKO, "_powershell", return_value="pwsh"):
                        command, env = HAKO.resolve_command(args.command, native, build_dir="out/legacy", explicit_build_dir=args.build_dir)
                        if platform == "win32":
                            self.assertEqual(command[command.index("-BuildDirName") + 1], expected)
                        else:
                            self.assertEqual(env["HAKO_BUILD_DIR"], expected)
                        self.assertEqual(command[-2:], ["-G", "Ninja"])
            finally:
                os.chdir(previous)

    def test_windows_preflight_accepts_an_initially_empty_issue_collection(self):
        script = (REPO_ROOT / "build-win.ps1").read_text(encoding="utf-8-sig")
        function_start = script.index("function Add-Issue")
        function_end = script.index("function Test-CMakeConfig", function_start)
        add_issue = script[function_start:function_end]
        self.assertIn("[AllowEmptyCollection()]", add_issue)

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
                "out/mujoco",
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


class RecipeStateTests(unittest.TestCase):
    def load_recipe(self, name):
        spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "tools" / "recipe" / f"{name}.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_validation_and_optimization_use_selected_state_and_keep_default(self):
        for name in ("generic_ackermann", "hunter"):
            module = self.load_recipe(name)
            with self.subTest(recipe=name), tempfile.TemporaryDirectory() as tmp, patch.object(module, "required", side_effect=lambda path, label: path):
                root = Path(tmp)
                with patch.object(module, "repo_root", return_value=root), patch.object(module.subprocess, "run") as run:
                    run.return_value.returncode = 0
                    for state in (None, root / "host-state", root / "docker-state"):
                        module.selected_state_dir = state
                        selected = state or root / ".hako"
                        module.validate_model()
                        command = run.call_args.args[0]
                        report = Path(command[command.index("--report") + 1])
                        self.assertTrue(report.is_relative_to(selected))
                        module.optimize_model(2)
                        command = run.call_args.args[0]
                        output = Path(command[command.index("--output") + 1])
                        self.assertTrue(output.is_relative_to(selected))
                        self.assertEqual(command[-2:], ["--trials", "2"])

    def test_generic_cli_resolves_relative_state_and_resets_default(self):
        module = self.load_recipe("generic_ackermann")
        with patch.object(module, "validate_model", return_value=0):
            self.assertEqual(module.main(["validate", "--state-dir", "external-state"]), 0)
            self.assertEqual(module.state_dir(), (Path.cwd() / "external-state").resolve())
            module.main(["validate"])
            self.assertEqual(module.state_dir(), REPO_ROOT / ".hako")

    def test_hunter_cli_propagates_state_to_generic_runtime(self):
        module = self.load_recipe("hunter")
        with patch.object(module, "required", side_effect=lambda path, label: path), patch.object(module.subprocess, "run") as run, patch.object(module.sys, "argv", ["hunter.py", "build", "--state-dir", "external-state"]):
            run.return_value.returncode = 0
            self.assertEqual(module.main(), 0)
            command = run.call_args.args[0]
            self.assertEqual(command[command.index("--state-dir") + 1], str((Path.cwd() / "external-state").resolve()))


if __name__ == "__main__":
    unittest.main()
