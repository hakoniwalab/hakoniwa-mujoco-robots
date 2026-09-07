"""Ackermann Foundation paths must agree with the Workspace install prefix."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import generic_ackermann as recipe


class FoundationPrefixTest(unittest.TestCase):
    def check_paths(self, install):
        self.assertEqual(recipe.install_root(), install)
        self.assertEqual(recipe.foundation_python(), install / "python/bin/python3")
        self.assertEqual(recipe.foundation_root() / "config/cpp_core_config.json",
                         install.parent / "config/cpp_core_config.json")

    def test_default_without_environment(self):
        with patch.dict(os.environ, {}, clear=True):
            self.check_paths(recipe.business_pack_root() / "work/foundation/install")

    def test_active_default_and_external_workspaces(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            for install in (root / "bp/work/foundation/install", root / "external/foundation/install"):
                with self.subTest(install=install), patch.dict(os.environ, {
                    "HAKONIWA_HOME": str(install)}, clear=True):
                    self.check_paths(install)

    def test_symlink_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            install = root / "foundation/install"
            install.mkdir(parents=True)
            alias = root / "prefix"
            alias.symlink_to(install, target_is_directory=True)
            with patch.dict(os.environ, {"HAKONIWA_HOME": str(alias)}, clear=True):
                self.check_paths(install)


if __name__ == "__main__":
    unittest.main()
