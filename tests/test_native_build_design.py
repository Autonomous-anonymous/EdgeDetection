import importlib.util
from pathlib import Path
import os
import stat
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
BUILD_PATH = ROOT / "edge-path-project" / "hil" / "deployment" / "build_r2100_hil.py"

spec = importlib.util.spec_from_file_location("build_r2100_hil", BUILD_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class NativeBuildDesignTests(unittest.TestCase):
    def test_pyinstaller_command_is_onefile_and_includes_dynamic_components(self):
        command = mod.pyinstaller_command(Path("dist"), Path("work"), Path("spec"))
        self.assertIn("--onefile", command)
        self.assertIn("--clean", command)
        self.assertIn("--noconfirm", command)
        for component in mod.HIDDEN_IMPORTS:
            index = command.index(component)
            self.assertEqual(command[index - 1], "--hidden-import")

    def test_architecture_normalization(self):
        self.assertEqual(mod.normalized_architecture("arm64"), "aarch64")
        self.assertEqual(mod.normalized_architecture("aarch64"), "aarch64")
        self.assertEqual(mod.normalized_architecture("AMD64"), "x86_64")

    def test_verify_executable_rejects_non_elf(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fake"
            path.write_bytes(b"#!/bin/sh\n")
            path.chmod(0o755)
            with self.assertRaisesRegex(RuntimeError, "expected Linux ELF"):
                mod.verify_executable(path)

    def test_verify_executable_accepts_elf_magic_and_sets_executable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fake"
            path.write_bytes(b"\x7fELF" + b"\x00" * 16)
            path.chmod(0o644)
            mod.verify_executable(path)
            self.assertTrue(path.stat().st_mode & stat.S_IXUSR)

    def test_manifest_declares_no_python_runtime_requirement(self):
        with tempfile.TemporaryDirectory() as directory:
            original_output = mod.OUTPUT_DIR
            original_interface = mod.INTERFACE_PATH
            try:
                temp = Path(directory)
                mod.OUTPUT_DIR = temp
                interface = temp / "interface.json"
                interface.write_text("{}")
                mod.INTERFACE_PATH = interface
                binary = temp / "r2100_hil"
                binary.write_bytes(b"\x7fELF")
                with mock.patch.object(mod, "normalized_architecture", return_value="aarch64"):
                    manifest = mod.build_manifest([binary], "6.test")
                self.assertEqual(manifest["format"], "standalone-linux-executable")
                self.assertEqual(manifest["entrypoint"], "r2100_hil")
                self.assertEqual(manifest["architecture"], "aarch64")
                self.assertIsNone(manifest["python_requirement"])
            finally:
                mod.OUTPUT_DIR = original_output
                mod.INTERFACE_PATH = original_interface


if __name__ == "__main__":
    unittest.main()
