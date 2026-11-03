from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / "edge-path-project" / "hil" / "deployment"


class NativeDeploymentScriptTests(unittest.TestCase):
    def test_shell_scripts_are_syntactically_valid(self):
        for name in ("start_hil.sh", "stop_hil.sh", "status_hil.sh", "deploy_r2100_hil.sh"):
            with self.subTest(name=name):
                result = subprocess.run(["bash", "-n", str(DEPLOY / name)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_runtime_scripts_no_longer_reference_zipapp(self):
        for name in ("start_hil.sh", "deploy_r2100_hil.sh", "r2100_hil_main.py"):
            text = (DEPLOY / name).read_text()
            self.assertNotIn("r2100_hil.pyz", text)

    def test_deploy_requires_prebuilt_arm64_standalone_package(self):
        text = (DEPLOY / "deploy_r2100_hil.sh").read_text()
        self.assertIn("standalone-linux-executable", text)
        self.assertIn("aarch64|arm64", text)
        self.assertIn("python_requirement", text)
        self.assertNotIn("build_r2100_hil.py\"", text)

    def test_entrypoint_help_uses_native_executable_name(self):
        result = subprocess.run(
            [sys.executable, str(DEPLOY / "r2100_hil_main.py"), "--help"],
            cwd=DEPLOY,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("standalone deployment executable", result.stdout)
        self.assertIn("r2100_hil preflight", result.stdout)
        self.assertNotIn(".pyz", result.stdout)


if __name__ == "__main__":
    unittest.main()
