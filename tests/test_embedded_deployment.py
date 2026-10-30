from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]

HIL_DIR = REPO_ROOT / "edge-path-project" / "hil"
DEPLOYMENT_DIR = HIL_DIR / "deployment"
DIST_DIR = HIL_DIR / "dist" / "r2100_hil"

BUILD_SCRIPT = DEPLOYMENT_DIR / "build_r2100_hil.py"

SHELL_SCRIPTS = [
    DEPLOYMENT_DIR / "start_hil.sh",
    DEPLOYMENT_DIR / "stop_hil.sh",
    DEPLOYMENT_DIR / "status_hil.sh",
    DEPLOYMENT_DIR / "deploy_r2100_hil.sh",
]

EXPECTED_DIST_FILES = {
    "r2100_hil.pyz",
    "manifest.json",
    "start_hil.sh",
    "stop_hil.sh",
    "status_hil.sh",
    "config/ruskotunturi_hil.json",
    "config/ruskotunturi_hil_alt_ids.json",
    "config/ruskotunturi_hil_alt_scaling.json",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)

    return digest.hexdigest()


class EmbeddedDeploymentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(
            [sys.executable, str(BUILD_SCRIPT)],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )

    def test_shell_scripts_have_valid_syntax(self):
        for script in SHELL_SCRIPTS:
            with self.subTest(script=script.name):
                result = subprocess.run(
                    ["bash", "-n", str(script)],
                    cwd=REPO_ROOT,
                    capture_output=True,
                    text=True,
                )

                self.assertEqual(
                    result.returncode,
                    0,
                    msg=result.stderr,
                )

    def test_build_contains_expected_files(self):
        actual = {
            str(path.relative_to(DIST_DIR))
            for path in DIST_DIR.rglob("*")
            if path.is_file()
        }

        self.assertEqual(actual, EXPECTED_DIST_FILES)

    def test_runtime_files_are_executable(self):
        for name in [
            "r2100_hil.pyz",
            "start_hil.sh",
            "stop_hil.sh",
            "status_hil.sh",
        ]:
            with self.subTest(file=name):
                path = DIST_DIR / name
                self.assertTrue(
                    path.stat().st_mode & 0o111,
                    msg=f"{name} is not executable",
                )

    def test_manifest_matches_built_files(self):
        manifest_path = DIST_DIR / "manifest.json"

        manifest = json.loads(manifest_path.read_text())

        self.assertEqual(
            manifest["format"],
            "python-zipapp",
        )
        self.assertEqual(
            manifest["entrypoint"],
            "r2100_hil.pyz",
        )

        expected_manifest_files = EXPECTED_DIST_FILES - {
            "manifest.json"
        }

        self.assertEqual(
            set(manifest["files"]),
            expected_manifest_files,
        )

        for relative, metadata in manifest["files"].items():
            with self.subTest(file=relative):
                path = DIST_DIR / relative

                self.assertEqual(
                    metadata["sha256"],
                    sha256(path),
                )
                self.assertEqual(
                    metadata["bytes"],
                    path.stat().st_size,
                )

    def test_zipapp_check_passes(self):
        result = subprocess.run(
            [str(DIST_DIR / "r2100_hil.pyz"), "check"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            result.returncode,
            0,
            msg=result.stderr,
        )

        self.assertIn(
            "Deployment configuration: PASS",
            result.stdout,
        )
        self.assertIn(
            "Profile             : ruskotunturi_hil",
            result.stdout,
        )

    def test_zipapp_main_help_passes(self):
        result = subprocess.run(
            [str(DIST_DIR / "r2100_hil.pyz"), "--help"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            result.returncode,
            0,
            msg=result.stderr,
        )

        self.assertIn(
            "R2100 HIL deployment executable",
            result.stdout,
        )

    def test_component_help_passes_without_can_hardware(self):
        for component in [
            "perception",
            "gateway",
            "aeb",
        ]:
            with self.subTest(component=component):
                result = subprocess.run(
                    [
                        str(DIST_DIR / "r2100_hil.pyz"),
                        component,
                        "--help",
                    ],
                    cwd=REPO_ROOT,
                    capture_output=True,
                    text=True,
                )

                self.assertEqual(
                    result.returncode,
                    0,
                    msg=(
                        f"{component}\n"
                        f"stdout:\n{result.stdout}\n"
                        f"stderr:\n{result.stderr}"
                    ),
                )


if __name__ == "__main__":
    unittest.main()
