#!/usr/bin/env python3
"""Build a standalone native R2100 HIL executable with PyInstaller."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile


HIL_DIR = Path(__file__).resolve().parents[1]
DEPLOYMENT_DIR = HIL_DIR / "deployment"
OUTPUT_DIR = HIL_DIR / "dist" / "r2100_hil"
OUTPUT_APP = OUTPUT_DIR / "r2100_hil"
INTERFACE_PATH = HIL_DIR / "config" / "ruskotunturi_aeb_interface.json"

sys.path.insert(0, str(HIL_DIR))
from stage_signal_config import write_stage_config  # noqa: E402


CONFIG_FILES = [
    "ruskotunturi_hil_alt_ids.json",
    "ruskotunturi_hil_alt_scaling.json",
]
DEPLOYMENT_SCRIPTS = ["start_hil.sh", "stop_hil.sh", "status_hil.sh"]
HIDDEN_IMPORTS = [
    "r2100_preflight",
    "r2100_can_gateway",
    "r2100_lidar_perception",
    "r2100_aeb_controller",
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalized_architecture(machine: str | None = None) -> str:
    machine = (machine or platform.machine()).lower()
    aliases = {
        "arm64": "aarch64",
        "amd64": "x86_64",
    }
    return aliases.get(machine, machine)


def pyinstaller_command(dist_dir: Path, work_dir: Path, spec_dir: Path) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--onefile",
        "--clean",
        "--noconfirm",
        "--name",
        "r2100_hil",
        "--distpath",
        str(dist_dir),
        "--workpath",
        str(work_dir),
        "--specpath",
        str(spec_dir),
        "--paths",
        str(HIL_DIR),
    ]
    for module in HIDDEN_IMPORTS:
        command.extend(["--hidden-import", module])
    command.append(str(DEPLOYMENT_DIR / "r2100_hil_main.py"))
    return command


def verify_executable(path: Path) -> None:
    if not path.is_file():
        raise RuntimeError(f"standalone executable was not produced: {path}")
    if not path.stat().st_mode & 0o111:
        path.chmod(path.stat().st_mode | 0o755)
    with path.open("rb") as handle:
        magic = handle.read(4)
    if magic != b"\x7fELF":
        raise RuntimeError(f"expected Linux ELF executable, got magic {magic!r}")


def build_manifest(files: list[Path], pyinstaller_version: str) -> dict:
    architecture = normalized_architecture()
    manifest = {
        "format": "standalone-linux-executable",
        "entrypoint": "r2100_hil",
        "architecture": architecture,
        "python_requirement": None,
        "builder": {
            "tool": "PyInstaller",
            "version": pyinstaller_version,
        },
        "signal_interface": {
            "path": "config/ruskotunturi_aeb_interface.json",
            "sha256": sha256(INTERFACE_PATH),
        },
        "runtime_components": ["perception", "gateway", "aeb"],
        "files": {},
    }
    for path in files:
        relative = path.relative_to(OUTPUT_DIR)
        manifest["files"][str(relative)] = {
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
        }
    return manifest


def build() -> None:
    try:
        pyinstaller_version = importlib.metadata.version("pyinstaller")
    except importlib.metadata.PackageNotFoundError as exc:
        raise SystemExit(
            "PyInstaller is required to build the standalone executable. "
            "Install the pinned build dependency before running this script."
        ) from exc

    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)
    OUTPUT_DIR.mkdir(parents=True)
    config_output = OUTPUT_DIR / "config"
    config_output.mkdir()

    # The default HIL CAN mapping is generated, never hand-maintained.
    write_stage_config(
        INTERFACE_PATH,
        "hil",
        config_output / "ruskotunturi_hil.json",
    )
    shutil.copy2(INTERFACE_PATH, config_output / INTERFACE_PATH.name)

    for filename in CONFIG_FILES:
        source = HIL_DIR / "config" / filename
        if source.exists():
            shutil.copy2(source, config_output / filename)

    with tempfile.TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        dist = temp / "dist"
        work = temp / "work"
        spec = temp / "spec"
        subprocess.run(pyinstaller_command(dist, work, spec), check=True)
        shutil.copy2(dist / "r2100_hil", OUTPUT_APP)

    OUTPUT_APP.chmod(0o755)
    verify_executable(OUTPUT_APP)

    for filename in DEPLOYMENT_SCRIPTS:
        source = DEPLOYMENT_DIR / filename
        if not source.exists():
            continue
        destination = OUTPUT_DIR / filename
        shutil.copy2(source, destination)
        destination.chmod(0o755)

    files = [path for path in sorted(OUTPUT_DIR.rglob("*")) if path.is_file()]
    manifest = build_manifest(files, pyinstaller_version)
    (OUTPUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    print("R2100 HIL standalone build: PASS")
    print(f"Architecture: {manifest['architecture']}")
    print(f"Output: {OUTPUT_DIR}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    build()


if __name__ == "__main__":
    main()
