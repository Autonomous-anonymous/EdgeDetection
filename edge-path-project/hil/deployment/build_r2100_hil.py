#!/usr/bin/env python3
"""Build the deployable R2100 HIL Python zipapp."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import zipapp


HIL_DIR = Path(__file__).resolve().parents[1]

DEPLOYMENT_DIR = HIL_DIR / "deployment"

OUTPUT_DIR = HIL_DIR / "dist" / "r2100_hil"
OUTPUT_APP = OUTPUT_DIR / "r2100_hil.pyz"

RUNTIME_FILES = [
    "can_protocol.py",
    "signal_mapping.py",
    "signal_codec.py",
    "r2100_can_gateway.py",
    "r2100_lidar_perception.py",
    "r2100_aeb_controller.py",
]

CONFIG_FILES = [
    "ruskotunturi_hil.json",
    "ruskotunturi_hil_alt_ids.json",
    "ruskotunturi_hil_alt_scaling.json",
]

DEPLOYMENT_SCRIPTS = [
    "start_hil.sh",
    "stop_hil.sh",
    "status_hil.sh",
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)

    return digest.hexdigest()


def main() -> None:
    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)

    OUTPUT_DIR.mkdir(parents=True)

    config_output = OUTPUT_DIR / "config"
    config_output.mkdir()

    with tempfile.TemporaryDirectory() as temp_dir:
        app_dir = Path(temp_dir)

        for filename in RUNTIME_FILES:
            shutil.copy2(
                HIL_DIR / filename,
                app_dir / filename,
            )

        shutil.copy2(
            DEPLOYMENT_DIR / "r2100_hil_main.py",
            app_dir / "__main__.py",
        )

        zipapp.create_archive(
            app_dir,
            target=OUTPUT_APP,
            interpreter="/usr/bin/env python3",
            compressed=True,
        )

    OUTPUT_APP.chmod(0o755)

    for filename in CONFIG_FILES:
        shutil.copy2(
            HIL_DIR / "config" / filename,
            config_output / filename,
        )

    for filename in DEPLOYMENT_SCRIPTS:
        destination = OUTPUT_DIR / filename

        shutil.copy2(
            DEPLOYMENT_DIR / filename,
            destination,
        )

        destination.chmod(0o755)

    manifest = {
        "format": "python-zipapp",
        "entrypoint": "r2100_hil.pyz",
        "python_requirement": "python3",
        "runtime_components": [
            "perception",
            "gateway",
            "aeb",
        ],
        "files": {},
    }

    for path in [
        OUTPUT_APP,
        *sorted(config_output.iterdir()),
        *(OUTPUT_DIR / name for name in DEPLOYMENT_SCRIPTS),
    ]:
        relative = path.relative_to(OUTPUT_DIR)

        manifest["files"][str(relative)] = {
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
        }

    manifest_path = OUTPUT_DIR / "manifest.json"

    manifest_path.write_text(
        json.dumps(manifest, indent=2) + "\n"
    )

    print("R2100 HIL deployment build: PASS")
    print(f"Output: {OUTPUT_DIR}")
    print()
    print("Files:")

    for path in sorted(OUTPUT_DIR.rglob("*")):
        if path.is_file():
            print(f"  {path.relative_to(OUTPUT_DIR)}")


if __name__ == "__main__":
    main()
