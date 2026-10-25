#!/usr/bin/env python3

from pathlib import Path
import shutil


REPO_ROOT = Path.home() / "edge-path-project-git"
GAZEBO_DIR = REPO_ROOT / "edge-path-project" / "gazebo"

SOURCE_DIR = (
    GAZEBO_DIR
    / "models"
    / "aeb_vehicle_360"
)

TARGET_DIR = (
    GAZEBO_DIR
    / "models"
    / "aeb_vehicle_360_b"
)

SOURCE_SDF = SOURCE_DIR / "model.sdf"
TARGET_SDF = TARGET_DIR / "model.sdf"

TARGET_CONFIG = (
    TARGET_DIR
    / "model.config"
)


def main():

    if not SOURCE_SDF.exists():

        print(
            "ERROR: source model not found:"
        )

        print(
            SOURCE_SDF
        )

        raise SystemExit(
            1
        )


    if TARGET_DIR.exists():

        print(
            "Removing old generated B model:"
        )

        print(
            TARGET_DIR
        )

        shutil.rmtree(
            TARGET_DIR
        )


    shutil.copytree(
        SOURCE_DIR,
        TARGET_DIR
    )


    text = TARGET_SDF.read_text(
        encoding="utf-8"
    )

    # Independent Gazebo model / command / odometry topics.
    text = text.replace(
        "aeb_vehicle_360",
        "aeb_vehicle_360_b"
    )

    # Independent absolute sensor topics.
    text = text.replace(
        "/aeb360/lidar",
        "/aeb360b/lidar"
    )

    text = text.replace(
        "/aeb360/camera/image",
        "/aeb360b/camera/image"
    )


    TARGET_SDF.write_text(
        text,
        encoding="utf-8"
    )


    if TARGET_CONFIG.exists():

        config = (
            TARGET_CONFIG.read_text(
                encoding="utf-8"
            )
        )

        config = config.replace(
            "aeb_vehicle_360",
            "aeb_vehicle_360_b"
        )

        TARGET_CONFIG.write_text(
            config,
            encoding="utf-8"
        )


    print()
    print(
        "Vehicle B model created successfully."
    )

    print(
        f"Source : {SOURCE_DIR}"
    )

    print(
        f"Target : {TARGET_DIR}"
    )

    print()
    print(
        "Vehicle A model was NOT modified."
    )

    print(
        "Vehicle B command topic:"
    )

    print(
        "/model/aeb_vehicle_360_b/cmd_vel"
    )

    print(
        "Vehicle B LiDAR topic:"
    )

    print(
        "/aeb360b/lidar"
    )

    print(
        "Vehicle B camera topic:"
    )

    print(
        "/aeb360b/camera/image"
    )


if __name__ == "__main__":

    main()
