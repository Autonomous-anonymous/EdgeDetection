#!/usr/bin/env python3
from pathlib import Path
import re
import shutil


REPO_ROOT = Path.home() / "edge-path-project-git"
GAZEBO_DIR = REPO_ROOT / "edge-path-project" / "gazebo"

SOURCE_DIR = GAZEBO_DIR / "models" / "aeb_vehicle"
TARGET_DIR = GAZEBO_DIR / "models" / "aeb_vehicle_360"

SOURCE_SDF = SOURCE_DIR / "model.sdf"
TARGET_SDF = TARGET_DIR / "model.sdf"

TARGET_CONFIG = TARGET_DIR / "model.config"


NEW_SENSOR_BLOCK = '''
    <sensor name="gpu_lidar_360" type="gpu_lidar">
      <!-- Roof-centred 360-degree LiDAR. -->
      <pose>0 0 0.85 0 0 0</pose>
      <topic>/aeb360/lidar</topic>
      <update_rate>10</update_rate>

      <ray>
        <scan>
          <horizontal>
            <!-- About 0.5 degree angular spacing over 360 degrees. -->
            <samples>721</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>

        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>

      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>

    <sensor name="front_rgb_camera" type="camera">
      <!-- Front RGB camera. Camera looks along the vehicle +X axis. -->
      <pose>0.9 0 0.75 0 0 0</pose>
      <topic>/aeb360/camera/image</topic>
      <update_rate>15</update_rate>

      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>

        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>

        <clip>
          <near>0.10</near>
          <far>100.0</far>
        </clip>
      </camera>

      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>
'''


def main():
    if not SOURCE_SDF.exists():
        print(f"ERROR: source model not found: {SOURCE_SDF}")
        raise SystemExit(1)

    if TARGET_DIR.exists():
        print(f"Removing old generated directory: {TARGET_DIR}")
        shutil.rmtree(TARGET_DIR)

    shutil.copytree(SOURCE_DIR, TARGET_DIR)

    text = TARGET_SDF.read_text(encoding="utf-8")

    text = text.replace("aeb_vehicle", "aeb_vehicle_360")
    text = text.replace("/aeb/lidar", "/aeb360/lidar")

    sensor_pattern = re.compile(
        r'<sensor\b[^>]*type=["\']gpu_lidar["\'][^>]*>.*?</sensor>',
        re.IGNORECASE | re.DOTALL,
    )

    matches = list(sensor_pattern.finditer(text))
    if len(matches) != 1:
        print(
            "ERROR: expected exactly one gpu_lidar sensor in the source model, "
            f"found {len(matches)}."
        )
        raise SystemExit(1)

    text = sensor_pattern.sub(
        NEW_SENSOR_BLOCK.strip(),
        text,
        count=1,
    )

    TARGET_SDF.write_text(
        text,
        encoding="utf-8",
    )

    if TARGET_CONFIG.exists():
        config = TARGET_CONFIG.read_text(
            encoding="utf-8",
        )
        config = config.replace(
            "aeb_vehicle",
            "aeb_vehicle_360",
        )
        TARGET_CONFIG.write_text(
            config,
            encoding="utf-8",
        )

    print()
    print("360-degree vehicle model created successfully.")
    print(f"Source : {SOURCE_DIR}")
    print(f"Target : {TARGET_DIR}")
    print()
    print("Original aeb_vehicle was NOT modified.")
    print("New LiDAR topic: /aeb360/lidar")
    print("Horizontal FOV: 360 degrees")
    print("Samples: 721")
    print("LiDAR update rate: 10 Hz")
    print("Camera topic: /aeb360/camera/image")
    print("Camera: 640x480 RGB, 15 Hz, 80 degree horizontal FOV")


if __name__ == "__main__":
    main()
