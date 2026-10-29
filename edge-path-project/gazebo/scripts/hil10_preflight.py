#!/usr/bin/env python3
"""Read-only check of HIL-10 dependencies against its tested HIL-9 baseline."""

import hashlib
from pathlib import Path

BASE_COMMIT = "52331fde756991988cbb94f01e6d33d615c8d7df"
EXPECTED = {
    "edge-path-project/gazebo/scripts/ruskotunturi_phase6_two_vehicle_test.py": "f73d0c4ba1475b5c552d515c9ba6250ec36cf82f",
    "edge-path-project/gazebo/scripts/ruskotunturi_hil_b_route_test.py": "86283a9da24e6d4977524562fd7a46a948f7705c",
    "edge-path-project/gazebo/scripts/ruskotunturi_hil9e_aeb_test.py": "c0d3455196ed1b6c862054aaadf67a54b4bff4c9",
    "edge-path-project/hil/can_protocol.py": "1354e8e27cd69a44f55d7e06bf6d8def22a032c4",
    "edge-path-project/hil/gazebo_lidar_sender.py": "7e8d05103ed2e2aa976b9b7b0c218640ef46e8a8",
    "edge-path-project/hil/r2100_aeb_controller.py": "57c9d83c58e1d2f1a85022929c1c23a7eedbf67b",
    "edge-path-project/hil/r2100_can_gateway.py": "1ef2b77e606e3211f3dbd424e301ff61891b1bb6",
    "edge-path-project/hil/r2100_lidar_perception.py": "c87e3214adee39f92783a2125aa722400bcceba6",
    "edge-path-project/gazebo/worlds/ruskotunturi_phase6_two_vehicle_world.sdf": "f4ebdc1b12bf9573283f4752ae5c730b65697fd6",
    "edge-path-project/gazebo/worlds/ruskotunturi_hil9e_aeb_world.sdf": "486a29a49cd784840402a3fd2cf4eeec868c7a3e"
}


def main():
    root = Path(__file__).resolve().parents[3]
    failures = []
    for relative, expected in EXPECTED.items():
        path = root / relative
        if not path.is_file():
            status = "MISSING"
        else:
            # Git blob hash; tolerate a checkout converting LF to CRLF.
            content = path.read_bytes().replace(b"\r\n", b"\n")
            actual = hashlib.sha1(b"blob " + str(len(content)).encode() + b"\0" + content).hexdigest()
            status = "OK" if actual == expected else "DIFFERS"
        print(f"{status:7s} {relative}")
        if status != "OK":
            failures.append(relative)
    print("\nBaseline:", BASE_COMMIT)
    if failures:
        print("Dependency mismatch: inspect local differences before the hardware run.")
        print("This check has not modified any files.")
        return 1
    print("Baseline dependencies match. Hardware/Gazebo validation is still required.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
