#!/usr/bin/env python3
"""Launch a fresh, isolated two-vehicle CAMINO pitch in Gazebo Harmonic."""

import argparse
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

GAZEBO = Path(__file__).resolve().parents[1]
BASE_WORLD = GAZEBO / "worlds" / "ruskotunturi_final_sil_demo_v6_world.sdf"
PAINT = {"A": "0.02 0.70 0.64 1", "B": "0.48 0.18 0.80 1"}
# Roof lettering in the chassis XY plane, below both optical sensors.
LETTERS = {
    "A": [((-.45, -.83), (0, .83)), ((0, .83), (.45, -.83)), ((-.30, -.20), (.30, -.20))],
    "B": [((-.40, -.82), (-.40, .82)), ((-.40, .82), (.24, .82)), ((.24, .82), (.42, .54)),
          ((.42, .54), (.42, .20)), ((.42, .20), (.18, 0)), ((.18, 0), (-.40, 0)),
          ((.18, 0), (.43, -.20)), ((.43, -.20), (.43, -.57)), ((.43, -.57), (.22, -.82)),
          ((.22, -.82), (-.40, -.82))],
}


def paint_vehicle(model, letter):
    chassis = model.find("link[@name='chassis']")
    material = chassis.find("visual[@name='chassis_visual']/material")
    for name in ("ambient", "diffuse"):
        material.find(name).text = PAINT[letter]
    for index, ((u, v), (u2, v2)) in enumerate(LETTERS[letter]):
        dx, dy = v - v2, u2 - u
        mark = ET.SubElement(chassis, "visual", name=f"pitch_{letter}_{index}")
        ET.SubElement(mark, "pose").text = f"{-(v+v2)/2} {(u+u2)/2} .183 0 0 {math.atan2(dy, dx)}"
        box = ET.SubElement(ET.SubElement(mark, "geometry"), "box")
        ET.SubElement(box, "size").text = f"{math.hypot(dx, dy):.4f} .105 .012"
        white = ET.SubElement(mark, "material")
        for name in ("ambient", "diffuse", "emissive"):
            ET.SubElement(white, name).text = "0.95 0.98 1 1" if name != "emissive" else "0.24 0.27 0.30 1"


def make_world(path):
    tree = ET.parse(BASE_WORLD)
    world = tree.getroot().find("world")
    world.find("physics/max_step_size").text = "0.004"
    world.find("scene/shadows").text = "false"
    world.find("light/cast_shadows").text = "false"
    vehicles = {"aeb_vehicle_360": "A", "aeb_vehicle_360_b": "B"}
    for include in world.findall("include"):
        name = include.findtext("name")
        if name in vehicles:
            model = ET.parse(GAZEBO / "models" / name / "model.sdf").getroot().find("model")
            model.insert(0, include.find("pose"))
            paint_vehicle(model, vehicles[name])
            chassis = model.find("link[@name='chassis']")
            camera = chassis.find("sensor[@type='camera']")
            if vehicles[name] == "A":
                camera.find("camera/image/width").text = "320"
                camera.find("camera/image/height").text = "240"
                camera.find("update_rate").text = "10"
            else:
                chassis.remove(camera)  # B's controller uses its full raw LiDAR scan.
            for sensor in chassis.findall("sensor"):
                sensor.find("visualize").text = "false"
            world.remove(include)
            world.append(model)
    # Short centre dashes on the common exit, aligned to the collision mesh.
    marks = ET.SubElement(world, "model", name="pitch_shared_lane")
    ET.SubElement(marks, "static").text = "true"
    road = ET.SubElement(marks, "link", name="paint")
    for index, (z, slope) in enumerate(((-10.080, -.023), (-10.006, -.057), (-9.910, -.030), (-9.835, -.038),
                                        (-9.797, -.044), (-9.803, .019), (-9.872, .021), (-9.897, .059))):
        mark = ET.SubElement(road, "visual", name=f"dash_{index}")
        ET.SubElement(mark, "pose").text = f"{-4.5-index*1.5} {27.5-index*1.5} {z} 0 {slope} {-3*math.pi/4}"
        box = ET.SubElement(ET.SubElement(mark, "geometry"), "box")
        ET.SubElement(box, "size").text = ".95 .20 .015"
        material = ET.SubElement(mark, "material")
        for key in ("ambient", "diffuse"):
            ET.SubElement(material, key).text = ".95 .96 .96 1"
    # An initial wide shot only. The audience's camera is independent of A's sensor.
    gui = ET.SubElement(world, "gui")
    scene = ET.SubElement(gui, "plugin", filename="MinimalScene", name="3D View")
    layout = ET.SubElement(scene, "gz-gui")
    ET.SubElement(layout, "property", key="state", type="string").text = "docked"
    ET.SubElement(layout, "property", key="showTitleBar", type="bool").text = "false"
    ET.SubElement(scene, "engine").text = "ogre2"
    ET.SubElement(scene, "scene").text = "scene"
    ET.SubElement(scene, "camera_pose").text = "4 7 20 0 .95 1.5708"
    for plugin in ("GzSceneManager", "InteractiveViewControl", "CameraTracking"):
        layout = ET.SubElement(ET.SubElement(gui, "plugin", filename=plugin, name=plugin), "gz-gui")
        for key, kind, value in (("state", "string", "floating"), ("showTitleBar", "bool", "false"),
                                 ("width", "double", "1"), ("height", "double", "1")):
            ET.SubElement(layout, "property", key=key, type=kind).text = value
    tree.write(path, encoding="utf-8", xml_declaration=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["sil", "hil"], help="SIL rehearsal or live R2100/CAN")
    parser.add_argument("--r2100-host", help="R2100 Ethernet address (HIL only)")
    parser.add_argument("--headless", action="store_true", help="server with EGL rendering, no GUI or panel")
    parser.add_argument("--no-panel", action="store_true", help="show status in the terminal only")
    parser.add_argument("--auto-start", action="store_true", help="start and exit without presenter prompts")
    parser.add_argument("--results-dir", type=Path, default=GAZEBO / "results" / "pitch")
    args = parser.parse_args(argv)
    if (args.mode == "hil") != bool(args.r2100_host):
        parser.error("hil requires --r2100-host ADDRESS; sil takes no R2100 address")
    return args


def close_world(process):
    # gz may have separate server and GUI children. Only stop our own session.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main(argv=None):
    args = arguments(argv)
    if not shutil.which("gz"):
        sys.exit("Gazebo Harmonic (gz) is required on the presentation computer.")
    os.environ["GZ_PARTITION"] = "camino_pitch_" + uuid.uuid4().hex
    print(f"GZ_PARTITION={os.environ['GZ_PARTITION']}", flush=True)
    previous = os.environ.get("GZ_SIM_RESOURCE_PATH")
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(GAZEBO / "models") + (os.pathsep + previous if previous else "")
    # Import after selecting the transport partition, before launching any world.
    from pitch_controller import PitchDemo
    from pitch_panel import PitchPanel

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    with tempfile.TemporaryDirectory(prefix="camino-pitch-") as directory:
        path = Path(directory) / "pitch.sdf"
        make_world(path)
        cmd = ["gz", "sim", "-r"]
        if args.headless:
            cmd += ["-s", "--headless-rendering"]
        launch_wall = time.monotonic()
        process = subprocess.Popen([*cmd, str(path)], start_new_session=True)
        panel = PitchPanel()
        try:
            if not args.headless and not args.no_panel:
                panel.open()
            panel.update("Lähtöpaikalla", "Lähtöpaikalla", "SIL · simulaatio" if args.mode == "sil" else "CAN · odottaa yhteyttä", "Valmistellaan", live=False)
            if process.poll() is not None:
                sys.exit("Gazebo exited before the demo began.")
            controller = PitchDemo(args.r2100_host, args.results_dir, panel, launch_wall, interactive=not args.auto_start)
            passed = controller.run()
            if not args.auto_start:
                input("Ajo päättyi. Sulje näkymä painamalla Enter. ")
            return 0 if passed else 1
        finally:
            try:
                close_world(process)
            finally:
                panel.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        sys.exit(130)
