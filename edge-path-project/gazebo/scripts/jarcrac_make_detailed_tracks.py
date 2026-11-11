#!/usr/bin/env python3
"""Import detailed JARCRAC tracks into a separate experimental model."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[3]
GZ = ROOT / 'edge-path-project/gazebo'
BASE = GZ / 'models/jarcrac/model.sdf'
MODEL_DIR = GZ / 'models/jarcrac_detailed_tracks'
MODEL_FILE = MODEL_DIR / 'model.sdf'
OUT = Path.home() / 'jarcrac_imu_experiment'

CMD = '/model/jarcrac/detailed_cmd_vel'
ODOM = '/model/jarcrac/detailed_odometry'
CONTACT = '/jarcrac/detailed/chassis_contacts'

WORLDS = {
    'flat': 'jarcrac_detailed_flat',
    'terrain': 'jarcrac_detailed_terrain',
}
ZIP_ROOT = 'edge-path-project/gazebo/models/jarcrac/'
ASSETS = (
    'track_left_belt.stl',
    'track_right_belt.stl',
    'track_left_collision.stl',
    'track_right_collision.stl',
)


def add(parent, element_name, value=None, **attrs):
    elem = ET.SubElement(parent, element_name, attrs)
    if value is not None:
        elem.text = str(value)
    return elem


def numerical_pose(text):
    parts = list(map(float, (text or '').split()))
    if len(parts) != 6 or not all(map(math.isfinite, parts)):
        raise RuntimeError('Invalid six-component SDF pose')
    return parts


def original_model():
    model = ET.parse(BASE).getroot().find('model')
    chassis = model.find("./link[@name='chassis']")
    lidar = chassis.find("./sensor[@name='gpu_lidar_front']")

    if (
        lidar is None
        or lidar.get('type') != 'gpu_lidar'
        or lidar.findtext('ray/scan/horizontal/samples') != '721'
        or lidar.findtext('ray/scan/vertical/samples') != '32'
    ):
        raise RuntimeError('Verified original 3D LiDAR missing')

    return model, chassis


def check_mesh(data, name):
    if len(data) < 84:
        raise RuntimeError('Invalid STL: ' + name)

    count = struct.unpack_from('<I', data, 80)[0]
    if count < 10 or len(data) != 84 + count*50:
        raise RuntimeError('Invalid binary STL: ' + name)

    bounds = [[], [], []]
    for i in range(count):
        v = struct.unpack_from('<12f', data, 84+i*50)
        for j in (3, 6, 9):
            for axis in range(3):
                bounds[axis].append(v[j+axis])

    if not all(math.isfinite(v) for axis in bounds for v in axis):
        raise RuntimeError('Nonfinite mesh coordinates')

    extents = [max(axis)-min(axis) for axis in bounds]
    if extents[0] < 2 or extents[1] < .45 or extents[2] < .6:
        raise RuntimeError('Unexpected belt dimensions: ' + name)


def configure(model):
    if model is None or model.get('name') != 'jarcrac':
        raise RuntimeError('Wrong ZIP model')

    base, old_chassis = original_model()
    chassis = model.find("./link[@name='chassis']")

    names = {link.get('name') for link in model.findall('link')}
    if names != {'chassis', 'track_left', 'track_right'}:
        raise RuntimeError('Unexpected ZIP link structure')

    if len(model.findall('joint')) != 2:
        raise RuntimeError('Expected two fixed track joints')

    old_z = numerical_pose(old_chassis.findtext('pose'))[2]
    new_z = numerical_pose(chassis.findtext('pose'))[2]

    # Restore the validated front 3D LiDAR.
    previous = chassis.find("./sensor[@type='gpu_lidar']")
    if previous is None:
        raise RuntimeError('ZIP LiDAR missing')
    chassis.remove(previous)

    sensor = deepcopy(
        old_chassis.find("./sensor[@name='gpu_lidar_front']")
    )
    pose = numerical_pose(sensor.findtext('pose'))
    pose[2] += old_z - new_z
    sensor.find('pose').text = ' '.join(
        f'{v:.9f}' for v in pose
    )
    chassis.append(sensor)

    # Keep the camera at the same world-relative height too.
    camera = chassis.find(
        "./sensor[@name='front_rgb_camera']"
    )
    original_camera = old_chassis.find(
        "./sensor[@name='front_rgb_camera']"
    )
    if camera is None or original_camera is None:
        raise RuntimeError('Camera missing')

    chassis.remove(camera)
    camera = deepcopy(original_camera)
    pose = numerical_pose(camera.findtext('pose'))
    pose[2] += old_z-new_z
    camera.find('pose').text = ' '.join(
        f'{v:.9f}' for v in pose
    )
    chassis.append(camera)

    plugins = model.findall('plugin')
    tracked = [
        p for p in plugins
        if p.get('name') == 'gz::sim::systems::TrackedVehicle'
    ]
    controllers = [
        p for p in plugins
        if p.get('name') == 'gz::sim::systems::TrackController'
    ]

    if len(tracked) != 1 or len(controllers) != 2:
        raise RuntimeError('Missing native track plugins')

    if {p.findtext('link') for p in controllers} != {
        'track_left', 'track_right'
    }:
        raise RuntimeError('Wrong controller links')

    for controller in controllers:
        for key, value in (
            ('min_velocity', '-0.3'),
            ('max_velocity', '0.3'),
            ('max_command_age', '0.3'),
        ):
            elem = controller.find(key)
            if elem is None:
                add(controller, key, value)
            else:
                elem.text = value

    driver = tracked[0]
    if driver.findtext('track_height') is None:
        raise RuntimeError('Missing official track_height')

    for key, value in (
        ('topic', CMD),
        ('odom_topic', ODOM),
        ('odom_publish_frequency', '30'),
    ):
        elem = driver.find(key)
        if elem is None:
            add(driver, key, value)
        else:
            elem.text = value

    for key, limit in (
        ('linear_velocity', .3),
        ('angular_velocity', .25),
    ):
        group = driver.find(key)
        if group is None:
            group = add(driver, key)

        for field, value in (
            ('min_velocity', -limit),
            ('max_velocity', limit),
        ):
            elem = group.find(field)
            if elem is None:
                add(group, field, value)
            else:
                elem.text = str(value)

    # All belt assets use their own resource directory.
    for side in ('left', 'right'):
        link = model.find(f"./link[@name='track_{side}']")
        if len(link.findall('collision')) != 1:
            raise RuntimeError('Wrong track collision structure')

        for uri in link.findall('.//uri'):
            if (
                uri.text
                and uri.text.startswith(
                    'model://jarcrac/meshes/track_'
                )
            ):
                uri.text = uri.text.replace(
                    'model://jarcrac/',
                    'model://jarcrac_detailed_tracks/',
                    1,
                )

    mass = sum(
        float(l.findtext('inertial/mass'))
        for l in model.findall('link')
    )
    previous_mass = sum(
        float(l.findtext('inertial/mass'))
        for l in base.findall('link')
    )

    if not math.isclose(mass, previous_mass, abs_tol=.002):
        raise RuntimeError('Vehicle mass changed unexpectedly')

    model.set('name', 'jarcrac_detailed_tracks')
    return mass


def import_zip(path):
    if not path.is_file():
        raise FileNotFoundError(path)

    if MODEL_FILE.exists():
        raise RuntimeError(
            'Model already imported. Regenerate without --zip.'
        )

    with ZipFile(path) as archive:
        sdf = ET.fromstring(
            archive.read(ZIP_ROOT+'model.sdf')
        )
        mass = configure(sdf.find('model'))

        meshes = {}
        for name in ASSETS:
            content = archive.read(
                ZIP_ROOT+'meshes/'+name
            )
            check_mesh(content, name)
            meshes[name] = content

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    meshdir = MODEL_DIR/'meshes'
    meshdir.mkdir(exist_ok=True)

    for name, content in meshes.items():
        destination = meshdir/name
        if (
            destination.exists()
            and destination.read_bytes() != content
        ):
            raise RuntimeError(
                'Refusing to overwrite existing mesh: '+name
            )
        destination.write_bytes(content)

    ET.indent(sdf, space='  ')
    MODEL_FILE.write_bytes(
        ET.tostring(
            sdf, encoding='utf-8', xml_declaration=True
        )
    )

    (MODEL_DIR/'model.config').write_text(
        '<model>\n'
        '  <name>jarcrac_detailed_tracks</name>\n'
        '  <version>1.0</version>\n'
        '  <sdf version="1.10">model.sdf</sdf>\n'
        '  <description>Experimental crawler tracks</description>\n'
        '</model>\n'
    )

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    (MODEL_DIR/'PROVENANCE.txt').write_text(
        'Detailed track geometry imported from user ZIP.\n'
        f'ZIP SHA256: {digest}\n'
        'ZIP documentation identifies Mevea / NUVE-LAB '
        'GPL-3.0-or-later provenance.\n'
        'Verify redistribution and attribution before Git push.\n'
        'Original JARCRAC model was not modified.\n'
    )

    print(f'Imported detailed tracks: {mass:.3f} kg')
    print('Model:', MODEL_FILE)


def generate(case):
    from jarcrac_imu_stage import SOURCES

    name = WORLDS[case]
    source = (
        GZ/'worlds/jarcrac_flat_test_world.sdf'
        if case == 'flat' else SOURCES['obstacle']
    )

    tree = ET.parse(source)
    world = tree.getroot().find('world')

    includes = [
        i for i in world.findall('include')
        if i.findtext('uri') == 'model://jarcrac'
    ]
    if len(includes) != 1:
        raise RuntimeError('Expected one JARCRAC include')

    old = includes[0]
    pose = numerical_pose(old.findtext('pose'))
    model = deepcopy(
        ET.parse(MODEL_FILE).getroot().find('model')
    )
    model.set('name', 'jarcrac')
    add(model, 'pose', ' '.join(map(str, pose)))

    index = list(world).index(old)
    world.remove(old)
    world.insert(index, model)
    world.set('name', name)

    chassis = model.find("./link[@name='chassis']")
    imu = add(
        chassis, 'sensor',
        name='imu_chassis', type='imu'
    )
    add(imu, 'pose', '0 0 0 0 0 0')
    add(imu, 'topic', '/jarcrac/imu')
    add(imu, 'always_on', 'true')
    add(imu, 'update_rate', '100')
    add(
        add(
            add(imu, 'imu'),
            'orientation_reference_frame'
        ),
        'localization', 'ENU'
    )

    add(
        world, 'plugin',
        filename='gz-sim-imu-system',
        name='gz::sim::systems::Imu'
    )
    add(
        world, 'plugin',
        filename='gz-sim-contact-system',
        name='gz::sim::systems::Contact'
    )

    contact = add(
        chassis, 'sensor',
        name='detailed_chassis_contact',
        type='contact'
    )
    add(contact, 'always_on', 'true')
    add(contact, 'update_rate', '50')
    add(contact, 'topic', CONTACT)
    add(
        add(contact, 'contact'),
        'collision', 'body_collision'
    )

    ET.indent(tree, space='  ')
    OUT.mkdir(parents=True, exist_ok=True)
    target = OUT/f'{name}.sdf'
    tree.write(
        target, encoding='utf-8',
        xml_declaration=True
    )

    # Check the file actually written to disk.
    check = ET.parse(target).getroot().find('world')
    vehicle = check.find("./model[@name='jarcrac']")
    lidar = vehicle.find(
        "./link[@name='chassis']/"
        "sensor[@name='gpu_lidar_front']"
    )
    assert check.get('name') == name
    assert lidar.findtext('ray/scan/horizontal/samples') == '721'
    assert lidar.findtext('ray/scan/vertical/samples') == '32'
    assert vehicle.findtext(
        "./plugin[@name='gz::sim::systems::TrackedVehicle']"
        "/topic"
    ) == CMD
    assert len([
        p for p in vehicle.findall('plugin')
        if p.get('name') == 'gz::sim::systems::TrackController'
    ]) == 2
    assert vehicle.findtext(
        "./link[@name='chassis']/sensor[@type='imu']/topic"
    ) == '/jarcrac/imu'
    assert vehicle.findtext(
        "./link[@name='chassis']/sensor[@type='contact']/topic"
    ) == CONTACT

    print(f'Generated {case}: {target}')
    print(f'  Command: {CMD}')
    print(f'  Odometry: {ODOM}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--zip', type=Path,
        help='Detailed JARCRAC ZIP; only needed on first import'
    )
    parser.add_argument(
        '--case', choices=('flat','terrain','both'),
        default='both'
    )
    args = parser.parse_args()

    if args.zip:
        import_zip(args.zip)

    if not MODEL_FILE.is_file():
        parser.error('Run once with --zip PATH first')

    cases = (
        ('flat','terrain')
        if args.case == 'both'
        else (args.case,)
    )
    for case in cases:
        generate(case)

    print('Original JARCRAC unchanged.')
    print('Live Gazebo physics not yet validated.')


if __name__ == '__main__':
    main()
