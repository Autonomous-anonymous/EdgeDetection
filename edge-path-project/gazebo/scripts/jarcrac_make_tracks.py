#!/usr/bin/env python3
"""Generate isolated JARCRAC native-contact-track worlds.

Rigid contact belts, not articulated Mevea rope tracks.
Original JARCRAC model, sensors and ADAS controllers remain unchanged.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

OUT = Path.home() / 'jarcrac_imu_experiment'
CMD = '/model/jarcrac/tracks_cmd_vel'
ODOM = '/model/jarcrac/tracks_odometry'
CONTACT = '/jarcrac/tracks/chassis_contacts'
WORLDS = {
    'flat': 'jarcrac_tracks_flat',
    'terrain': 'jarcrac_tracks_terrain',
}

WIDTH = 0.24
SEPARATION = 1.27
BELT_MASS = 60.0  # Replaces six 10 kg wheels on each side
LENGTH = 2.10
HEIGHT = 0.24


def tag(parent, element_name, value=None, **attrs):
    element = ET.SubElement(parent, element_name, attrs)
    if value is not None:
        element.text = str(value)
    return element


def geometry(parent, kind, x=0., visual=False):
    if kind == 'bar':
        tag(parent, 'pose', '0 0 -0.02 0 0 0')
        box = tag(tag(parent, 'geometry'), 'box')
        tag(box, 'size', '1.86 0.24 0.18')
    else:
        tag(
            parent, 'pose',
            f'{x:.5f} 0 0.01 1.570796327 0 0'
        )
        cylinder = tag(tag(parent, 'geometry'), 'cylinder')
        tag(cylinder, 'radius', '0.12')
        tag(cylinder, 'length', str(WIDTH))

    if visual:
        material = tag(parent, 'material')
        tag(material, 'diffuse', '0.08 0.09 0.11 1')
        tag(material, 'ambient', '0.08 0.09 0.11 1')
    else:
        ode = tag(tag(tag(parent, 'surface'), 'friction'), 'ode')
        tag(ode, 'mu', '0.7')
        tag(ode, 'mu2', '150')
        tag(ode, 'fdir1', '0 1 0')


def add_track(model, side):
    name = f'{side}_track'
    y = SEPARATION / 2 * (1 if side == 'left' else -1)

    link = tag(model, 'link', name=name)
    tag(link, 'pose', f'0 {y:.6f} 0.11 0 0 0')

    inertial = tag(link, 'inertial')
    tag(inertial, 'mass', str(BELT_MASS))

    inertia = tag(inertial, 'inertia')
    dims = (LENGTH, WIDTH, HEIGHT)
    for axis, a, b in (
        ('ixx', 1, 2),
        ('iyy', 0, 2),
        ('izz', 0, 1),
    ):
        value = BELT_MASS * (dims[a]**2 + dims[b]**2) / 12
        tag(inertia, axis, f'{value:.6f}')

    for axis in ('ixy', 'ixz', 'iyz'):
        tag(inertia, axis, '0')

    # Three collision shapes form each non-deformable belt:
    # a straight bottom contact section and rounded ends.
    for shape, x in (
        ('bar', 0.),
        ('front', 0.93),
        ('rear', -0.93),
    ):
        kind = 'bar' if shape == 'bar' else 'roller'
        for element_type in ('collision', 'visual'):
            item = tag(
                link, element_type,
                name=f'{name}_{shape}_{element_type}'
            )
            geometry(
                item, kind, x,
                visual=(element_type == 'visual'),
            )

    joint = tag(
        model, 'joint',
        name=f'{name}_fixed',
        type='fixed'
    )
    tag(joint, 'parent', 'chassis')
    tag(joint, 'child', name)


def add_plugins(model):
    vehicle = tag(
        model, 'plugin',
        filename='gz-sim-tracked-vehicle-system',
        name='gz::sim::systems::TrackedVehicle'
    )

    for side in ('left', 'right'):
        entry = tag(vehicle, f'{side}_track')
        tag(entry, 'link', f'{side}_track')

    tag(vehicle, 'tracks_separation', str(SEPARATION))
    # Gazebo Sim 8 example uses this spelling.
    tag(vehicle, 'tracks_height', str(HEIGHT))
    tag(vehicle, 'steering_efficiency', '0.5')
    tag(vehicle, 'topic', CMD)
    tag(vehicle, 'odom_topic', ODOM)
    tag(vehicle, 'odom_publish_frequency', '30')

    linear = tag(vehicle, 'linear_velocity')
    tag(linear, 'min_velocity', '-0.3')
    tag(linear, 'max_velocity', '0.3')

    angular = tag(vehicle, 'angular_velocity')
    tag(angular, 'min_velocity', '-0.25')
    tag(angular, 'max_velocity', '0.25')

    for side in ('left', 'right'):
        controller = tag(
            model, 'plugin',
            filename='gz-sim-track-controller-system',
            name='gz::sim::systems::TrackController'
        )
        tag(controller, 'link', f'{side}_track')
        tag(controller, 'min_velocity', '-0.3')
        tag(controller, 'max_velocity', '0.3')
        tag(controller, 'max_command_age', '0.3')


def convert_model(model):
    if model is None or model.get('name') != 'jarcrac':
        raise RuntimeError('Expected embedded JARCRAC')

    lidar = model.find(
        "./link[@name='chassis']/"
        "sensor[@name='gpu_lidar_front']"
    )
    if (
        lidar is None
        or lidar.findtext('ray/scan/vertical/samples') != '32'
    ):
        raise RuntimeError('Wrong JARCRAC 3D LiDAR')

    links = model.findall('link')
    joints = model.findall('joint')
    wheel_links = [
        item for item in links
        if item.get('name', '').startswith('wheel')
    ]
    wheel_joints = [
        item for item in joints
        if item.get('name', '').startswith('wheel')
    ]

    if len(wheel_links) != 12 or len(wheel_joints) != 12:
        raise RuntimeError('Expected 12 original wheel links/joints')

    plugins = [
        p for p in model.findall('plugin')
        if p.get('name') == 'gz::sim::systems::DiffDrive'
    ]
    if len(plugins) != 1:
        raise RuntimeError('Expected one original DiffDrive')

    original_mass = sum(
        float(link.findtext('inertial/mass'))
        for link in links
    )

    for item in wheel_links + wheel_joints:
        model.remove(item)
    model.remove(plugins[0])

    add_track(model, 'left')
    add_track(model, 'right')
    add_plugins(model)

    final_mass = sum(
        float(link.findtext('inertial/mass'))
        for link in model.findall('link')
    )

    if not math.isclose(
        original_mass, final_mass, abs_tol=1e-5
    ):
        raise RuntimeError(
            f'Mass mismatch: {original_mass} -> {final_mass}'
        )

    return final_mass


def prepare(case):
    from jarcrac_imu_stage import (
        GZ, MODEL, SOURCES, add_imu_world
    )

    name = WORLDS[case]
    source = (
        GZ / 'worlds/jarcrac_flat_test_world.sdf'
        if case == 'flat'
        else SOURCES['obstacle']
    )
    output = OUT / f'{name}.sdf'

    add_imu_world(source, MODEL, output, name)

    tree = ET.parse(output)
    world = tree.getroot().find('world')
    model = world.find("./model[@name='jarcrac']")

    mass = convert_model(model)

    # Monitor chassis collisions separately from expected
    # track-to-ground contacts.
    tag(
        world, 'plugin',
        filename='gz-sim-contact-system',
        name='gz::sim::systems::Contact'
    )
    chassis = model.find("./link[@name='chassis']")
    sensor = tag(
        chassis, 'sensor',
        name='tracks_chassis_contact',
        type='contact'
    )
    tag(sensor, 'always_on', 'true')
    tag(sensor, 'update_rate', '50')
    tag(sensor, 'topic', CONTACT)
    tag(tag(sensor, 'contact'), 'collision', 'body_collision')

    ET.indent(tree, space='  ')
    tree.write(
        output,
        encoding='utf-8',
        xml_declaration=True,
    )

    # Check the generated file, not just the in-memory XML.
    check = ET.parse(output).getroot().find('world')
    generated = check.find("./model[@name='jarcrac']")
    kinds = [
        p.get('name') for p in generated.findall('plugin')
    ]

    assert check.get('name') == name
    assert kinds.count(
        'gz::sim::systems::TrackedVehicle'
    ) == 1
    assert kinds.count(
        'gz::sim::systems::TrackController'
    ) == 2
    assert 'gz::sim::systems::DiffDrive' not in kinds
    assert generated.find(
        "./link[@name='left_track']"
    ) is not None
    assert generated.find(
        "./link[@name='right_track']"
    ) is not None
    assert generated.findtext(
        "./plugin[@name='gz::sim::systems::TrackedVehicle']"
        "/topic"
    ) == CMD
    assert generated.findtext(
        "./link[@name='chassis']/sensor[@type='imu']/topic"
    ) == '/jarcrac/imu'
    assert generated.findtext(
        "./link[@name='chassis']/"
        "sensor[@name='gpu_lidar_front']/topic"
    ) == '/jarcrac/lidar'

    print(f'{case.upper()} WORLD: {output}')
    print(f'Total mass: {mass:.3f} kg')
    print(f'Command topic: {CMD}')
    print('TrackedVehicle + two TrackControllers: configured')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--case',
        choices=('flat', 'terrain', 'both'),
        default='both',
    )
    args = parser.parse_args()

    cases = (
        ('flat', 'terrain')
        if args.case == 'both'
        else (args.case,)
    )
    for case in cases:
        prepare(case)

    print('Original JARCRAC model unchanged.')
    print('Physical track behaviour is NOT yet validated.')


if __name__ == '__main__':
    main()
