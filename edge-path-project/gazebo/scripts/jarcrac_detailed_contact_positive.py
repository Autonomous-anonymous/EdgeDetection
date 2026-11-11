#!/usr/bin/env python3
"""Isolated stationary positive test for detailed JARCRAC chassis contacts."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

SOURCE = (
    Path.home()
    / 'jarcrac_imu_experiment/jarcrac_detailed_flat.sdf'
)
WORLD = 'jarcrac_detailed_contact_positive'
TARGET = SOURCE.parent / (WORLD + '.sdf')
TOPIC = '/jarcrac/detailed/chassis_contacts'
PUBLISHED_TOPIC = (
    f'/world/{WORLD}/model/jarcrac/link/chassis/'
    'sensor/detailed_chassis_contact/contact'
)
REPORT = Path(
    'edge-path-project/gazebo/results/jarcrac/'
    'jarcrac_detailed_contact_positive.json'
)


def add(parent, tag, value=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if value is not None:
        child.text = str(value)
    return child


def pose(element):
    if element is None:
        return (0., 0., 0., 0., 0., 0.)
    if element.get('relative_to'):
        raise RuntimeError('Unexpected relative_to pose; refusing approximation')
    values = tuple(float(x) for x in element.text.split())
    if len(values) != 6:
        raise RuntimeError('Expected six-element pose')
    return values


def rotate(v, rpy):
    x, y, z = v
    roll, pitch, yaw = rpy

    c, s = math.cos(roll), math.sin(roll)
    y, z = c*y - s*z, s*y + c*z

    c, s = math.cos(pitch), math.sin(pitch)
    x, z = c*x + s*z, -s*x + c*z

    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c*x - s*y, s*x + c*y
    return (x, y, z)


def transform(parent_pose, point):
    xyz = parent_pose[:3]
    rotated = rotate(point, parent_pose[3:])
    return tuple(xyz[i] + rotated[i] for i in range(3))


def prepare():
    if not SOURCE.is_file():
        raise RuntimeError('Missing detailed flat world')
    if TARGET.exists():
        raise RuntimeError('Refusing to overwrite existing test world')

    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')

    if world is None or world.get('name') != 'jarcrac_detailed_flat':
        raise RuntimeError('Wrong source world')

    vehicle = world.find("./model[@name='jarcrac']")
    if vehicle is None:
        raise RuntimeError('Vehicle missing')

    chassis = vehicle.find("./link[@name='chassis']")
    body = chassis.find("./collision[@name='body_collision']")
    sensor = chassis.find(
        "./sensor[@name='detailed_chassis_contact']"
    )

    if body is None or sensor is None:
        raise RuntimeError('Chassis collision or contact sensor missing')

    if (
        sensor.findtext('topic') != TOPIC
        or sensor.findtext('contact/collision') != 'body_collision'
    ):
        raise RuntimeError('Contact sensor configuration mismatch')

    if world.find(
        "./plugin[@name='gz::sim::systems::Contact']"
    ) is None:
        raise RuntimeError('World Contact system missing')

    size = tuple(
        float(v) for v in body.findtext('geometry/box/size').split()
    )
    if len(size) != 3 or min(size) <= 0:
        raise RuntimeError('Unexpected chassis collision geometry')

    # Compute the top face center in world coordinates.
    p_model = pose(vehicle.find('pose'))
    p_link = pose(chassis.find('pose'))
    p_body = pose(body.find('pose'))

    top_body = (0., 0., size[2] / 2)
    top_link = transform(p_body, top_body)
    top_model = transform(p_link, top_link)
    top_world = transform(p_model, top_model)

    # Freeze the vehicle and remove driving plugins only in this
    # disposable positive-contact test world.
    static = vehicle.find('static')
    if static is None:
        static = ET.SubElement(vehicle, 'static')
    static.text = 'true'

    for plugin in list(vehicle.findall('plugin')):
        if plugin.get('name') in (
            'gz::sim::systems::TrackedVehicle',
            'gz::sim::systems::TrackController'
        ):
            vehicle.remove(plugin)

    radius = 0.22
    mass = 2.0
    inertia = 0.4 * mass * radius * radius

    # Sphere begins above the top of the chassis and falls
    # under gravity. JARCRAC itself cannot move.
    sphere_pos = (
        top_world[0],
        top_world[1],
        top_world[2] + 0.55
    )

    ball = add(world, 'model', name='contact_test_ball')
    add(ball, 'pose', ' '.join(f'{x:.8f}' for x in (*sphere_pos, 0.0, 0.0, 0.0)))

    link = add(ball, 'link', name='ball_link')

    inertial = add(link, 'inertial')
    add(inertial, 'mass', mass)
    tensor = add(inertial, 'inertia')

    for axis in ('ixx', 'iyy', 'izz'):
        add(tensor, axis, inertia)

    for axis in ('ixy', 'ixz', 'iyz'):
        add(tensor, axis, 0)

    collision = add(link, 'collision', name='ball_collision')
    geometry = add(collision, 'geometry')
    add(add(geometry, 'sphere'), 'radius', radius)

    world.set('name', WORLD)

    ET.indent(tree, space='  ')
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    tree.write(TARGET, encoding='utf-8', xml_declaration=True)

    if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != digest:
        raise RuntimeError('Original world was modified')

    print('CONTACT POSITIVE FIXTURE GENERATED')
    print('World:', TARGET)
    print('Chassis top-center XYZ:', top_world)
    print('Test sphere XYZ:', sphere_pos)
    print('Test sphere mass:', mass, 'kg')
    print('JARCRAC: static')
    print('Velocity publishers: none')
    print('Original source: unchanged')


def monitor():
    from gz.transport13 import Node
    from gz.msgs10.contacts_pb2 import Contacts

    lock = threading.Lock()
    detected = threading.Event()

    state = {
        'messages': 0,
        'nonempty_messages': 0,
        'contact_pairs': [],
        'matched_contacts': 0
    }

    def callback(msg):
        with lock:
            state['messages'] += 1

            if len(msg.contact):
                state['nonempty_messages'] += 1

            for item in msg.contact:
                names = (
                    item.collision1.name,
                    item.collision2.name
                )

                if len(state['contact_pairs']) < 8:
                    state['contact_pairs'].append(list(names))

                if (
                    any('body_collision' in n for n in names)
                    and any('contact_test_ball' in n for n in names)
                ):
                    state['matched_contacts'] += 1
                    detected.set()

    node = Node()

    if not node.subscribe(Contacts, PUBLISHED_TOPIC, callback):
        raise RuntimeError('Contact subscription failed')

    print('Listening:', PUBLISHED_TOPIC, flush=True)
    print('Waiting for positive chassis–sphere contact...', flush=True)

    detected.wait(timeout=90)

    with lock:
        result = dict(state)

    result.update(
        world=WORLD,
        passed=result['matched_contacts'] > 0,
        mode='stationary_fixture',
        vehicle_driven=False,
        actual_driving_contact_validated=False
    )

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(result, indent=2) + '\n',
        encoding='utf-8'
    )

    print(json.dumps(result, indent=2))
    print(
        'CHASSIS CONTACT POSITIVE TEST:',
        'PASS' if result['passed'] else 'FAIL'
    )
    print('Report:', REPORT)

    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('prepare', 'monitor'))
    args = parser.parse_args()

    if args.action == 'prepare':
        prepare()
    else:
        monitor()
