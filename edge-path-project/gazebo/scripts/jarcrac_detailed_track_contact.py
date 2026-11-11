#!/usr/bin/env python3
"""Positive track-ground contact test. No driving commands."""
import argparse
import json
import threading
import time
from pathlib import Path
import xml.etree.ElementTree as ET

SOURCE = (
    Path.home()
    / 'jarcrac_imu_experiment/jarcrac_detailed_flat.sdf'
)
WORLD = 'jarcrac_detailed_track_contact'
TARGET = SOURCE.parent / (WORLD + '.sdf')

REPORT = Path(
    'edge-path-project/gazebo/results/jarcrac/'
    'jarcrac_detailed_track_contact.json'
)

def topic(side):
    return (
        f'/world/{WORLD}/model/jarcrac/'
        f'link/track_{side}/sensor/'
        f'track_{side}_contact_probe/contact'
    )

def prepare():
    if TARGET.exists():
        raise SystemExit('Refusing to overwrite: ' + str(TARGET))

    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')

    if world is None or world.get('name') != 'jarcrac_detailed_flat':
        raise RuntimeError('Wrong source world')

    model = world.find("./model[@name='jarcrac']")
    ground = world.find("./model[@name='ground_plane']")

    if model is None or ground is None:
        raise RuntimeError('Vehicle or ground missing')

    if model.findtext('static', 'false') == 'true':
        raise RuntimeError('Vehicle unexpectedly static')

    if world.find(
        "./plugin[@name='gz::sim::systems::Contact']"
    ) is None:
        raise RuntimeError('Contact system missing')

    # Remove velocity-related systems from the disposable test only.
    for plugin in list(model.findall('plugin')):
        if plugin.get('name') in (
            'gz::sim::systems::TrackedVehicle',
            'gz::sim::systems::TrackController'
        ):
            model.remove(plugin)

    for side in ('left', 'right'):
        link = model.find(f"./link[@name='track_{side}']")

        if link is None:
            raise RuntimeError('Track link missing')

        collision = link.find(
            f"./collision[@name='track_{side}_collision']"
        )

        if collision is None:
            raise RuntimeError('Track collision missing')

        if collision.find('geometry/mesh') is None:
            raise RuntimeError('Original STL collision not present')

        sensor = ET.SubElement(
            link, 'sensor',
            name=f'track_{side}_contact_probe',
            type='contact'
        )

        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '100'

        contact = ET.SubElement(sensor, 'contact')
        ET.SubElement(contact, 'collision').text = (
            f'track_{side}_collision'
        )

    world.set('name', WORLD)

    ET.indent(tree, space='  ')
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    tree.write(
        TARGET, encoding='utf-8',
        xml_declaration=True
    )

    print('TRACK-GROUND TEST WORLD GENERATED')
    print('World:', TARGET)
    print('Track meshes: unchanged')
    print('Track sensors: left and right')
    print('Vehicle: dynamic, no drive plugins')
    print('Original source: unchanged')

def monitor():
    from gz.transport13 import Node
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V

    lock = threading.RLock()

    state = {
        'left': 0,
        'right': 0,
        'left_messages': 0,
        'right_messages': 0,
        'pairs': [],
        'first_vehicle_z': None,
        'last_vehicle_z': None,
        'sim_time_s': None,
    }

    def contact_callback(side):
        def callback(msg):
            with lock:
                state[f'{side}_messages'] += 1

                for c in msg.contact:
                    a = c.collision1.name
                    b = c.collision2.name

                    if len(state['pairs']) < 8:
                        state['pairs'].append([side, a, b])

                    expected = (
                        f'jarcrac::track_{side}::'
                        f'track_{side}_collision'
                    )

                    if (
                        (a == expected and 'ground_plane' in b)
                        or (b == expected and 'ground_plane' in a)
                    ):
                        state[side] += 1

        return callback

    def on_clock(msg):
        with lock:
            state['sim_time_s'] = (
                msg.sim.sec + msg.sim.nsec * 1e-9
            )

    def on_pose(msg):
        for p in msg.pose:
            if p.name.split('::')[-1] == 'jarcrac':
                with lock:
                    if state['first_vehicle_z'] is None:
                        state['first_vehicle_z'] = p.position.z
                    state['last_vehicle_z'] = p.position.z
                return

    node = Node()

    for side in ('left', 'right'):
        if not node.subscribe(
            Contacts, topic(side), contact_callback(side)
        ):
            raise RuntimeError(
                'Cannot subscribe: ' + topic(side)
            )

    if not node.subscribe(
        Clock, f'/world/{WORLD}/clock', on_clock
    ):
        raise RuntimeError('Clock subscription failed')

    if not node.subscribe(
        Pose_V, f'/world/{WORLD}/pose/info', on_pose
    ):
        raise RuntimeError('Pose subscription failed')

    print('READ-ONLY CONTACT MONITOR')
    print('Left:', topic('left'))
    print('Right:', topic('right'))
    print('Start Gazebo physics (Play) now.', flush=True)

    end_wall = time.monotonic() + 60
    start_sim = None

    while time.monotonic() < end_wall:
        with lock:
            s = dict(state)

        if s['left'] > 0 and s['right'] > 0:
            break

        if s['sim_time_s'] is not None:
            if start_sim is None:
                start_sim = s['sim_time_s']

            if s['sim_time_s'] - start_sim >= 3.0:
                break

        time.sleep(0.05)

    with lock:
        result = dict(state)

    result.update(
        test='detailed_track_ground_contact',
        passed=(
            result['left'] > 0
            and result['right'] > 0
        ),
        no_drive_commands=True,
        original_mesh_collisions_unchanged=True
    )

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(result, indent=2) + '\n'
    )

    print(json.dumps(result, indent=2))
    print(
        'TRACK-GROUND CONTACT TEST:',
        'PASS' if result['passed'] else 'NOT VERIFIED'
    )
    print('Report:', REPORT)

    if not result['passed']:
        raise SystemExit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'action', choices=('prepare', 'monitor')
    )
    args = parser.parse_args()

    if args.action == 'prepare':
        prepare()
    else:
        monitor()
