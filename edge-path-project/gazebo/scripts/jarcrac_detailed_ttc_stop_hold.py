#!/usr/bin/env python3
"""Detailed-track adapter for the existing experimental TTC/hold test.

Reuses the validated TTC logic and known-static-obstacle map classifier.
Changes only the isolated world, command topic and odometry subscription.
Simulation-only. No independent watchdog or real-world safety assurance.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import jarcrac_adas_ttc_stop_hold as base


NAME = 'jarcrac_detailed_ttc_stop_hold'
OUTPUT = Path.home() / 'jarcrac_imu_experiment' / (NAME + '.sdf')
SOURCE = (
    Path.home() / 'jarcrac_imu_experiment'
    / 'jarcrac_detailed_terrain.sdf'
)

CMD = '/model/jarcrac/detailed_ttc_cmd_vel'
ODOM = '/model/jarcrac/detailed_odometry'
CONTACT = '/jarcrac/detailed/chassis_contacts'

# All assignments are local to this Python process.
# Original TTC and tracked model files are unchanged.
base.WORLD_NAME = NAME
base.WORLD_PATH = OUTPUT
base.POSE_TOPIC = f'/world/{NAME}/pose/info'
base.CLOCK_TOPIC = f'/world/{NAME}/clock'
base.CMD_TOPIC = CMD
base.CONTACT_TOPIC = CONTACT
base.DEFAULT_REPORT = Path(
    '/tmp/jarcrac_detailed_ttc_stop_hold.json'
)


class OdometryAdapterNode:
    """Route the original monitor's odometry subscription to tracks."""

    def __init__(self, inner):
        self.inner = inner

    def subscribe(self, message_type, topic, callback):
        if topic == '/model/jarcrac/odometry':
            topic = ODOM
        return self.inner.subscribe(message_type, topic, callback)

    def advertise(self, topic, message_type):
        return self.inner.advertise(topic, message_type)


OriginalTTCMonitor = base.TTCMonitor


class DetailedTTCMonitor(OriginalTTCMonitor):
    def __init__(self, node, msgs, *, every_scans):
        super().__init__(
            OdometryAdapterNode(node),
            msgs,
            every_scans=every_scans
        )


base.TTCMonitor = DetailedTTCMonitor


def prepare_world(output=OUTPUT):
    if not SOURCE.is_file():
        raise FileNotFoundError(
            'Generate detailed terrain world first: ' + str(SOURCE)
        )

    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')

    if world is None or world.get('name') != 'jarcrac_detailed_terrain':
        raise RuntimeError('Unexpected source world')

    model = world.find("./model[@name='jarcrac']")
    obstacle = world.find(
        f"./model[@name='{base.KNOWN_OBSTACLE}']"
    )

    if model is None or obstacle is None:
        raise RuntimeError('Vehicle or known obstacle missing')

    if obstacle.findtext('static') != 'true':
        raise RuntimeError('Known obstacle is not static')

    obstacle_xy = tuple(
        map(float, obstacle.findtext('pose').split()[:2])
    )
    if math.dist(obstacle_xy, base.OBSTACLE_XY) > .02:
        raise RuntimeError('Known obstacle moved')

    spawn = tuple(
        map(float, model.findtext('pose').split()[:2])
    )
    if math.dist(spawn, base.EXPECTED_START_XY) > .02:
        raise RuntimeError('Unexpected vehicle spawn')

    tracked = model.find(
        "./plugin[@name='gz::sim::systems::TrackedVehicle']"
    )
    if tracked is None:
        raise RuntimeError('Native tracked-vehicle plugin missing')

    if tracked.findtext('topic') != \
            '/model/jarcrac/detailed_cmd_vel':
        raise RuntimeError('Unexpected original command topic')

    if tracked.findtext('odom_topic') != ODOM:
        raise RuntimeError('Unexpected track odometry topic')

    lidar = model.find(
        "./link[@name='chassis']/"
        "sensor[@name='gpu_lidar_front']"
    )
    if (
        lidar is None
        or lidar.findtext('topic') != '/jarcrac/lidar'
        or lidar.findtext('ray/scan/horizontal/samples') != '721'
        or lidar.findtext('ray/scan/vertical/samples') != '32'
    ):
        raise RuntimeError('Expected 721x32 front LiDAR')

    chassis = model.find("./link[@name='chassis']")
    if chassis.findtext(
        "./sensor[@type='imu']/topic"
    ) != '/jarcrac/imu':
        raise RuntimeError('IMU missing')

    if chassis.findtext(
        "./sensor[@type='contact']/topic"
    ) != CONTACT:
        raise RuntimeError('Contact sensor missing')

    controllers = [
        p for p in model.findall('plugin')
        if p.get('name') == 'gz::sim::systems::TrackController'
    ]
    if len(controllers) != 2:
        raise RuntimeError('Expected two track controllers')

    if any(
        p.findtext('max_command_age') != '0.3'
        for p in controllers
    ):
        raise RuntimeError('Track command timeout missing')

    # Only change the detailed test world's velocity topic.
    tracked.find('topic').text = CMD
    world.set('name', NAME)

    ET.indent(tree, space='  ')
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(
        output,
        encoding='utf-8',
        xml_declaration=True
    )

    print('Generated isolated detailed TTC world:', output)
    print('Command topic:', CMD)
    print('Odometry topic:', ODOM)
    print('Original vehicle and TTC controller unchanged.')
    return output


base.prepare_world = prepare_world


if __name__ == '__main__':
    base.main()
