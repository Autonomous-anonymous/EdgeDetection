#!/usr/bin/env python3
"""Create isolated +3-degree LiDAR yaw experiment."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

folder = Path.home() / 'jarcrac_imu_experiment'
source = folder / 'jarcrac_detailed_flat.sdf'
target = folder / 'jarcrac_detailed_lidar_yaw_plus3.sdf'

original_hash = hashlib.sha256(
    source.read_bytes()
).hexdigest()

tree = ET.parse(source)
world = tree.getroot().find('world')

if world is None or world.get('name') != 'jarcrac_detailed_flat':
    raise RuntimeError('Unexpected source world')

model = world.find("./model[@name='jarcrac']")
sensor = model.find(
    "./link[@name='chassis']/"
    "sensor[@name='gpu_lidar_front']"
)

if (
    sensor is None
    or sensor.findtext('ray/scan/horizontal/samples') != '721'
    or sensor.findtext('ray/scan/vertical/samples') != '32'
):
    raise RuntimeError('Expected 3D LiDAR missing')

pose = list(map(float, sensor.findtext('pose').split()))
if len(pose) != 6:
    raise RuntimeError('Invalid LiDAR pose')

original_yaw = pose[5]
pose[5] += math.radians(3.0)

sensor.find('pose').text = ' '.join(
    f'{v:.12g}' for v in pose
)

world.set('name', 'jarcrac_detailed_lidar_yaw_plus3')

ET.indent(tree, space='  ')
tree.write(
    target,
    encoding='utf-8',
    xml_declaration=True
)

check = ET.parse(target).getroot()
new_sensor = check.find(
    "./world/model[@name='jarcrac']/"
    "link[@name='chassis']/"
    "sensor[@name='gpu_lidar_front']"
)

assert new_sensor is not None
assert math.isclose(
    float(new_sensor.findtext('pose').split()[5])
    - original_yaw,
    math.radians(3),
    abs_tol=1e-8
)

assert hashlib.sha256(
    source.read_bytes()
).hexdigest() == original_hash

print('Generated:', target)
print('LiDAR yaw offset: +3 degrees')
print('Vehicle visuals: retained')
print('Vehicle collisions: unchanged')
print('Original model/world: unchanged')
