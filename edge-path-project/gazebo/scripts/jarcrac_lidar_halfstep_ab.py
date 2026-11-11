#!/usr/bin/env python3
"""Shift LiDAR sampling grid by +0.125 degrees, without moving sensor."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

folder = Path.home() / 'jarcrac_imu_experiment'
source = folder / 'jarcrac_detailed_flat.sdf'
target = folder / 'jarcrac_detailed_lidar_halfstep.sdf'

if not source.is_file():
    raise SystemExit(f'Missing source: {source}')

if target.exists():
    raise SystemExit(f'Refusing to overwrite: {target}')

before = hashlib.sha256(source.read_bytes()).hexdigest()

tree = ET.parse(source)
world = tree.getroot().find('world')

if world is None or world.get('name') != 'jarcrac_detailed_flat':
    raise RuntimeError('Unexpected source world')

sensor = world.find(
    "./model[@name='jarcrac']/"
    "link[@name='chassis']/"
    "sensor[@name='gpu_lidar_front']"
)

if sensor is None:
    raise RuntimeError('Original front LiDAR missing')

horizontal = sensor.find('ray/scan/horizontal')
vertical = sensor.find('ray/scan/vertical')

if horizontal is None or vertical is None:
    raise RuntimeError('Missing LiDAR scan configuration')

if (
    horizontal.findtext('samples') != '721'
    or vertical.findtext('samples') != '32'
):
    raise RuntimeError('Unexpected LiDAR resolution')

lo = float(horizontal.findtext('min_angle'))
hi = float(horizontal.findtext('max_angle'))

if (
    not math.isclose(lo, -math.pi/2, abs_tol=1e-5)
    or not math.isclose(hi, math.pi/2, abs_tol=1e-5)
):
    raise RuntimeError('Unexpected original horizontal FOV')

shift = math.radians(0.125)

horizontal.find('min_angle').text = f'{lo+shift:.12f}'
horizontal.find('max_angle').text = f'{hi+shift:.12f}'

# The sensor's physical pose is deliberately unchanged.
original_pose = sensor.findtext('pose')

world.set('name', 'jarcrac_detailed_lidar_halfstep')

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

assert new_sensor.findtext('pose') == original_pose
assert new_sensor.findtext(
    'ray/scan/horizontal/samples'
) == '721'
assert new_sensor.findtext(
    'ray/scan/vertical/samples'
) == '32'

new_h = new_sensor.find('ray/scan/horizontal')
new_lo = float(new_h.findtext('min_angle'))
new_hi = float(new_h.findtext('max_angle'))

angles = [
    math.degrees(new_lo + i*(new_hi-new_lo)/720)
    for i in range(721)
]

for seam in (-45., 45.):
    distance = min(abs(a-seam) for a in angles)
    if not .12 < distance < .13:
        raise RuntimeError(
            f'Sampling grid still intersects {seam} degrees'
        )
    print(
        f'Nearest sample to {seam:+.0f} deg: '
        f'{distance:.3f} deg away'
    )

if hashlib.sha256(source.read_bytes()).hexdigest() != before:
    raise RuntimeError('Source world changed')

print('HALF-STEP WORLD: PASS')
print('Generated:', target)
print('LiDAR mounting: UNCHANGED')
print('Horizontal samples: 721')
print('Vertical layers: 32')
print('Horizontal span: 180 degrees')
print('Angular grid offset: +0.125 degrees')
print('Vehicle geometry and physics: UNCHANGED')
