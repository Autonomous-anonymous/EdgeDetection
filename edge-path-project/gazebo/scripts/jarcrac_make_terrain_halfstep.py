#!/usr/bin/env python3
"""Create isolated Ruskotunturi LiDAR half-step A/B world."""
from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET

folder = Path.home() / 'jarcrac_imu_experiment'
source = folder / 'jarcrac_detailed_terrain.sdf'
target = folder / 'jarcrac_detailed_terrain_halfstep.sdf'

if not source.is_file():
    raise SystemExit(f'Missing source: {source}')
if target.exists():
    raise SystemExit(f'Refusing to overwrite: {target}')

original_hash = hashlib.sha256(source.read_bytes()).hexdigest()

tree = ET.parse(source)
world = tree.getroot().find('world')

if world is None or world.get('name') != 'jarcrac_detailed_terrain':
    raise RuntimeError('Unexpected terrain world')

model = world.find("./model[@name='jarcrac']")
obstacle = world.find(
    "./model[@name='jarcrac_terrain_test_obstacle']"
)

if model is None or obstacle is None:
    raise RuntimeError('Vehicle or known obstacle missing')

sensor = model.find(
    "./link[@name='chassis']/"
    "sensor[@name='gpu_lidar_front']"
)

if sensor is None or sensor.get('type') != 'gpu_lidar':
    raise RuntimeError('Expected GPU LiDAR missing')

horizontal = sensor.find('ray/scan/horizontal')
vertical = sensor.find('ray/scan/vertical')

if (
    horizontal.findtext('samples') != '721'
    or vertical.findtext('samples') != '32'
):
    raise RuntimeError('Unexpected LiDAR resolution')

lo = float(horizontal.findtext('min_angle'))
hi = float(horizontal.findtext('max_angle'))

if not (
    math.isclose(lo, -math.pi/2, abs_tol=1e-5)
    and math.isclose(hi, math.pi/2, abs_tol=1e-5)
):
    raise RuntimeError('Unexpected original scan angles')

shift = math.radians(0.125)

horizontal.find('min_angle').text = f'{lo + shift:.12f}'
horizontal.find('max_angle').text = f'{hi + shift:.12f}'

world.set('name', 'jarcrac_detailed_terrain_halfstep')

ET.indent(tree, space='  ')
tree.write(
    target,
    encoding='utf-8',
    xml_declaration=True
)

# Confirm original file was not modified.
assert hashlib.sha256(source.read_bytes()).hexdigest() == original_hash

# Confirm the output has the intended scan grid.
check = ET.parse(target).getroot()
updated = check.find(
    "./world/model[@name='jarcrac']/"
    "link[@name='chassis']/"
    "sensor[@name='gpu_lidar_front']"
)

angles = updated.find('ray/scan/horizontal')
new_lo = float(angles.findtext('min_angle'))
new_hi = float(angles.findtext('max_angle'))

for seam in (-45., 45.):
    nearest = min(
        abs(math.degrees(
            new_lo + i*(new_hi-new_lo)/720
        ) - seam)
        for i in range(721)
    )
    if not .12 < nearest < .13:
        raise RuntimeError('Sampling grid intersects seam')

print('TERRAIN HALF-STEP WORLD: PASS')
print('Generated:', target)
print('LiDAR angular offset: +0.125 degrees')
print('Known obstacle: retained')
print('Detailed track geometry: unchanged')
print('Original files: unchanged')
