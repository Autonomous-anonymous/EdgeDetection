#!/usr/bin/env python3
"""Remove JARCRAC geometry; retain its exact LiDAR mounting."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

source = (
    Path.home()
    / 'jarcrac_imu_experiment/jarcrac_detailed_flat.sdf'
)
target = (
    Path.home()
    / 'jarcrac_imu_experiment/jarcrac_detailed_lidar_isolated.sdf'
)

tree = ET.parse(source)
world = tree.getroot().find('world')

if world is None or world.get('name') != 'jarcrac_detailed_flat':
    raise RuntimeError('Unexpected source world')

vehicle = world.find("./model[@name='jarcrac']")
if vehicle is None:
    raise RuntimeError('Detailed JARCRAC model missing')

chassis = vehicle.find("./link[@name='chassis']")
lidar = chassis.find("./sensor[@name='gpu_lidar_front']")

if (
    lidar is None
    or lidar.get('type') != 'gpu_lidar'
    or lidar.findtext('ray/scan/horizontal/samples') != '721'
    or lidar.findtext('ray/scan/vertical/samples') != '32'
    or lidar.findtext('topic') != '/jarcrac/lidar'
):
    raise RuntimeError('Expected original 3D LiDAR missing')

rig = ET.Element('model', {'name': 'jarcrac_lidar_isolated'})
ET.SubElement(rig, 'static').text = 'true'

# Preserve the original model position.
model_pose = vehicle.find('pose')
if model_pose is not None:
    rig.append(deepcopy(model_pose))

# Preserve the chassis and LiDAR relative transforms.
# Do not copy any vehicle geometry or track controllers.
link = ET.SubElement(rig, 'link', {'name': 'chassis'})
link_pose = chassis.find('pose')
if link_pose is not None:
    link.append(deepcopy(link_pose))

link.append(deepcopy(lidar))

index = list(world).index(vehicle)
world.remove(vehicle)
world.insert(index, rig)
world.set('name', 'jarcrac_detailed_lidar_isolated')

ET.indent(tree, space='  ')
target.parent.mkdir(parents=True, exist_ok=True)
tree.write(
    target,
    encoding='utf-8',
    xml_declaration=True
)

# Confirm no JARCRAC drive links or track systems survived.
check = ET.parse(target).getroot().find('world')
models = check.findall("./model[@name='jarcrac_lidar_isolated']")

assert len(models) == 1
assert len(models[0].findall('link')) == 1
assert len(models[0].findall('plugin')) == 0
assert len(models[0].findall('.//sensor')) == 1
assert models[0].find("./link/sensor").get('name') == 'gpu_lidar_front'

print('ISOLATED LIDAR WORLD GENERATED')
print('World:', target)
print('LiDAR: 721 x 32; same mounting transform')
print('Vehicle body and tracks: removed')
print('Velocity controllers: removed')
print('Original files: unchanged')
