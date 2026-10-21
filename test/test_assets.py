from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ros2_amr_nmpc_telemetry.simulation import PlantConfig

ROOT = Path(__file__).resolve().parents[1]


def test_robot_geometry_agrees_with_simulator():
    config = PlantConfig()
    for name in ('amr.urdf', 'amr_gazebo.urdf'):
        root = ET.parse(ROOT/'urdf'/name).getroot()
        joints = {j.attrib['name']:j for j in root.findall('joint')}
        left = float(joints['left_wheel_joint'].find('origin').attrib['xyz'].split()[1])
        right = float(joints['right_wheel_joint'].find('origin').attrib['xyz'].split()[1])
        assert left-right == config.wheel_separation
        for link in ('left_wheel','right_wheel'):
            radius = root.find(f"link[@name='{link}']/collision/geometry/cylinder").attrib['radius']
            assert float(radius) == config.wheel_radius
        links = {link.attrib['name'] for link in root.findall('link')}
        for joint in joints.values():
            assert joint.find('parent').attrib['link'] in links
            assert joint.find('child').attrib['link'] in links


def test_gazebo_world_and_bridge_direction():
    assert ET.parse(ROOT/'worlds/warehouse.sdf').getroot().find('world').attrib['name']=='amr_lab'
    mappings = yaml.safe_load((ROOT/'config/gazebo_bridge.yaml').read_text())
    names = [m['ros_topic_name'] for m in mappings]
    assert len(names) == len(set(names))
    for m in mappings:
        assert m['direction'] == ('ROS_TO_GZ' if m['ros_topic_name']=='/gazebo/cmd_vel' else 'GZ_TO_ROS')
