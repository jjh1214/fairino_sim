"""Unit tests for fairino_sim.description (requires the built workspace to be sourced)."""

import os
import xml.etree.ElementTree as ET

from fairino_sim import description as d
import pytest


def controllers_file():
    return os.path.join(d.share_dir('fairino_sim'), 'config', 'frcobot_controller2.yaml')


def joints(urdf):
    return {j.get('name'): j for j in ET.fromstring(urdf).findall('joint')}


def links(urdf):
    return {link.get('name') for link in ET.fromstring(urdf).findall('link')}


def write_tool(tmp_path, text, mesh=True):
    if mesh:
        (tmp_path / 'tool.stl').write_bytes(b'solid t\nendsolid t\n')
    path = tmp_path / 'tool.yaml'
    path.write_text(text)
    return str(path)


@pytest.mark.parametrize('model', d.SUPPORTED_MODELS)
def test_default_description_matches_previous_behaviour(model):
    urdf = d.build_urdf(model, controllers_file(), damping=300)
    root = ET.fromstring(urdf)
    assert root.get('name') == f'{model}_robot'
    assert 'package://' not in urdf
    assert 'damping="300"' in urdf
    initial = [p.text for p in root.iter('param') if p.get('name') == 'initial_value']
    assert [float(v) for v in initial] == list(d.DEFAULT_INITIAL_POSITIONS)
    origin = joints(urdf)['world_to_base_link'].find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == [0.0, 0.0, 0.0]
    assert 'tool_link' not in links(urdf)
    assert joints(urdf)['tcp_joint'].find('parent').get('link') == 'flange'


def test_flange_offset_comes_from_the_model_table():
    flange = joints(d.build_urdf('fairino20_v6', controllers_file()))['wrist3_to_flange']
    assert flange.find('origin').get('xyz').split()[2] == '0.12'


def test_base_pose_initial_positions_and_tool():
    tool = d.load_tool(d.resolve_tool_file('example_gripper'))
    urdf = d.build_urdf('fairino5_v6', controllers_file(), base_xyz='0.1 0 0.75',
                        base_rpy=[0, 0, 1.5708], initial_positions='0 -1.2 1.4 -1.8 -1.57 0',
                        tool=tool)
    origin = joints(urdf)['world_to_base_link'].find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == [0.1, 0.0, 0.75]
    assert [float(v) for v in origin.get('rpy').split()] == [0.0, 0.0, 1.5708]
    assert 'tool_link' in links(urdf)
    tcp = joints(urdf)['tcp_joint']
    assert tcp.find('parent').get('link') == 'tool_link'
    assert [float(v) for v in tcp.find('origin').get('xyz').split()] == [0.0, 0.0, 0.12]
    meshes = [m for m in ET.fromstring(urdf).iter('mesh')
              if 'example_gripper' in m.get('filename')]
    assert meshes and all(m.get('scale') == '0.001 0.001 0.001' for m in meshes)


def test_srdf_disables_tool_collisions_with_touch_links():
    tool = d.load_tool(d.resolve_tool_file('example_gripper'))
    root = ET.fromstring(d.build_srdf('fairino5_v6', tool))
    rules = [(e.get('link1'), e.get('link2')) for e in root.findall('disable_collisions')]
    assert ('tool_link', 'wrist3_link') in rules
    assert ('tool_link', 'wrist3_link') not in [
        (e.get('link1'), e.get('link2'))
        for e in ET.fromstring(d.build_srdf('fairino5_v6')).findall('disable_collisions')]


def test_tool_rpy_in_degrees(tmp_path):
    path = write_tool(tmp_path, 'mesh: {visual: tool.stl, units: m}\n'
                                'tcp: {xyz: [0, 0, 0.1], rpy_deg: [0, 90, 0]}\n')
    tool = d.load_tool(path)
    assert tool.tcp_rpy == pytest.approx((0.0, 1.5707963, 0.0))
    assert tool.scale == 1.0 and tool.touch_links == ('wrist3_link',)


@pytest.mark.parametrize('text, message', [
    ('mesh: {visual: tool.stl}\n', 'mesh.units is required'),
    ('mesh: {visual: tool.step, units: mm}\n', 'convert STEP files'),
    ('mesh: {visual: missing.stl, units: mm}\n', 'mesh file not found'),
    ('mesh: {visual: tool.stl, units: mm}\nmass: 0\n', 'mass must be a positive'),
    ('mesh: {visual: tool.stl, units: mm}\ntcp: {xyz: [0, 0]}\n', 'tcp.xyz must be 3'),
    ('mesh: {visual: tool.stl, units: mm}\nmounting: {}\n', 'unknown keys'),
])
def test_invalid_tool_files_raise_clear_errors(tmp_path, text, message):
    with pytest.raises(d.DescriptionError, match=message):
        d.load_tool(write_tool(tmp_path, text))


def test_unknown_touch_link_is_rejected(tmp_path):
    tool = d.load_tool(write_tool(tmp_path, 'mesh: {visual: tool.stl, units: mm}\n'
                                            'touch_links: [wrist9_link]\n'))
    with pytest.raises(d.DescriptionError, match='wrist9_link'):
        d.build_srdf('fairino5_v6', tool)


@pytest.mark.parametrize('call, message', [
    (lambda: d.check_model('fairino3_mt_v6'), 'Unsupported robot_name'),
    (lambda: d.build_urdf('fairino5_v6', controllers_file(), initial_positions='0 0 0'),
     'initial_positions must be 6 numbers'),
    (lambda: d.build_urdf('fairino5_v6', controllers_file(), base_xyz='0 0 x'),
     'base_xyz must be 3 numbers'),
    (lambda: d.resolve_tool_file('no_such_tool'), 'Tool file not found'),
])
def test_invalid_arguments_raise_clear_errors(call, message):
    with pytest.raises(d.DescriptionError, match=message):
        call()


def test_no_tool_arguments():
    assert d.resolve_tool_file('') is None
    assert d.resolve_tool_file('none') is None


def test_controller_and_package_names():
    assert d.trajectory_controller_name('fairino16_v6') == 'fairino16_controller'
    assert d.moveit_config_package('fairino3_v6') == 'fairino3_v6_moveit2_config'
