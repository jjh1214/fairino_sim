"""Tests of the STL reader and the MoveIt, Gazebo, TF and RViz message builders."""

import math
import os
import struct
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
from fairino_workcell import scene
from fairino_workcell.cell import load_cell
from fairino_workcell.meshes import MeshError, read_stl
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
import pytest
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

EXAMPLE = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell',
                       'cell.yaml')
SQUARE = [((0, 0, 0), (1, 0, 0), (1, 1, 0)), ((0, 0, 0), (1, 1, 0), (0, 1, 0))]


def write_binary_stl(path, triangles):
    data = bytearray(80) + struct.pack('<I', len(triangles))
    for triangle in triangles:
        data += struct.pack('<12fH', 0, 0, 1, *[c for corner in triangle for c in corner], 0)
    path.write_bytes(bytes(data))
    return str(path)


def write_ascii_stl(path, triangles):
    lines = ['solid square']
    for triangle in triangles:
        lines += ['facet normal 0 0 1', 'outer loop']
        lines += ['vertex {} {} {}'.format(*corner) for corner in triangle]
        lines += ['endloop', 'endfacet']
    path.write_text('\n'.join(lines + ['endsolid square']))
    return str(path)


@pytest.mark.parametrize('writer', [write_binary_stl, write_ascii_stl])
def test_read_stl_shares_vertices_and_scales(tmp_path, writer):
    vertices, triangles = read_stl(writer(tmp_path / 'square.stl', SQUARE), scale=0.001)
    assert vertices.shape == (4, 3) and triangles.shape == (2, 3)
    assert vertices.max() == pytest.approx(0.001)
    for triangle, expected in zip(triangles, SQUARE):
        corners = [tuple(vertices[i] * 1000) for i in triangle]
        assert corners == [tuple(map(float, c)) for c in expected]


def test_read_stl_errors(tmp_path):
    with pytest.raises(MeshError, match='Cannot read'):
        read_stl(str(tmp_path / 'missing.stl'))
    (tmp_path / 'junk.stl').write_bytes(b'\x00\x01junk')
    with pytest.raises(MeshError, match='not an STL'):
        read_stl(str(tmp_path / 'junk.stl'))


def test_quaternion_from_rpy():
    x, y, z, w = scene.quaternion_from_rpy(0, 0, math.pi / 2)
    assert (x, y, z, w) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert scene.quaternion_from_rpy(math.pi, 0, 0) == pytest.approx((1, 0, 0, 0), abs=1e-12)


def test_collision_objects_of_the_example_cell():
    cell = load_cell(EXAMPLE)
    objects = {obj.name: scene.collision_object(obj) for obj in cell.objects}
    pedestal = objects['pedestal']
    assert pedestal.primitives[0].type == SolidPrimitive.BOX
    assert list(pedestal.primitives[0].dimensions) == pytest.approx([0.3, 0.3, 0.75])
    assert pedestal.pose.position.z == pytest.approx(0.375)
    conveyor = objects['conveyor']
    assert conveyor.header.frame_id == 'world' and len(conveyor.meshes) == 1
    assert len(conveyor.meshes[0].triangles) > 100
    assert list(conveyor.subframe_names) == ['station']
    assert conveyor.subframe_poses[0].position.z == pytest.approx(0.8)
    assert max(v.z for v in conveyor.meshes[0].vertices) == pytest.approx(0.84, abs=1e-3)


def test_cylinder_dimensions_follow_solid_primitive_indices(tmp_path):
    (tmp_path / 'cell.yaml').write_text(
        'robot: {model: fairino5_v6}\n'
        'objects:\n  - {name: drum, cylinder: {radius: 0.2, length: 1.5}}\n')
    drum = scene.collision_object(load_cell(str(tmp_path / 'cell.yaml')).objects[0])
    primitive = drum.primitives[0]
    assert primitive.type == SolidPrimitive.CYLINDER
    assert primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT] == pytest.approx(1.5)
    assert primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] == pytest.approx(0.2)


def test_allow_collisions_extends_the_matrix_symmetrically():
    matrix = AllowedCollisionMatrix(
        entry_names=['base_link', 'j_link'],
        entry_values=[AllowedCollisionEntry(enabled=[False, True]),
                      AllowedCollisionEntry(enabled=[True, False])])
    matrix = scene.allow_collisions(matrix, [('pedestal', 'base_link')])
    names = list(matrix.entry_names)
    assert names == ['base_link', 'j_link', 'pedestal']
    table = [list(entry.enabled) for entry in matrix.entry_values]
    assert all(len(row) == 3 for row in table)
    assert table[2][0] and table[0][2]
    assert table[0][1] and not table[1][2]


def test_gazebo_sdf_and_markers():
    cell = load_cell(EXAMPLE)
    conveyor = cell.objects[1]
    model = ET.fromstring(scene.gazebo_sdf(conveyor)).find('model')
    assert model.get('name') == 'conveyor' and model.find('static').text == 'true'
    uris = [uri.text for uri in model.iter('uri')]
    assert len(uris) == 1 and uris[0].endswith('visual.glb')
    assert model.find('link/collision') is None     # no Gazebo contacts: the executor checks
    assert model.find('link/visual/material') is not None  # colour given in the cell file
    pedestal = ET.fromstring(scene.gazebo_sdf(cell.objects[0])).find('model')
    assert pedestal.find('link/visual/geometry/box/size').text == '0.3 0.3 0.75'

    array = scene.markers(cell)
    assert array.markers[0].action == Marker.DELETEALL
    assert len(array.markers) == len(cell.objects) + 1
    mesh_marker = array.markers[2]
    assert mesh_marker.type == Marker.MESH_RESOURCE
    assert mesh_marker.mesh_resource.startswith('file:///')
    assert not mesh_marker.mesh_use_embedded_materials
    assert array.markers[1].type == Marker.CUBE


def test_static_transforms():
    cell = load_cell(EXAMPLE)
    transforms = scene.static_transforms(cell, stamp=Time())
    children = [t.child_frame_id for t in transforms]
    assert children == cell.frame_names()
    station = transforms[children.index('conveyor/station')]
    assert station.header.frame_id == 'conveyor'
    assert station.transform.translation.z == pytest.approx(0.8)
