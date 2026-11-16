"""Tests of the cell file loader (requires the built workspace to be sourced)."""

import math
import os

from fairino_workcell.cell import CellError, Conveyor, load_cell
import pytest

EXAMPLE = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell',
                       'cell.yaml')
STL = 'solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n' \
      'endloop\nendfacet\nendsolid t\n'


def write_cell(tmp_path, objects='', robot='robot: {model: fairino5_v6}\n'):
    (tmp_path / 'thing.stl').write_text(STL)
    (tmp_path / 'thing.glb').write_bytes(b'glTF')
    path = tmp_path / 'cell.yaml'
    path.write_text(robot + ('objects:\n' + objects if objects else ''))
    return str(path)


def test_example_cell():
    cell = load_cell(EXAMPLE)
    assert cell.name == 'conveyor_cell'
    assert cell.robot.model == 'fairino5_v6'
    assert cell.robot.base.xyz == (0.0, 0.0, 0.75)
    assert cell.robot.initial_positions[1] == pytest.approx(-math.pi / 2)
    assert cell.robot.tool_file.endswith(os.path.join('example_gripper', 'tool.yaml'))
    names = [obj.name for obj in cell.objects]
    assert names == ['pedestal', 'conveyor', 'table', 'fence']
    conveyor = cell.objects[1]
    assert conveyor.geometry.kind == 'mesh' and os.path.isfile(conveyor.geometry.collision)
    assert conveyor.geometry.has_embedded_materials and conveyor.color_given
    assert 'conveyor/station' in cell.frame_names()
    assert cell.objects[0].geometry.size == (0.3, 0.3, 0.75)
    assert cell.objects[0].touch_links == ('base_link', 'shoulder_link')
    assert conveyor.conveyor == Conveyor(part='part', entry='entry', station='station',
                                         speed=0.15, spacing=0.30, count=4)
    part, = cell.parts
    assert part.name == 'part' and part.movable and part.frame('grasp').xyz == (0.0, 0.0, 0.03)
    assert not any(obj.movable for obj in cell.objects)


def test_movable_objects_and_part_types(tmp_path):
    cell = load_cell(write_cell(tmp_path, (
        '  - {name: crate, box: [0.2, 0.2, 0.2], movable: true, '
        'frames: {grasp: {xyz: [0, 0, 0.2]}}}\n'
        '  - name: belt\n'
        '    box: [2, 0.4, 0.8]\n'
        '    frames: {a: {xyz: [-1, 0, 0.4]}, b: {xyz: [0.5, 0, 0.4]}}\n'
        '    conveyor: {part: box, entry: a, station: b, speed: 0.2, spacing: 0.3}\n'),
        robot='robot: {model: fairino5_v6}\n'
              'parts:\n  - {name: box, cylinder: {radius: 0.05, length: 0.1}, '
              'frames: {grasp: {xyz: [0, 0, 0.05]}}}\n'))
    crate, belt = cell.objects
    assert crate.movable and crate.conveyor is None
    assert belt.conveyor.count == 0 and belt.conveyor.speed == 0.2
    assert cell.parts[0].geometry.kind == 'cylinder' and cell.parts[0].pose.xyz == (0, 0, 0)


def test_shapes_units_and_defaults(tmp_path):
    cell = load_cell(write_cell(tmp_path, (
        '  - {name: block, box: [1, 2, 3]}\n'
        '  - {name: drum, cylinder: {radius: 0.2, length: 1.0}, '
        'pose: {xyz: [1, 0, 0], rpy_deg: [0, 0, 90]}}\n'
        '  - {name: mesh_mm, collision: thing.stl, units: mm, color: [1, 0, 0]}\n')))
    block, drum, mesh = cell.objects
    assert block.color[3] == 1.0 and not block.color_given
    assert drum.geometry.size == (0.2, 1.0)
    assert drum.pose.rpy[2] == pytest.approx(math.pi / 2)
    assert mesh.geometry.scale == 0.001
    assert mesh.geometry.visual == mesh.geometry.collision
    assert mesh.color == (1.0, 0.0, 0.0, 1.0)
    assert cell.robot.tool_file == ''
    assert cell.name == tmp_path.name


def test_tool_path_is_relative_to_the_cell_file(tmp_path):
    (tmp_path / 'tools').mkdir()
    (tmp_path / 'tools' / 'grip.stl').write_text(STL)
    (tmp_path / 'tools' / 'grip.yaml').write_text('mesh: {visual: grip.stl, units: mm}\n')
    cell = load_cell(write_cell(
        tmp_path, robot='robot: {model: fairino3_v6, tool: tools/grip.yaml}\n'))
    assert cell.robot.tool_file == str(tmp_path / 'tools' / 'grip.yaml')


@pytest.mark.parametrize('robot, objects, message', [
    ('robot: {}\n', '', 'robot.model is required'),
    ('robot: {model: fairino3_mt_v6}\n', '', 'Unsupported robot_name'),
    ('robot: {model: fairino5_v6, tool: nope}\n', '', 'Tool file not found'),
    ('robot: {model: fairino5_v6, initial_positions: [0, 0, 0, 0, 0, 0], '
     'initial_positions_deg: [0, 0, 0, 0, 0, 0]}\n', '', 'not both'),
    ('robot: {model: fairino5_v6}\nextra: 1\n', '', 'unknown keys'),
    (None, '  - {name: a, box: [1, 1, 1], visual: thing.stl, units: m}\n', 'exactly one shape'),
    (None, '  - {name: a, visual: thing.stl}\n', 'units is required'),
    (None, '  - {name: a, visual: thing.glb, units: m}\n', 'a collision STL is required'),
    (None, '  - {name: a, collision: missing.stl, units: m}\n', 'collision mesh not found'),
    (None, '  - {name: a, visual: missing.stl, units: m}\n', 'visual mesh not found'),
    (None, '  - {name: a, visual: thing.glb, collision: thing.glb, units: m}\n',
     'collision mesh must be one of .stl'),
    (None, '  - {name: a, box: [1, 1, 1]}\n  - {name: a, box: [1, 1, 1]}\n', 'used twice'),
    (None, '  - {name: base_link, box: [1, 1, 1]}\n', 'already a frame of the robot'),
    (None, '  - {name: 2bad, box: [1, 1, 1]}\n', 'name must start with a letter'),
    (None, '  - {name: a, box: [1, 1, 1], touch_links: [wing]}\n', 'not a link of the robot'),
    (None, '  - {name: a, box: [1, 1, 1], color: [2, 0, 0]}\n', 'between 0 and 1'),
    (None, '  - {name: a, box: [1, 1, 1], frames: [x]}\n', 'frames must map'),
    (None, '  - {name: a, box: [1, -1, 1]}\n', 'must be positive'),
    (None, '  - {name: a, cylinder: {radius: 1}}\n', 'cylinder must be'),
    (None, '  - {name: a, box: [1, 1, 1], units: mm}\n', 'units only applies to meshes'),
    (None, '  - {name: a, box: [1, 1, 1], pose: {xyz: [0, 0]}}\n', 'pose.xyz must be 3'),
    (None, '  - {name: a, box: [1, 1, 1], movable: 1}\n', 'movable must be true or false'),
])
def test_invalid_cells_raise_clear_errors(tmp_path, robot, objects, message):
    if robot is None:
        path = write_cell(tmp_path, objects)
    else:
        path = write_cell(tmp_path, objects, robot)
    with pytest.raises(CellError, match=message):
        load_cell(path)


PARTS = ('robot: {model: fairino5_v6}\n'
         'parts:\n  - {name: box, box: [0.1, 0.1, 0.1]}\n')
BELT = ('  - name: belt\n    box: [2, 0.4, 0.8]\n'
        '    frames: {a: {xyz: [-1, 0, 0.4]}, b: {xyz: [0.5, 0, 0.4]}}\n'
        '    conveyor: {%s}\n')
GOOD = 'part: box, entry: a, station: b, speed: 0.2, spacing: 0.3'


@pytest.mark.parametrize('robot, objects, message', [
    (PARTS, BELT % 'part: crate, entry: a, station: b, speed: 0.2, spacing: 0.3',
     "part 'crate' is not a part type"),
    (PARTS, BELT % 'part: box, entry: a, station: c, speed: 0.2, spacing: 0.3',
     "station 'c' is not a frame of the object"),
    (PARTS, BELT % 'part: box, entry: a, station: a, speed: 0.2, spacing: 0.3',
     'entry and station must be at different places'),
    (PARTS, BELT % 'part: box, entry: a, station: b, speed: 0, spacing: 0.3',
     'speed must be positive'),
    (PARTS, BELT % 'part: box, entry: a, station: b, speed: fast, spacing: 0.3',
     'speed must be a number'),
    (PARTS, BELT % (GOOD + ', count: -1'), 'count must be a whole number'),
    (PARTS, BELT % (GOOD + ', count: true'), 'count must be a whole number'),
    (PARTS, BELT % 'part: box, entry: a, station: b', 'speed, spacing required'),
    (PARTS, BELT % (GOOD + ', rate: 3'), 'unknown keys'),
    (PARTS, '  - {name: box, box: [1, 1, 1]}\n', 'both an object and a part type'),
    (PARTS, '  - {name: box_2, box: [1, 1, 1]}\n', "reserved for the parts of type 'box'"),
    (PARTS + '  - {name: box, box: [1, 1, 1]}\n', '', "part type 'box' is defined twice"),
    ('robot: {model: fairino5_v6}\n'
     'parts:\n  - {name: p, box: [1, 1, 1], pose: {xyz: [0, 0, 1]}}\n', '', 'unknown keys'),
    ('robot: {model: fairino5_v6}\nparts: {name: p}\n', '', 'parts must be a list'),
])
def test_invalid_parts_and_conveyors(tmp_path, robot, objects, message):
    with pytest.raises(CellError, match=message):
        load_cell(write_cell(tmp_path, objects, robot))


def test_unreadable_files(tmp_path):
    with pytest.raises(CellError, match='Cannot read cell file'):
        load_cell(str(tmp_path / 'missing.yaml'))
    bad = tmp_path / 'bad.yaml'
    bad.write_text('robot: [unclosed\n')
    with pytest.raises(CellError, match='not valid YAML'):
        load_cell(str(bad))
