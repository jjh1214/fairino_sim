"""Tests of import_cad; skipped unless the dependencies of requirements-cad.txt are installed."""

import os

from fairino_workcell import cad_import
from fairino_workcell.cell import load_cell
import pytest

try:
    import trimesh
except ImportError:  # the CAD tool dependencies are optional at runtime
    trimesh = None

# A module-level pytest.importorskip() would hide every other test file from collection.
pytestmark = pytest.mark.skipif(
    trimesh is None, reason='CAD dependencies (requirements-cad.txt) are not installed')

EXAMPLE_CAD = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell', 'cad')


def write_box_stl(path, extents, centre=(0, 0, 0)):
    trimesh.creation.box(extents=extents).apply_translation(centre).export(path)
    return str(path)


def test_stl_in_millimetres_is_converted_to_metres(tmp_path):
    source = write_box_stl(tmp_path / 'block.stl', [400, 200, 100], [0, 0, 50])
    result = cad_import.convert(source, str(tmp_path / 'meshes'), units='mm')
    assert result.name == 'block'
    assert os.path.isfile(result.visual) and os.path.isfile(result.collision)
    assert result.info['size_m'] == pytest.approx([0.4, 0.2, 0.1])
    assert result.info['bounds_m']['min'][2] == pytest.approx(0.0)
    reloaded = trimesh.load(result.collision)
    assert reloaded.extents == pytest.approx([0.4, 0.2, 0.1])


def test_step_units_come_from_the_file(tmp_path):
    result = cad_import.convert(os.path.join(EXAMPLE_CAD, 'part.step'), str(tmp_path))
    assert result.info['size_m'] == pytest.approx([0.12, 0.08, 0.06], abs=1e-4)
    assert result.info['source_units'] == 'from the STEP file'


def test_origin_options(tmp_path):
    source = write_box_stl(tmp_path / 'b.stl', [2, 2, 2], [5, 5, 5])
    centre = cad_import.convert(source, str(tmp_path / 'c'), units='m', origin='center')
    assert centre.info['bounds_m']['min'] == pytest.approx([-1, -1, -1])
    bottom = cad_import.convert(source, str(tmp_path / 'b'), units='m', origin='bottom')
    assert bottom.info['bounds_m']['min'] == pytest.approx([-1, -1, 0])


def test_collision_simplification_modes(tmp_path):
    sphere = tmp_path / 'ball.stl'
    trimesh.creation.icosphere(subdivisions=4, radius=50).export(sphere)
    out = str(tmp_path / 'out')
    mesh = cad_import.convert(str(sphere), out, units='mm', max_faces=500)
    assert mesh.info['visual_faces'] > 500 >= mesh.info['collision_faces']
    box = cad_import.convert(str(sphere), out, name='ball_box', units='mm', collision='box')
    assert box.info['collision_faces'] == 12
    convex = cad_import.convert(str(sphere), out, name='ball_hull', units='mm',
                                collision='convex')
    assert 0 < convex.info['collision_faces'] <= convex.info['visual_faces']


@pytest.mark.parametrize('name, units, message', [
    ('block.stl', None, 'carry no units'),
    ('block.iges', 'mm', 'unsupported format'),
    ('missing.stl', 'mm', 'File not found'),
])
def test_errors(tmp_path, name, units, message):
    path = tmp_path / name
    if name != 'missing.stl':
        write_box_stl(tmp_path / 'block.stl', [1, 1, 1])
        if name != 'block.stl':
            path.write_text('not a mesh')
    with pytest.raises(cad_import.CadImportError, match=message):
        cad_import.convert(str(path), str(tmp_path / 'out'), units=units)


def test_object_names_and_cell_entry(tmp_path):
    assert cad_import.object_name('/x/Conveyor Line-2.STEP') == 'conveyor_line_2'
    assert cad_import.object_name('3d part.stl') == 'object_3d_part'
    source = write_box_stl(tmp_path / 'table.stl', [1, 1, 1])
    result = cad_import.convert(source, str(tmp_path / 'meshes'), units='m')
    entry = cad_import.cell_entry(result, str(tmp_path))
    assert 'visual: meshes/table/visual.glb' in entry and 'units: m' in entry


def test_printed_entries_load_into_a_cell_file(tmp_path):
    """What import_cad prints, pasted into cell.yaml, gives objects and part types."""
    table = cad_import.convert(write_box_stl(tmp_path / 'table.stl', [600, 600, 750],
                                             [0, 0, 375]),
                               str(tmp_path / 'meshes'), units='mm')
    part = cad_import.convert(os.path.join(EXAMPLE_CAD, 'part.step'), str(tmp_path / 'meshes'))
    (tmp_path / 'cell.yaml').write_text(
        'robot: {model: fairino5_v6}\n'
        'parts:\n' + cad_import.cell_entry(part, str(tmp_path), part=True)
        + 'objects:\n' + cad_import.cell_entry(table, str(tmp_path)))
    cell = load_cell(str(tmp_path / 'cell.yaml'))
    assert [obj.name for obj in cell.objects] == ['table']
    assert cell.objects[0].pose.xyz == (0, 0, 0)
    assert cell.objects[0].geometry.collision == str(tmp_path / 'meshes' / 'table' /
                                                     'collision.stl')
    assert cell.parts[0].name == 'part' and cell.parts[0].frame('grasp') is not None


def test_command_line(tmp_path, capsys):
    source = write_box_stl(tmp_path / 'plate.stl', [100, 100, 10])
    code = cad_import.main([source, '--units', 'mm', '--out', str(tmp_path / 'm'),
                            '--cell-dir', str(tmp_path)])
    assert code == 0
    assert 'name: plate' in capsys.readouterr().out
    assert cad_import.main([source, '--out', str(tmp_path / 'm')]) == 1
    assert cad_import.main([source, '--units', 'mm', '--out', str(tmp_path / 'm'),
                            '--part']) == 0
    printed = capsys.readouterr().out
    assert 'parts list' in printed and 'grasp:' in printed and 'pose:' not in printed
