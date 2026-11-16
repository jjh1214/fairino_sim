"""
Convert CAD files (STEP, STL, OBJ) into the meshes used by a fairino_workcell cell.

For every input file it writes, in metres:
  <out>/<name>/visual.glb      full-detail mesh for Gazebo and RViz (keeps STEP colours)
  <out>/<name>/collision.stl   simplified mesh for MoveIt collision checking and Gazebo
  <out>/<name>/info.yaml       source, units, size and triangle counts
and prints the matching objects entry for cell.yaml.

The tool does not need ROS. Install its dependencies once:
  pip install -r fairino_workcell/requirements-cad.txt
"""

import argparse
from dataclasses import dataclass
import os
import re
import sys

import yaml

UNITS = {'m': 1.0, 'cm': 0.01, 'mm': 0.001, 'in': 0.0254}
STEP_FORMATS = ('.step', '.stp')
MESH_FORMATS = ('.stl', '.obj')
COLLISION_MODES = ('mesh', 'convex', 'box')
ORIGIN_MODES = ('keep', 'center', 'bottom')


class CadImportError(RuntimeError):
    """A CAD file cannot be converted."""


@dataclass(frozen=True)
class Converted:
    """Result of converting one CAD file."""

    name: str
    visual: str
    collision: str
    info: dict


def _trimesh():
    try:
        import trimesh
    except ImportError:
        raise CadImportError(
            'trimesh is not installed. Install the CAD tool dependencies with: '
            'pip install -r fairino_workcell/requirements-cad.txt') from None
    return trimesh


def object_name(path):
    """Return a cell object name derived from a file name."""
    stem = os.path.splitext(os.path.basename(path))[0].lower()
    name = re.sub(r'[^a-z0-9_]+', '_', stem).strip('_')
    if not name or not name[0].isalpha():
        name = 'object_' + name
    return name


def load_scene(path, units=None):
    """Load a CAD file as a trimesh Scene in metres; units apply to STL and OBJ only."""
    trimesh = _trimesh()
    extension = os.path.splitext(path)[1].lower()
    if not os.path.isfile(path):
        raise CadImportError(f'File not found: {path}')
    if extension in STEP_FORMATS:
        try:
            import cascadio  # noqa: F401  (trimesh reads STEP through cascadio)
        except ImportError:
            raise CadImportError(
                'Reading STEP files needs cascadio: pip install cascadio') from None
        scale = 1.0  # STEP files carry their units; trimesh returns metres
    elif extension in MESH_FORMATS:
        if units is None:
            raise CadImportError(
                f'{path}: STL and OBJ files carry no units; pass --units (m, cm, mm or in)')
        scale = UNITS[units]
    else:
        raise CadImportError(f'{path}: unsupported format {extension!r}; use STEP, STL or OBJ')

    try:
        loaded = trimesh.load(path)
    except Exception as exc:  # trimesh and its loaders raise many exception types
        raise CadImportError(f'{path}: cannot be read: {exc}') from exc
    scene = loaded if isinstance(loaded, trimesh.Scene) else trimesh.Scene(loaded)
    if not scene.geometry:
        raise CadImportError(f'{path}: contains no geometry')
    return scene.scaled(scale) if scale != 1.0 else scene


def move_origin(scene, mode):
    """Move the origin to the CAD origin (keep), the bounding box centre or its bottom centre."""
    if mode not in ORIGIN_MODES:
        raise CadImportError(f'origin must be one of {", ".join(ORIGIN_MODES)}')
    if mode == 'keep':
        return (0.0, 0.0, 0.0)
    low, high = scene.bounds
    offset = -(low + high) / 2.0
    if mode == 'bottom':
        offset[2] = -low[2]
    scene.apply_translation(offset)
    return tuple(round(float(v), 6) for v in offset)


def _as_mesh(scene):
    try:
        return scene.to_mesh()
    except AttributeError:  # trimesh < 4
        return scene.dump(concatenate=True)


def collision_mesh(scene, mode='mesh', max_faces=3000):
    """Return the collision mesh: the decimated mesh, its convex hull or its bounding box."""
    if mode not in COLLISION_MODES:
        raise CadImportError(f'collision must be one of {", ".join(COLLISION_MODES)}')
    mesh = _as_mesh(scene)
    mesh.merge_vertices()
    if mode == 'convex':
        return mesh.convex_hull
    if mode == 'box':
        return mesh.bounding_box.to_mesh()
    if len(mesh.faces) > max_faces:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
        except ImportError:
            raise CadImportError(
                'Simplifying meshes needs fast-simplification: '
                'pip install fast-simplification') from None
    return mesh


def convert(path, out_dir, name=None, units=None, origin='keep', collision='mesh',
            max_faces=3000):
    """Convert one CAD file and write visual.glb, collision.stl and info.yaml."""
    name = name or object_name(path)
    scene = load_scene(path, units)
    offset = move_origin(scene, origin)
    mesh = collision_mesh(scene, collision, max_faces)
    if len(mesh.faces) == 0:
        raise CadImportError(f'{path}: the collision mesh is empty')

    target = os.path.join(out_dir, name)
    os.makedirs(target, exist_ok=True)
    visual_path = os.path.join(target, 'visual.glb')
    collision_path = os.path.join(target, 'collision.stl')
    scene.export(visual_path, file_type='glb')
    mesh.export(collision_path, file_type='stl')

    low, high = scene.bounds
    is_step = os.path.splitext(path)[1].lower() in STEP_FORMATS
    info = {
        'source': os.path.relpath(os.path.abspath(path), os.path.abspath(target)),
        'source_units': 'from the STEP file' if is_step else units,
        'origin': origin,
        'origin_offset_m': list(offset),
        'size_m': [round(float(v), 4) for v in high - low],
        'bounds_m': {'min': [round(float(v), 4) for v in low],
                     'max': [round(float(v), 4) for v in high]},
        'visual_faces': int(sum(len(g.faces) for g in scene.geometry.values()
                                if hasattr(g, 'faces'))),
        'collision': collision,
        'collision_faces': int(len(mesh.faces)),
    }
    with open(os.path.join(target, 'info.yaml'), 'w') as stream:
        yaml.safe_dump(info, stream, sort_keys=False)
    return Converted(name=name, visual=visual_path, collision=collision_path, info=info)


def cell_entry(converted, cell_dir='.'):
    """Return the cell.yaml objects entry for a converted file."""
    def relative(path):
        return os.path.relpath(path, cell_dir)

    return (f'  - name: {converted.name}\n'
            f'    visual: {relative(converted.visual)}\n'
            f'    collision: {relative(converted.collision)}\n'
            f'    units: m\n'
            f'    pose: {{xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}}\n')


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='import_cad',
        description='Convert STEP, STL and OBJ files into fairino_workcell meshes (metres).')
    parser.add_argument('inputs', nargs='+', help='STEP (.step/.stp), STL or OBJ files')
    parser.add_argument('-o', '--out', required=True,
                        help='output directory, usually <cell directory>/meshes')
    parser.add_argument('--units', choices=sorted(UNITS),
                        help='units of STL and OBJ files (STEP files carry their own)')
    parser.add_argument('--name', help='object name, for a single input (default: file name)')
    parser.add_argument('--origin', choices=ORIGIN_MODES, default='keep',
                        help='keep the CAD origin (default), or move it to the bounding box '
                             'centre or bottom centre')
    parser.add_argument('--collision', choices=COLLISION_MODES, default='mesh',
                        help='collision shape: simplified mesh (default), convex hull or box')
    parser.add_argument('--max-faces', type=int, default=3000,
                        help='triangle budget of the simplified collision mesh (default 3000)')
    parser.add_argument('--cell-dir', default='.',
                        help='directory of cell.yaml; printed paths are relative to it')
    args = parser.parse_args(argv)
    if args.name and len(args.inputs) > 1:
        parser.error('--name works with a single input file')

    entries = []
    try:
        for path in args.inputs:
            result = convert(path, args.out, args.name, args.units, args.origin,
                             args.collision, args.max_faces)
            info = result.info
            print(f'{path} -> {os.path.dirname(result.visual)}  size {info["size_m"]} m, '
                  f'{info["visual_faces"]} visual / {info["collision_faces"]} collision '
                  'triangles', file=sys.stderr)
            entries.append(cell_entry(result, args.cell_dir))
    except CadImportError as exc:
        print(f'import_cad: error: {exc}', file=sys.stderr)
        return 1
    print('# Add to the objects list of cell.yaml, then set each pose:')
    print(''.join(entries), end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
