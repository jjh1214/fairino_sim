"""
Load and validate a work cell file (cell.yaml).

A cell describes the robot (model, base pose, initial joint angles, tool) and the equipment
around it (meshes or primitive shapes, their poses and named frames). Relative paths are
resolved against the directory of the cell file.
"""

from dataclasses import dataclass
import math
import os
import re

from fairino_sim import description
import yaml

NAME_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
CELL_KEYS = ('cell', 'robot', 'objects')
ROBOT_KEYS = ('model', 'base', 'initial_positions', 'initial_positions_deg', 'tool')
OBJECT_KEYS = ('name', 'visual', 'collision', 'units', 'box', 'cylinder', 'pose', 'color',
               'frames', 'touch_links', 'movable')
VISUAL_FORMATS = ('.glb', '.gltf', '.dae', '.obj', '.stl')
EMBEDDED_MATERIAL_FORMATS = ('.glb', '.gltf', '.dae')
DEFAULT_COLOR = (0.62, 0.64, 0.68, 1.0)
WORLD_FRAME = 'world'


class CellError(RuntimeError):
    """The cell file is invalid."""


@dataclass(frozen=True)
class Pose:
    """Position (m) and roll, pitch, yaw (rad)."""

    xyz: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Geometry:
    """Shape of a cell object: a mesh pair, a box or a cylinder."""

    kind: str
    visual: str = ''
    collision: str = ''
    scale: float = 1.0
    size: tuple = ()

    @property
    def has_embedded_materials(self):
        return self.kind == 'mesh' and self.visual.lower().endswith(EMBEDDED_MATERIAL_FORMATS)


@dataclass(frozen=True)
class CellObject:
    """A piece of equipment in the cell."""

    name: str
    geometry: Geometry
    pose: Pose
    color: tuple
    color_given: bool
    frames: tuple
    touch_links: tuple
    movable: bool = False

    def frame(self, name):
        """Return the Pose of a named frame of the object, or None."""
        return dict(self.frames).get(name)


@dataclass(frozen=True)
class Robot:
    """Robot settings passed to fairino_sim."""

    model: str
    base: Pose
    initial_positions: tuple
    tool_file: str


@dataclass(frozen=True)
class Cell:
    """A loaded and validated work cell."""

    name: str
    path: str
    robot: Robot
    objects: tuple

    def frame_names(self):
        """Return every TF frame the cell publishes."""
        names = []
        for obj in self.objects:
            names.append(obj.name)
            names.extend(frame_id(obj.name, frame) for frame, _ in obj.frames)
        return names


def frame_id(object_name, frame_name):
    """Return the TF frame id of a named frame on an object."""
    return f'{object_name}/{frame_name}'


def load_cell(path):
    """Load and validate a cell file; raise CellError with the location of any problem."""
    path = os.path.abspath(os.path.expanduser(path))
    data = _read_yaml(path)
    base_dir = os.path.dirname(path)
    _check_keys(data, CELL_KEYS, path)
    robot = _load_robot(data.get('robot'), base_dir, path)
    links = description.robot_links(robot.model)

    items = data.get('objects') or []
    if not isinstance(items, list):
        raise CellError(f'{path}: objects must be a list')
    objects, names = [], set()
    for index, item in enumerate(items):
        obj = _load_object(item, index, base_dir, links, path)
        if obj.name in names:
            raise CellError(f'{path}: object name {obj.name!r} is used twice')
        names.add(obj.name)
        objects.append(obj)
    return Cell(name=str(data.get('cell') or os.path.basename(base_dir)), path=path,
                robot=robot, objects=tuple(objects))


def _read_yaml(path):
    try:
        with open(path) as stream:
            data = yaml.safe_load(stream)
    except OSError as exc:
        raise CellError(f'Cannot read cell file {path}: {exc.strerror}') from None
    except yaml.YAMLError as exc:
        raise CellError(f'Cell file {path} is not valid YAML: {exc}') from None
    if not isinstance(data, dict):
        raise CellError(f'Cell file {path} must contain a mapping')
    return data


def _check_keys(data, allowed, where):
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise CellError(f'{where}: unknown keys {unknown}; allowed keys are {list(allowed)}')


def _pose(data, what, where):
    try:
        xyz, rpy = description.parse_pose(data, what)
    except description.DescriptionError as exc:
        raise CellError(f'{where}: {exc}') from None
    return Pose(xyz=xyz, rpy=rpy)


def _vector(value, length, what, where):
    try:
        return description.parse_vector(value, length, what)
    except description.DescriptionError as exc:
        raise CellError(f'{where}: {exc}') from None


def _load_robot(data, base_dir, path):
    if not isinstance(data, dict) or 'model' not in data:
        raise CellError(f'{path}: robot.model is required')
    where = f'{path}: robot'
    _check_keys(data, ROBOT_KEYS, where)
    model = str(data['model'])
    try:
        description.check_model(model)
    except description.DescriptionError as exc:
        raise CellError(f'{where}: {exc}') from None

    if 'initial_positions' in data and 'initial_positions_deg' in data:
        raise CellError(f'{where}: give initial_positions or initial_positions_deg, not both')
    if 'initial_positions_deg' in data:
        degrees = _vector(data['initial_positions_deg'], 6, 'initial_positions_deg', where)
        initial = tuple(math.radians(v) for v in degrees)
    else:
        initial = _vector(data.get('initial_positions', description.DEFAULT_INITIAL_POSITIONS),
                          6, 'initial_positions', where)

    return Robot(model=model, base=_pose(data.get('base'), 'base', where),
                 initial_positions=initial,
                 tool_file=_tool_file(data.get('tool'), base_dir, where))


def _tool_file(value, base_dir, where):
    if value is None or str(value).strip().lower() in ('', 'none'):
        return ''
    value = os.path.expanduser(str(value).strip())
    if os.sep in value or value.endswith(('.yaml', '.yml')):
        value = os.path.join(base_dir, value)
    try:
        tool_file = description.resolve_tool_file(value)
        description.load_tool(tool_file)
    except description.DescriptionError as exc:
        raise CellError(f'{where}: tool: {exc}') from None
    return tool_file


def _load_object(item, index, base_dir, links, path):
    where = f'{path}: objects[{index}]'
    if not isinstance(item, dict):
        raise CellError(f'{where} must be a mapping')
    _check_keys(item, OBJECT_KEYS, where)
    name = item.get('name')
    if not isinstance(name, str) or not NAME_PATTERN.match(name):
        raise CellError(f'{where}: name must start with a letter and contain only letters, '
                        'digits and _')
    where = f'{path}: object {name!r}'
    if name in links:
        raise CellError(f'{where}: the name is already a frame of the robot')

    has_mesh = 'visual' in item or 'collision' in item
    shapes = [key for key in ('box', 'cylinder') if key in item]
    if len(shapes) + int(has_mesh) != 1:
        raise CellError(f'{where}: give exactly one shape: visual/collision meshes, box or '
                        'cylinder')
    if has_mesh:
        geometry = _mesh_geometry(item, base_dir, where)
    elif 'box' in item:
        size = _vector(item['box'], 3, 'box', where)
        if min(size) <= 0:
            raise CellError(f'{where}: box sizes must be positive (metres)')
        geometry = Geometry(kind='box', size=size)
    else:
        geometry = _cylinder_geometry(item['cylinder'], where)
    if 'units' in item and not has_mesh:
        raise CellError(f'{where}: units only applies to meshes (box and cylinder are in metres)')

    return CellObject(
        name=name,
        geometry=geometry,
        pose=_pose(item.get('pose'), 'pose', where),
        color=_color(item.get('color'), where),
        color_given='color' in item,
        frames=_frames(item.get('frames'), where),
        touch_links=_touch_links(item.get('touch_links'), links, where),
        movable=_flag(item.get('movable', False), 'movable', where),
    )


def _flag(value, what, where):
    if not isinstance(value, bool):
        raise CellError(f'{where}: {what} must be true or false')
    return value


def _mesh_geometry(item, base_dir, where):
    units = item.get('units')
    if units not in description.MESH_UNITS:
        raise CellError(f"{where}: units is required for meshes and must be 'm' or 'mm'")
    visual, collision = item.get('visual'), item.get('collision')
    if collision is None and not str(visual).lower().endswith('.stl'):
        raise CellError(f'{where}: a collision STL is required when the visual mesh is not an '
                        'STL (make one with import_cad)')
    # A single STL serves as both meshes; report problems under the key that was written.
    collision_path = _mesh_file(base_dir, collision if collision is not None else visual,
                                ('.stl',), 'collision' if collision is not None else 'visual',
                                where, ' (convert other formats with import_cad)')
    visual_path = (_mesh_file(base_dir, visual, VISUAL_FORMATS, 'visual', where)
                   if visual is not None else collision_path)
    return Geometry(kind='mesh', visual=visual_path, collision=collision_path,
                    scale=description.MESH_UNITS[units])


def _mesh_file(base_dir, value, formats, what, where, hint=''):
    path = os.path.normpath(os.path.join(base_dir, os.path.expanduser(str(value))))
    if not path.lower().endswith(formats):
        raise CellError(f'{where}: the {what} mesh must be one of {", ".join(formats)}{hint}')
    if not os.path.isfile(path):
        raise CellError(f'{where}: {what} mesh not found: {path}')
    return path


def _cylinder_geometry(data, where):
    if not isinstance(data, dict) or set(data) != {'radius', 'length'}:
        raise CellError(f'{where}: cylinder must be {{radius: r, length: l}} in metres')
    try:
        radius, length = float(data['radius']), float(data['length'])
    except (TypeError, ValueError):
        raise CellError(f'{where}: cylinder radius and length must be numbers') from None
    if radius <= 0 or length <= 0:
        raise CellError(f'{where}: cylinder radius and length must be positive')
    return Geometry(kind='cylinder', size=(radius, length))


def _color(value, where):
    if value is None:
        return DEFAULT_COLOR
    if not isinstance(value, (list, tuple)) or len(value) not in (3, 4):
        raise CellError(f'{where}: color must be [r, g, b] or [r, g, b, a] between 0 and 1')
    rgba = _vector(value, len(value), 'color', where) + ((1.0,) if len(value) == 3 else ())
    if not all(0.0 <= c <= 1.0 for c in rgba):
        raise CellError(f'{where}: color values must be between 0 and 1')
    return rgba


def _frames(value, where):
    if value is None:
        return ()
    if not isinstance(value, dict):
        raise CellError(f'{where}: frames must map frame names to poses')
    frames = []
    for name, pose in value.items():
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            raise CellError(f'{where}: frame name {name!r} must start with a letter and contain '
                            'only letters, digits and _')
        frames.append((name, _pose(pose, f'frames.{name}', where)))
    return tuple(frames)


def _touch_links(value, links, where):
    if value is None:
        return ()
    if isinstance(value, str) or not all(isinstance(v, str) for v in value):
        raise CellError(f'{where}: touch_links must be a list of robot link names')
    for link in value:
        if link not in links:
            raise CellError(f'{where}: touch link {link!r} is not a link of the robot')
    return tuple(value)
