"""
Build the simulated FAIRINO robot description shared by Gazebo, MoveIt and RViz.

The launch files use this module so that robot_state_publisher, Gazebo, move_group and
RViz all receive the same URDF and SRDF, including the optional tool on the flange.
"""

from dataclasses import dataclass
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import xacro
import yaml

SUPPORTED_MODELS = (
    'fairino3_v6',
    'fairino5_v6',
    'fairino10_v6',
    'fairino16_v6',
    'fairino20_v6',
    'fairino30_v6',
)
DEFAULT_INITIAL_POSITIONS = (0.0, -1.571, 0.0, 0.0, 0.0, 0.0)
MESH_UNITS = {'m': 1.0, 'mm': 0.001}
MESH_EXTENSIONS = ('.stl', '.obj', '.dae', '.glb', '.gltf')
TOOL_KEYS = {'name', 'mesh', 'mount', 'tcp', 'mass', 'touch_links', 'fingers'}
FINGER_KEYS = {'mesh', 'origin', 'stroke', 'speed', 'mass'}
FINGER_LINKS = ('finger_left', 'finger_right')        # on the +y and the -y side of the tool
FINGER_JOINTS = ('finger_left_joint', 'finger_right_joint')
GRIPPER_CONTROLLER = 'gripper_controller'
ADDED_LINKS = ('world', 'flange', 'tool_link', 'tcp') + FINGER_LINKS


class DescriptionError(RuntimeError):
    """The robot description cannot be built from the given inputs."""


@dataclass(frozen=True)
class Fingers:
    """
    The two fingers of a parallel gripper, which close along the y axis of the tool.

    The finger mesh is the finger on the +y side in its own frame: its gripping face lies in the
    plane y = 0 and it extends to +y. The finger frame sits at origin_xyz on the tool when the
    gripper is closed; each finger joint is the distance of its gripping face from the tool's xz
    plane, from 0 (closed) to stroke (open). The other finger is the same mesh turned 180
    degrees about z.
    """

    visual: str
    collision: str
    scale: float
    origin_xyz: tuple
    stroke: float        # m each finger travels
    speed: float         # m/s of each finger
    mass: float


@dataclass(frozen=True)
class Tool:
    """Tool mounted on the robot flange, as read from a tool YAML file."""

    name: str
    visual: str
    collision: str
    scale: float
    mount_xyz: tuple
    mount_rpy: tuple
    tcp_xyz: tuple
    tcp_rpy: tuple
    mass: float
    touch_links: tuple
    fingers: Fingers = None


def share_dir(package):
    """Return the share directory of a package, with a readable error if it is missing."""
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise DescriptionError(
            f"ROS package '{package}' was not found. Build it in the workspace and "
            'source install/setup.bash.') from None


def moveit_config_package(robot_name):
    """Return the name of the MoveIt config package of a robot model."""
    return f'{robot_name}_moveit2_config'


def trajectory_controller_name(robot_name):
    """Return the joint trajectory controller name used by the MoveIt config package."""
    return robot_name.rsplit('_', 1)[0] + '_controller'


def check_model(robot_name, moveit=False):
    """Raise DescriptionError unless robot_name is a supported model."""
    if robot_name not in SUPPORTED_MODELS:
        raise DescriptionError(
            f"Unsupported robot_name '{robot_name}'. "
            f"Supported models: {', '.join(SUPPORTED_MODELS)}")
    share_dir('fairino_description')
    if moveit:
        share_dir(moveit_config_package(robot_name))


def parse_vector(value, length, what):
    """Parse 'x y z' text or a list of numbers into a tuple of floats of a given length."""
    items = value.split() if isinstance(value, str) else value
    try:
        vector = tuple(float(v) for v in items)
    except (TypeError, ValueError):
        raise DescriptionError(f'{what} must be {length} numbers, got {value!r}') from None
    if len(vector) != length:
        raise DescriptionError(f'{what} must be {length} numbers, got {value!r}')
    return vector


def _format(vector):
    return ' '.join(repr(float(v)) for v in vector)


def parse_pose(data, what):
    """Read {xyz: [m], rpy: [rad]} or {xyz, rpy_deg: [deg]} into (xyz, rpy) tuples."""
    if data is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    if not isinstance(data, dict):
        raise DescriptionError(f'{what} must be a mapping with xyz and rpy (or rpy_deg)')
    unknown = set(data) - {'xyz', 'rpy', 'rpy_deg'}
    if unknown:
        raise DescriptionError(f'{what}: unknown keys {sorted(unknown)}')
    if 'rpy' in data and 'rpy_deg' in data:
        raise DescriptionError(f'{what}: give either rpy or rpy_deg, not both')
    xyz = parse_vector(data.get('xyz', (0, 0, 0)), 3, f'{what}.xyz')
    if 'rpy_deg' in data:
        degrees = parse_vector(data['rpy_deg'], 3, f'{what}.rpy_deg')
        rpy = tuple(math.radians(v) for v in degrees)
    else:
        rpy = parse_vector(data.get('rpy', (0, 0, 0)), 3, f'{what}.rpy')
    return xyz, rpy


def resolve_tool_file(tool):
    """Return the tool YAML path for a launch argument: '' or 'none', a tool name, or a path."""
    tool = (tool or '').strip()
    if not tool or tool.lower() == 'none':
        return None
    if os.sep in tool or tool.endswith(('.yaml', '.yml')):
        path = os.path.abspath(os.path.expanduser(tool))
    else:
        path = os.path.join(share_dir('fairino_sim'), 'tools', tool, 'tool.yaml')
    if not os.path.isfile(path):
        raise DescriptionError(f'Tool file not found: {path}')
    return path


def _mesh_path(base_dir, relative, tool_file):
    path = os.path.normpath(os.path.join(base_dir, os.path.expanduser(str(relative))))
    if not path.lower().endswith(MESH_EXTENSIONS):
        raise DescriptionError(
            f'{tool_file}: mesh {relative} must be one of {", ".join(MESH_EXTENSIONS)} '
            '(convert STEP files to a mesh first)')
    if not os.path.isfile(path):
        raise DescriptionError(f'{tool_file}: mesh file not found: {path}')
    return path


def _positive(data, key, default, what):
    try:
        value = float(data.get(key, default))
    except (TypeError, ValueError):
        value = -1.0
    if value <= 0.0:
        raise DescriptionError(f'{what}: {key} must be a positive number')
    return value


def _meshes(mesh, base_dir, path, what):
    """Return (visual, collision, scale) of a mesh mapping of a tool file."""
    if not isinstance(mesh, dict) or 'visual' not in mesh:
        raise DescriptionError(f'Tool file {path}: {what}.visual is required')
    units = mesh.get('units')
    if units not in MESH_UNITS:
        raise DescriptionError(
            f"Tool file {path}: {what}.units is required and must be 'm' or 'mm' "
            '(STL and OBJ files carry no units)')
    visual = _mesh_path(base_dir, mesh['visual'], path)
    collision = _mesh_path(base_dir, mesh.get('collision', mesh['visual']), path)
    return visual, collision, MESH_UNITS[units]


def _load_fingers(data, base_dir, path):
    where = f'Tool file {path}: fingers'
    if not isinstance(data, dict):
        raise DescriptionError(f'{where} must be a mapping')
    unknown = set(data) - FINGER_KEYS
    if unknown:
        raise DescriptionError(f'{where}: unknown keys {sorted(unknown)}')
    for key in ('mesh', 'stroke', 'speed'):
        if key not in data:
            raise DescriptionError(f'{where}: {key} is required')
    visual, collision, scale = _meshes(data['mesh'], base_dir, path, 'fingers.mesh')
    origin = data.get('origin') or {}
    if not isinstance(origin, dict) or set(origin) - {'xyz'}:
        raise DescriptionError(f'{where}: origin must be {{xyz: [x, y, z]}} in metres')
    return Fingers(visual=visual, collision=collision, scale=scale,
                   origin_xyz=parse_vector(origin.get('xyz', (0, 0, 0)), 3,
                                           'fingers.origin.xyz'),
                   stroke=_positive(data, 'stroke', None, where),
                   speed=_positive(data, 'speed', None, where),
                   mass=_positive(data, 'mass', 0.05, where))


def load_tool(path):
    """Load and validate a tool YAML file. Mesh paths are relative to the YAML file."""
    try:
        with open(path) as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise DescriptionError(f'Cannot read tool file {path}: {exc}') from None
    if not isinstance(data, dict):
        raise DescriptionError(f'Tool file {path} must contain a mapping')
    unknown = set(data) - TOOL_KEYS
    if unknown:
        raise DescriptionError(f'Tool file {path}: unknown keys {sorted(unknown)}')

    base_dir = os.path.dirname(os.path.abspath(path))
    visual, collision, scale = _meshes(data.get('mesh'), base_dir, path, 'mesh')

    mount_xyz, mount_rpy = parse_pose(data.get('mount'), 'mount')
    tcp_xyz, tcp_rpy = parse_pose(data.get('tcp'), 'tcp')
    try:
        mass = float(data.get('mass', 0.5))
    except (TypeError, ValueError):
        mass = -1.0
    if mass <= 0.0:
        raise DescriptionError(f'Tool file {path}: mass must be a positive number (kg)')
    touch_links = data.get('touch_links', ['wrist3_link'])
    if isinstance(touch_links, str) or not all(isinstance(v, str) for v in touch_links):
        raise DescriptionError(f'Tool file {path}: touch_links must be a list of link names')

    fingers = _load_fingers(data['fingers'], base_dir, path) if 'fingers' in data else None

    return Tool(
        name=str(data.get('name', os.path.basename(base_dir))),
        visual=visual,
        collision=collision,
        scale=scale,
        mount_xyz=mount_xyz,
        mount_rpy=mount_rpy,
        tcp_xyz=tcp_xyz,
        tcp_rpy=tcp_rpy,
        mass=mass,
        touch_links=tuple(touch_links),
        fingers=fingers,
    )


def gripper_controllers_file():
    """Return the ros2_control parameters of the gripper controller (tools with fingers)."""
    return os.path.join(share_dir('fairino_sim'), 'config', 'gripper_controller.yaml')


def build_urdf(robot_name, controllers_file, base_xyz='0 0 0', base_rpy='0 0 0',
               initial_positions=DEFAULT_INITIAL_POSITIONS, tool=None, damping=None):
    """
    Return the URDF text used by Gazebo, robot_state_publisher, MoveIt and RViz.

    A tool with fingers adds the finger joints, open at the start, and the parameters of the
    gripper controller for Gazebo.
    """
    mappings = {
        'robot_name': robot_name,
        'controllers_file': controllers_file,
        'base_xyz': _format(parse_vector(base_xyz, 3, 'base_xyz')),
        'base_rpy': _format(parse_vector(base_rpy, 3, 'base_rpy')),
        'initial_positions': _format(parse_vector(initial_positions, 6, 'initial_positions')),
    }
    if tool is not None:
        mappings.update({
            'tool_visual': 'file://' + tool.visual,
            'tool_collision': 'file://' + tool.collision,
            'tool_scale': repr(tool.scale),
            'tool_mount_xyz': _format(tool.mount_xyz),
            'tool_mount_rpy': _format(tool.mount_rpy),
            'tool_mass': repr(tool.mass),
            'tcp_xyz': _format(tool.tcp_xyz),
            'tcp_rpy': _format(tool.tcp_rpy),
        })
    if tool is not None and tool.fingers is not None:
        fingers = tool.fingers
        mappings.update({
            'finger_visual': 'file://' + fingers.visual,
            'finger_collision': 'file://' + fingers.collision,
            'finger_scale': repr(fingers.scale),
            'finger_xyz': _format(fingers.origin_xyz),
            'finger_stroke': repr(fingers.stroke),
            'finger_speed': repr(fingers.speed),
            'finger_mass': repr(fingers.mass),
            'gripper_controllers_file': gripper_controllers_file(),
        })
    xacro_file = os.path.join(share_dir('fairino_sim'), 'config', 'frcobot_gz.urdf.xacro')
    try:
        urdf = xacro.process_file(xacro_file, mappings=mappings).toprettyxml(indent='  ')
    except Exception as exc:  # xacro raises several unrelated exception types
        raise DescriptionError(f'xacro failed for {robot_name}: {exc}') from exc

    # Gazebo cannot resolve package:// URIs of fairino_description; use absolute paths.
    urdf = urdf.replace('package://fairino_description',
                        'file://' + share_dir('fairino_description'))
    if damping is not None:
        # fairino_description ships damping="0"; Gazebo needs some damping to stay stable.
        urdf = urdf.replace('damping="0"', f'damping="{damping}"')
    return urdf


def robot_links(robot_name):
    """Return the link names of a model, including the frames added by fairino_sim."""
    urdf_file = os.path.join(share_dir('fairino_description'), 'urdf', f'{robot_name}.urdf')
    root = ET.parse(urdf_file).getroot()
    return {link.get('name') for link in root.findall('link')} | set(ADDED_LINKS)


def build_srdf(robot_name, tool=None):
    """Return the model's SRDF from its MoveIt config, extended with the tool's rules."""
    srdf_file = os.path.join(share_dir(moveit_config_package(robot_name)), 'config',
                             f'{robot_name}_robot.srdf')
    try:
        root = ET.parse(srdf_file).getroot()
    except (OSError, ET.ParseError) as exc:
        raise DescriptionError(f'Cannot read SRDF {srdf_file}: {exc}') from None

    if tool is not None:
        links = robot_links(robot_name)
        for link in tool.touch_links:
            if link not in links:
                raise DescriptionError(
                    f"Tool '{tool.name}': touch link '{link}' is not a link of {robot_name}")
            reason = 'Adjacent' if link == 'wrist3_link' else 'User'
            ET.SubElement(root, 'disable_collisions', link1='tool_link', link2=link,
                          reason=reason)
        if tool.fingers is not None:
            # the fingers slide on the tool and meet when the gripper closes empty
            pairs = [(finger, 'tool_link', 'Adjacent') for finger in FINGER_LINKS]
            pairs += [(finger, link, 'User') for finger in FINGER_LINKS
                      for link in tool.touch_links]
            pairs.append((FINGER_LINKS[0], FINGER_LINKS[1], 'User'))
            for link1, link2, reason in pairs:
                ET.SubElement(root, 'disable_collisions', link1=link1, link2=link2,
                              reason=reason)
    return ET.tostring(root, encoding='unicode')
