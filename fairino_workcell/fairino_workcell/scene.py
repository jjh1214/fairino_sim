"""Turn cell objects into MoveIt collision objects, Gazebo models, TF frames and RViz markers."""

import math
from xml.sax.saxutils import escape

from fairino_workcell.cell import frame_id, WORLD_FRAME
from fairino_workcell.meshes import read_stl
from fairino_workcell.motion import matrix_quaternion
from geometry_msgs.msg import Point, Pose, TransformStamped
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, ObjectColor
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the (x, y, z, w) quaternion of fixed-axis roll, pitch, yaw angles."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def pose_msg(pose):
    """Return a geometry_msgs/Pose for a cell Pose."""
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pose.xyz)
    q = quaternion_from_rpy(*pose.rpy)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


def _identity():
    msg = Pose()
    msg.orientation.w = 1.0
    return msg


def _transform(parent, child, pose, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    p = pose_msg(pose)
    msg.transform.translation.x = p.position.x
    msg.transform.translation.y = p.position.y
    msg.transform.translation.z = p.position.z
    msg.transform.rotation = p.orientation
    return msg


def static_transforms(cell, stamp, include_movable=True):
    """Return world -> object and object -> object/frame transforms of the cell objects."""
    transforms = []
    for obj in cell.objects:
        if obj.movable and not include_movable:
            continue
        transforms.append(_transform(WORLD_FRAME, obj.name, obj.pose, stamp))
        transforms.extend(frame_transforms(obj, stamp))
    return transforms


def frame_transforms(obj, stamp):
    """Return the object -> object/frame transforms of one object."""
    return [_transform(obj.name, frame_id(obj.name, name), pose, stamp)
            for name, pose in obj.frames]


def transform_msg(parent, child, matrix, stamp):
    """Return a TransformStamped for a 4x4 transform."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        float(v) for v in matrix[:3, 3])
    x, y, z, w = matrix_quaternion(matrix[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y = x, y
    msg.transform.rotation.z, msg.transform.rotation.w = z, w
    return msg


def collision_object(obj):
    """Return the MoveIt collision object (ADD) of a cell object, with its frames as subframes."""
    msg = CollisionObject()
    msg.header.frame_id = WORLD_FRAME
    msg.id = obj.name
    msg.pose = pose_msg(obj.pose)
    geometry = obj.geometry
    if geometry.kind == 'mesh':
        vertices, triangles = read_stl(geometry.collision, geometry.scale)
        mesh = Mesh()
        mesh.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)])
                          for a, b, c in triangles]
        msg.meshes = [mesh]
        msg.mesh_poses = [_identity()]
    else:
        primitive = SolidPrimitive()
        if geometry.kind == 'box':
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(v) for v in geometry.size]
        else:
            primitive.type = SolidPrimitive.CYLINDER
            radius, length = geometry.size
            dimensions = [0.0, 0.0]
            dimensions[SolidPrimitive.CYLINDER_HEIGHT] = float(length)
            dimensions[SolidPrimitive.CYLINDER_RADIUS] = float(radius)
            primitive.dimensions = dimensions
        msg.primitives = [primitive]
        msg.primitive_poses = [_identity()]
    msg.subframe_names = [name for name, _ in obj.frames]
    msg.subframe_poses = [pose_msg(pose) for _, pose in obj.frames]
    msg.operation = CollisionObject.ADD
    return msg


def remove_object(name):
    """Return a collision object message that removes an object from the planning scene."""
    msg = CollisionObject()
    msg.header.frame_id = WORLD_FRAME
    msg.id = name
    msg.operation = CollisionObject.REMOVE
    return msg


def _rgba(color):
    return ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]),
                     a=float(color[3]))


def object_color(obj):
    """Return the planning scene colour of a cell object."""
    return ObjectColor(id=obj.name, color=_rgba(obj.color))


def allow_collisions(matrix, pairs):
    """Allow contact between the given (name, name) pairs in an AllowedCollisionMatrix."""
    names = list(matrix.entry_names)
    rows = [list(entry.enabled) for entry in matrix.entry_values]

    def index(name):
        if name not in names:
            names.append(name)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))
        return names.index(name)

    for first, second in pairs:
        i, j = index(first), index(second)
        rows[i][j] = rows[j][i] = True
    matrix.entry_names = names
    matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
    return matrix


def _sdf_geometry(geometry, path):
    if geometry.kind == 'mesh':
        scale = ' '.join([repr(geometry.scale)] * 3)
        return f'<mesh><uri>file://{escape(path)}</uri><scale>{scale}</scale></mesh>'
    if geometry.kind == 'box':
        return '<box><size>{} {} {}</size></box>'.format(*geometry.size)
    radius, length = geometry.size
    return f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'


def _uses_cell_color(obj):
    return obj.color_given or not obj.geometry.has_embedded_materials


def gazebo_sdf(obj):
    """
    Return the SDF of a static Gazebo model for a cell object.

    The model is visual only: workcell_executor checks collisions and places the movable
    objects itself, so Gazebo physics must not stop the robot at contacts that a step allows
    (fingers around a part, a part set down on a table).
    """
    x, y, z = obj.pose.xyz
    roll, pitch, yaw = obj.pose.rpy
    material = ''
    if _uses_cell_color(obj):
        rgba = ' '.join(repr(float(c)) for c in obj.color)
        material = f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.10">\n'
        f'  <model name="{escape(obj.name)}">\n'
        '    <static>true</static>\n'
        f'    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>\n'
        '    <link name="link">\n'
        f'      <visual name="visual"><geometry>{_sdf_geometry(obj.geometry, obj.geometry.visual)}'
        f'</geometry>{material}</visual>\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n')


def markers(cell, namespace='workcell'):
    """Return RViz markers showing the visual geometry of every cell object."""
    array = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    array.markers.append(clear)
    for index, obj in enumerate(cell.objects):
        marker = Marker()
        marker.header.frame_id = obj.name   # follows the object when it moves
        marker.frame_locked = True
        marker.ns = namespace
        marker.id = index
        marker.action = Marker.ADD
        marker.pose = _identity()
        marker.color = _rgba(obj.color)
        geometry = obj.geometry
        if geometry.kind == 'mesh':
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = 'file://' + geometry.visual
            marker.mesh_use_embedded_materials = not _uses_cell_color(obj)
            marker.scale.x = marker.scale.y = marker.scale.z = geometry.scale
        elif geometry.kind == 'box':
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in geometry.size)
        else:
            marker.type = Marker.CYLINDER
            radius, length = geometry.size
            marker.scale.x = marker.scale.y = 2.0 * radius
            marker.scale.z = float(length)
        array.markers.append(marker)
    return array
