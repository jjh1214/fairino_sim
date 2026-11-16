"""
Collision and clearance checks of robot paths against a work cell, with MoveIt (moveit_py).

moveit_py shares one collision world between all PlanningScene objects and cannot return
contact names, so the checker rebuilds a single scene for each state of the cell and answers
each question with its own allowed collision matrix (ACM):

- self collisions: robot links and held objects, as allowed by the SRDF;
- environment collisions: robot links and held objects against the cell objects;
- clearance: robot links and held objects against copies of the cell objects attached to the
  fixed 'world' link. MoveIt returns the self-distance last, so attaching the copies is what
  makes moveit_py report the distance to the environment.

Colliding and closest pairs are named by repeating a check with one object or body enabled.
"""

from dataclasses import dataclass, field
import os
import tempfile
import xml.etree.ElementTree as ET

from fairino_workcell import motion
from fairino_workcell.cell import Pose
from fairino_workcell.scene import collision_object, pose_msg
from moveit.core import collision_detection, planning_scene, robot_model
from moveit_msgs.msg import AttachedCollisionObject
import numpy as np

COPY_SUFFIX = '__clearance'
ACM = collision_detection.AllowedCollisionMatrix


@dataclass
class PathCheck:
    """Result of checking a joint-space path."""

    collision_at: float = -1.0          # path fraction of the first collision, -1 if none
    collisions: list = field(default_factory=list)   # 'body <-> object' pairs
    min_clearance: float = -1.0         # metres, -1 if not computed
    closest_body: str = ''
    closest_object: str = ''
    tcp_path: list = field(default_factory=list)     # TCP positions along the path
    end_joints: tuple = ()


class Checker:
    """Collision checks of one robot in one cell, for any object poses and held objects."""

    _world_owner = None     # the checker whose objects are in the shared collision world

    def __init__(self, cell, urdf, srdf, group, tool_bodies):
        self.cell = cell
        self.group = group
        self._tmp = tempfile.mkdtemp(prefix='fairino_checker_')
        urdf_file, srdf_file = (os.path.join(self._tmp, n) for n in ('robot.urdf', 'robot.srdf'))
        with open(urdf_file, 'w') as stream:
            stream.write(urdf)
        with open(srdf_file, 'w') as stream:
            stream.write(srdf)
        self.model = robot_model.RobotModel(urdf_xml_path=urdf_file, srdf_xml_path=srdf_file)
        self.links = [link.get('name') for link in ET.fromstring(urdf).findall('link')
                      if link.find('collision') is not None]
        self.disabled = [(e.get('link1'), e.get('link2'))
                         for e in ET.fromstring(srdf).findall('disable_collisions')]
        self.tool_bodies = list(tool_bodies)
        self.objects = {obj.name: obj for obj in cell.objects}
        self.touch = {(obj.name, link) for obj in cell.objects for link in obj.touch_links}
        self._synced = None
        self.held = {}
        self.world = []

    # ---------------------------------------------------------------- scene
    def sync(self, object_poses, held, held_touch_links):
        """
        Rebuild the scene for the given object poses and held objects.

        object_poses maps object -> 4x4 world pose; held maps object -> 4x4 pose relative to
        the TCP, for the objects in the gripper.
        """
        key = (tuple(sorted((n, p.tobytes()) for n, p in object_poses.items())),
               tuple(sorted((n, p.tobytes()) for n, p in held.items())))
        if key == self._synced and Checker._world_owner is self:
            return
        self.scene = planning_scene.PlanningScene(self.model)
        self.scene.remove_all_collision_objects()   # the collision world is shared
        self.held = {name: list(held_touch_links) for name in held}
        self.world = [name for name in self.objects if name not in held]
        for name in self.world:
            self.scene.apply_collision_object(self._object(name, 'world', object_poses[name]))
            copy = AttachedCollisionObject(link_name='world')
            copy.object = self._object(name, 'world', object_poses[name], name + COPY_SUFFIX)
            self.scene.process_attached_collision_object(copy)
        for name, pose in held.items():
            attached = AttachedCollisionObject(link_name='tcp', touch_links=held_touch_links)
            attached.object = self._object(name, 'tcp', pose)
            self.scene.process_attached_collision_object(attached)
        self.state = self.scene.current_state
        self._synced = key
        Checker._world_owner = self

    def _object(self, name, frame, matrix, object_id=None):
        msg = collision_object(self.objects[name])
        msg.id = object_id or name
        msg.header.frame_id = frame
        msg.pose = pose_msg(_pose(matrix))
        return msg

    # ---------------------------------------------------------------- ACMs
    def _bodies(self):
        return self.links + list(self.held)

    def _copies(self):
        return [name + COPY_SUFFIX for name in self.world]

    def _allowed(self, body, obj, allowed_pairs):
        return (obj, body) in self.touch or (body, obj) in allowed_pairs

    def _self_acm(self):
        acm = ACM(self._bodies() + self._copies() + self.world, True)
        bodies = self._bodies()
        for i, a in enumerate(bodies):
            for b in bodies[i + 1:]:
                acm.set_entry(a, b, False)
        for a, b in self.disabled:
            acm.set_entry(a, b, True)
        for name, touch_links in self.held.items():
            for link in touch_links:
                acm.set_entry(name, link, True)
        return acm

    def _env_acm(self, allowed_pairs, only_object=None, only_body=None):
        bodies = self._bodies()
        acm = ACM(bodies + self._copies() + self.world, True)
        for obj in self.world:
            if only_object and obj != only_object:
                continue
            for body in bodies:
                if not (only_body and body != only_body):
                    acm.set_entry(body, obj, self._allowed(body, obj, allowed_pairs))
        return acm

    def _clearance_acm(self, allowed_pairs, only_object=None, only_body=None):
        bodies = self._bodies()
        acm = ACM(bodies + self._copies() + self.world, True)
        for obj in self.world:
            if only_object and obj != only_object:
                continue
            for body in bodies:
                if not (only_body and body != only_body):
                    acm.set_entry(body, obj + COPY_SUFFIX, self._allowed(body, obj, allowed_pairs))
        return acm

    def _check(self, acm, distance=False):
        request = collision_detection.CollisionRequest()
        request.distance = distance
        result = collision_detection.CollisionResult()
        self.scene.check_collision(request, result, self.state, acm)
        return result.collision, result.distance

    # ---------------------------------------------------------------- queries
    def set_joints(self, joints):
        self.state.set_joint_group_positions(self.group, list(joints))
        self.state.update()

    def link_pose(self, link):
        return np.array(self.state.get_global_link_transform(link))

    def tcp_pose(self, joints):
        """Return the 4x4 world pose of the TCP for the given joint angles."""
        self.set_joints(joints)
        return self.link_pose('tcp')

    def collisions(self, allowed_pairs=()):
        """Return the colliding 'body <-> object' pairs of the current state."""
        allowed_pairs = set(allowed_pairs)
        pairs = []
        if self._check(self._self_acm())[0]:
            bodies = self._bodies()
            for i, a in enumerate(bodies):
                for b in bodies[i + 1:]:
                    acm = ACM(self._bodies() + self._copies() + self.world, True)
                    if (a, b) in self.disabled or (b, a) in self.disabled:
                        continue
                    if a in self.held and b in self.held.get(a, ()):
                        continue
                    acm.set_entry(a, b, False)
                    if self._check(acm)[0]:
                        pairs.append(f'{a} <-> {b}')
        if self._check(self._env_acm(allowed_pairs))[0]:
            for obj in self.world:
                if not self._check(self._env_acm(allowed_pairs, only_object=obj))[0]:
                    continue
                for body in self._bodies():
                    if self._check(self._env_acm(allowed_pairs, obj, body))[0]:
                        pairs.append(f'{body} <-> {obj}')
        return pairs

    def in_collision(self, allowed_pairs=()):
        allowed_pairs = set(allowed_pairs)
        return (self._check(self._self_acm())[0]
                or self._check(self._env_acm(allowed_pairs))[0])

    def clearance(self, allowed_pairs=()):
        """Return (distance, body, object) of the closest robot body and cell object."""
        allowed_pairs = set(allowed_pairs)
        distance = self._check(self._clearance_acm(allowed_pairs), distance=True)[1]
        if not self.world or distance > 1e9:
            return -1.0, '', ''
        per_object = {obj: self._check(self._clearance_acm(allowed_pairs, only_object=obj),
                                       distance=True)[1] for obj in self.world}
        obj = min(per_object, key=per_object.get)
        per_body = {body: self._check(self._clearance_acm(allowed_pairs, obj, body),
                                      distance=True)[1] for body in self._bodies()}
        body = min(per_body, key=per_body.get)
        return max(float(distance), 0.0), body, obj

    def touching(self, held_name):
        """Return the cell objects that a held object touches in the current state."""
        result = []
        for obj in self.world:
            acm = self._env_acm(set(), only_object=obj, only_body=held_name)
            if self._check(acm)[0]:
                result.append(obj)
        return result

    def check_path(self, start, goal, allowed_pairs=(), step=np.radians(0.5),
                   clearance_every=8):
        """Check the joint-space line from start to goal; stop at the first collision."""
        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        result = PathCheck(end_joints=tuple(goal))
        fractions = motion.path_fractions(start, goal, step)
        distances = {}
        for index, fraction in enumerate(fractions):
            joints = start + fraction * (goal - start)
            self.set_joints(joints)
            result.tcp_path.append(tuple(self.link_pose('tcp')[:3, 3]))
            if self.in_collision(allowed_pairs):
                result.collision_at = float(fraction)
                result.collisions = self.collisions(allowed_pairs)
                result.min_clearance = 0.0
                return result
            if index % clearance_every == 0 or index == len(fractions) - 1:
                distances[index] = self._check(self._clearance_acm(set(allowed_pairs)),
                                               distance=True)[1]
        if not distances or min(distances.values()) > 1e9:
            return result
        best = min(distances, key=distances.get)
        window = range(max(0, best - clearance_every),
                       min(len(fractions), best + clearance_every + 1))
        for index in window:
            if index not in distances:
                self.set_joints(start + fractions[index] * (goal - start))
                distances[index] = self._check(self._clearance_acm(set(allowed_pairs)),
                                               distance=True)[1]
        best = min(distances, key=distances.get)
        self.set_joints(start + fractions[best] * (goal - start))
        result.min_clearance, result.closest_body, result.closest_object = self.clearance(
            allowed_pairs)
        return result


def _pose(matrix):
    """Return a cell Pose (xyz, rpy) of a 4x4 transform."""
    rotation = matrix[:3, :3]
    pitch = float(np.arcsin(-np.clip(rotation[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) > 1e-9:
        roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    else:  # gimbal lock
        roll, yaw = float(np.arctan2(-rotation[1, 2], rotation[1, 1])), 0.0
    return Pose(xyz=tuple(float(v) for v in matrix[:3, 3]), rpy=(roll, pitch, yaw))
