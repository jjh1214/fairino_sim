"""Spawn, move and remove static models through the Gazebo world services (gz-transport)."""

import time

from fairino_workcell import motion
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode

TIMEOUT_MS = 5000
POSE_TIMEOUT_MS = 200


class Gazebo:
    """Client of the services of one Gazebo world."""

    def __init__(self, world):
        self.node = GzNode()
        self.world = world

    def wait(self, timeout):
        """Wait until the world services are up; return False after timeout seconds."""
        service = f'/world/{self.world}/create'
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if service in self.node.service_list():
                return True
            time.sleep(0.5)
        return False

    def spawn(self, sdf):
        request = EntityFactory()
        request.sdf = sdf
        request.allow_renaming = False
        ok, response = self.node.request(f'/world/{self.world}/create', request, EntityFactory,
                                         Boolean, TIMEOUT_MS)
        return ok and response.data

    def remove(self, name):
        request = Entity()
        request.name = name
        request.type = Entity.MODEL
        ok, response = self.node.request(f'/world/{self.world}/remove', request, Entity,
                                         Boolean, TIMEOUT_MS)
        return ok and response.data

    def set_pose(self, name, matrix):
        """Move a model to a 4x4 world pose."""
        request = GzPose()
        request.name = name
        request.position.x, request.position.y, request.position.z = (
            float(v) for v in matrix[:3, 3])
        x, y, z, w = motion.matrix_quaternion(matrix[:3, :3])
        request.orientation.x, request.orientation.y = x, y
        request.orientation.z, request.orientation.w = z, w
        ok, response = self.node.request(f'/world/{self.world}/set_pose', request, GzPose,
                                         Boolean, POSE_TIMEOUT_MS)
        return ok and response.data
