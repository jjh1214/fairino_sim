"""
Publish a work cell to MoveIt, Gazebo, TF and RViz.

ROS parameters:
  cell_file     path of cell.yaml
  use_moveit    add the objects to the MoveIt planning scene (default true)
  use_gazebo    spawn the objects as static Gazebo models (default true)
  gazebo_world  name of the Gazebo world (default 'empty')
  wait_timeout  seconds to wait for move_group and Gazebo at start-up (default 120)

Service:
  /workcell/reload (std_srvs/Trigger)  read cell_file again and update everything
Topics:
  /workcell/markers (visualization_msgs/MarkerArray, latched)  visual meshes for RViz
  /workcell/cell (std_msgs/String, latched)  '<cell file>#<n>' after every (re)load
"""

import threading
import time

from fairino_workcell.cell import CellError, load_cell
from fairino_workcell.meshes import MeshError
from fairino_workcell.scene import (allow_collisions, collision_object, gazebo_sdf, markers,
                                    object_color, remove_object, static_transforms)
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

GZ_TIMEOUT_MS = 5000


class Gazebo:
    """Spawn and remove static models through the Gazebo world services."""

    def __init__(self, world):
        self.node = GzNode()
        self.world = world

    def wait(self, timeout):
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
                                         Boolean, GZ_TIMEOUT_MS)
        return ok and response.data

    def remove(self, name):
        request = Entity()
        request.name = name
        request.type = Entity.MODEL
        ok, response = self.node.request(f'/world/{self.world}/remove', request, Entity,
                                         Boolean, GZ_TIMEOUT_MS)
        return ok and response.data


class WorkcellManager(Node):
    """Keep MoveIt, Gazebo, TF and RViz in sync with a cell file."""

    def __init__(self):
        super().__init__('workcell_manager')
        self.cell_file = self.declare_parameter('cell_file', '').value
        self.use_moveit = self.declare_parameter('use_moveit', True).value
        self.use_gazebo = self.declare_parameter('use_gazebo', True).value
        world = self.declare_parameter('gazebo_world', 'empty').value
        self.wait_timeout = float(self.declare_parameter('wait_timeout', 120.0).value)
        # Movable objects get dynamic TF from workcell_executor when it runs.
        self.publish_movable_tf = self.declare_parameter('publish_movable_tf', True).value
        self.applied = 0

        group = ReentrantCallbackGroup()
        self.apply_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene',
                                               callback_group=group)
        self.scene_client = self.create_client(GetPlanningScene, 'get_planning_scene',
                                               callback_group=group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Publish /tf_static directly: tf2_ros.StaticTransformBroadcaster in Jazzy ignores
        # updates to frames it has already sent, so moved objects would keep their old pose.
        self.tf_publisher = self.create_publisher(TFMessage, '/tf_static', latched)
        self.marker_publisher = self.create_publisher(MarkerArray, 'workcell/markers', latched)
        self.cell_publisher = self.create_publisher(String, 'workcell/cell', latched)
        self.create_service(Trigger, 'workcell/reload', self.on_reload, callback_group=group)
        self.gazebo = Gazebo(world) if self.use_gazebo else None
        self.cell = None
        self.lock = threading.Lock()

    def apply(self, startup=False):
        """Load the cell file and publish it everywhere; return a one-line summary."""
        with self.lock:
            cell = load_cell(self.cell_file)
            previous = self.cell
            timeout = self.wait_timeout if startup else 5.0
            self.tf_publisher.publish(TFMessage(transforms=static_transforms(
                cell, self.get_clock().now().to_msg(), self.publish_movable_tf)))
            self.marker_publisher.publish(markers(cell))
            results = [f'TF {len(cell.frame_names())} frames']
            if self.use_moveit:
                results.append('MoveIt ' + self.apply_moveit(cell, previous, timeout))
            if self.gazebo is not None:
                results.append('Gazebo ' + self.apply_gazebo(cell, previous, timeout))
            self.cell = cell
            self.applied += 1
            self.cell_publisher.publish(String(data=f'{cell.path}#{self.applied}'))
            return f"workcell '{cell.name}' ({len(cell.objects)} objects): " + ', '.join(results)

    def apply_moveit(self, cell, previous, timeout):
        if not (self.apply_client.wait_for_service(timeout_sec=timeout)
                and self.scene_client.wait_for_service(timeout_sec=5.0)):
            return 'not available, skipped'
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        names = {obj.name for obj in cell.objects}
        if previous is not None:
            scene.world.collision_objects.extend(
                remove_object(obj.name) for obj in previous.objects if obj.name not in names)
        scene.world.collision_objects.extend(collision_object(obj) for obj in cell.objects)
        scene.object_colors = [object_color(obj) for obj in cell.objects]
        pairs = [(obj.name, link) for obj in cell.objects for link in obj.touch_links]
        if pairs:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            current = self.call(self.scene_client, request)
            if current is None:
                return 'failed (no allowed collision matrix)'
            scene.allowed_collision_matrix = allow_collisions(
                current.scene.allowed_collision_matrix, pairs)
        response = self.call(self.apply_client, ApplyPlanningScene.Request(scene=scene))
        return 'ok' if response is not None and response.success else 'FAILED'

    def apply_gazebo(self, cell, previous, timeout):
        if not self.gazebo.wait(timeout):
            return 'not available, skipped'
        if previous is not None:
            for obj in previous.objects:
                self.gazebo.remove(obj.name)
            time.sleep(1.0)  # removal finishes on the next simulation steps
        failed = [obj.name for obj in cell.objects if not self.gazebo.spawn(gazebo_sdf(obj))]
        return 'ok' if not failed else f'FAILED for {failed}'

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            return None
        return future.result()

    def on_reload(self, request, response):
        try:
            response.message = self.apply()
            response.success = True
            self.get_logger().info(response.message)
        except (CellError, MeshError) as exc:
            response.message = str(exc)
            response.success = False
            self.get_logger().error(response.message)
        return response


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WorkcellManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()
    code = 0
    try:
        node.get_logger().info(node.apply(startup=True))
        spin_thread.join()
    except (CellError, MeshError) as exc:
        node.get_logger().error(str(exc))
        code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return code
