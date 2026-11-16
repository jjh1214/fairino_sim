"""
Run and check recipe steps in the work cell: PTP moves, grip and release, virtual IO.

Every step is checked before it runs (see fairino_workcell.robot_cell): the target must be
reachable and within the joint limits, and the whole path must be free of collisions of the
robot, the tool and the held object with the cell and with themselves. Steps that pass closer
to the cell than clearance_warning are reported as warnings. Check-only steps use a separate
check session, so a recipe can be validated without moving the simulation.

Interfaces (fairino_workcell_msgs):
  action  /workcell/move          Move       check the path, then move (or only check)
  service /workcell/gripper       Gripper    grip the object at the TCP, or release it
  service /workcell/set_io        SetIO      set a virtual digital output
  service /workcell/check/begin   std_srvs/Trigger  start a check session from the current state
  service /workcell/run/begin     std_srvs/Trigger  start a recipe run (clears the step markers)
  topic   /workcell/io            IOState    virtual IO (latched)
  topic   /workcell/step_report   StepReport one message per step
  topic   /workcell/check_markers visualization_msgs/MarkerArray  TCP paths of the steps
"""

import math
import threading
import time
import traceback

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from fairino_sim import description
from fairino_workcell import motion
from fairino_workcell.cell import CellError, load_cell
from fairino_workcell.kinematics import KinematicsError
from fairino_workcell.robot_cell import (DEFAULT_SPEED, HELD_TOUCH_LINKS, JOINTS, new_report,
                                         RobotCell, StepError)
from fairino_workcell.scene import frame_transforms, transform_msg
from fairino_workcell_msgs.action import Move
from fairino_workcell_msgs.msg import IOState, StepReport
from fairino_workcell_msgs.srv import Gripper, SetIO
from geometry_msgs.msg import Point
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import (AttachedCollisionObject, DisplayTrajectory, PlanningScene,
                             RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

STATUS_COLORS = {StepReport.OK: (0.2, 0.8, 0.3), StepReport.WARNING: (1.0, 0.65, 0.0),
                 StepReport.FAILED: (0.95, 0.15, 0.15)}
LOAD_ERRORS = (CellError, KinematicsError, description.DescriptionError)


class WorkcellExecutor(Node):
    """Check and execute recipe steps for the robot of a cell file."""

    def __init__(self):
        super().__init__('workcell_executor')
        p = self.declare_parameter
        self.cell_file = p('cell_file', '').value
        self.default_speed = float(p('default_speed', DEFAULT_SPEED).value)
        self.settings = {
            'joint_acceleration': float(p('joint_acceleration', 3.0).value),
            'clearance_warning': float(p('clearance_warning', 0.01).value),
            'grasp_tolerance': float(p('grasp_tolerance', 0.01).value),
            'check_step': math.radians(float(p('check_step_deg', 0.5).value)),
        }
        self.execution_tolerance = float(p('execution_tolerance', 0.02).value)
        self.gazebo_world = p('gazebo_world', 'empty').value
        self.use_gazebo = p('use_gazebo', True).value

        self.lock = threading.Lock()        # one step at a time
        self.joint_lock = threading.Lock()
        self.joint_positions = {}
        self.io = {}
        self.step_count = 0
        self.markers = MarkerArray()
        self.cell_version = None
        self._load_cell(self.cell_file)
        self.controller = description.trajectory_controller_name(self.cell.robot.model)

        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 20,
                                 callback_group=group)
        self.create_subscription(String, '/workcell/cell', self._on_cell, latched,
                                 callback_group=group)
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                               callback_group=group)
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, f'/{self.controller}/follow_joint_trajectory',
            callback_group=group)
        self.report_pub = self.create_publisher(StepReport, '/workcell/step_report', 10)
        self.io_pub = self.create_publisher(IOState, '/workcell/io', latched)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 20)
        self.marker_pub = self.create_publisher(MarkerArray, '/workcell/check_markers', latched)
        self.display_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 5)
        ActionServer(self, Move, '/workcell/move', self._on_move, callback_group=group,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(Gripper, '/workcell/gripper', self._on_gripper, callback_group=group)
        self.create_service(SetIO, '/workcell/set_io', self._on_set_io, callback_group=group)
        self.create_service(Trigger, '/workcell/check/begin', self._on_check_begin,
                            callback_group=group)
        self.create_service(Trigger, '/workcell/run/begin', self._on_run_begin,
                            callback_group=group)
        self.gz = GzNode() if self.use_gazebo else None
        self.create_timer(0.05, self._publish_objects, callback_group=group)
        self.io_pub.publish(IOState())
        self.get_logger().info(
            f"workcell executor ready for '{self.cell.name}' ({self.cell.robot.model}, "
            f'controller {self.controller})')

    # ------------------------------------------------------------------ cell and state
    def _load_cell(self, path):
        cell = load_cell(path)
        self.core = RobotCell(cell, **self.settings)
        self.cell = cell
        self.run_state = self.core.initial_state()
        self.check_state = None

    def _on_cell(self, msg):
        path, _, version = msg.data.rpartition('#')
        if self.cell_version is None:
            self.cell_version = version
            return
        if version == self.cell_version:
            return
        self.cell_version = version
        with self.lock:
            for name in list(self.run_state.held):
                self._move_group_attach(name, attach=False)
            try:
                self._load_cell(path)
                self.get_logger().info(f'cell reloaded: {path}')
            except LOAD_ERRORS as exc:
                self.get_logger().error(f'cannot reload the cell: {exc}')

    def _on_joint_states(self, msg):
        with self.joint_lock:
            self.joint_positions.update(zip(msg.name, msg.position))

    def current_joints(self):
        with self.joint_lock:
            if not all(name in self.joint_positions for name in JOINTS):
                raise StepError('no joint states received yet')
            return tuple(self.joint_positions[name] for name in JOINTS)

    def _state(self, check_only):
        if not check_only:
            self.run_state.joints = self.current_joints()
            return self.run_state
        if self.check_state is None:
            self._begin_check()
        return self.check_state

    def _begin_check(self):
        self.run_state.joints = self.current_joints()
        self.check_state = self.run_state.copy()
        self._clear_markers()

    def _clear_markers(self):
        self.markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        self.marker_pub.publish(self.markers)
        self.markers = MarkerArray()

    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            return None
        return future.result()

    # ------------------------------------------------------------------ move
    def _on_move(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            self.step_count += 1
            report = new_report(goal.name or 'move', 'move', goal.check_only)
            state = None
            try:
                state = self._state(goal.check_only)
                report.start_joints = list(state.joints)
                self._move(goal, state, report, goal_handle)
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of dropping the goal
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            report.end_joints = list(state.joints) if state is not None else []
            self._finish(report)
        result = Move.Result(success=report.status != StepReport.FAILED, report=report)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _move(self, goal, state, report, goal_handle):
        goal_handle.publish_feedback(Move.Feedback(phase='target'))
        if goal.target_type == Move.Goal.JOINTS:
            target = tuple(float(v) for v in goal.joints)
        elif goal.target_type == Move.Goal.POSE:
            p, o = goal.pose.pose.position, goal.pose.pose.orientation
            target = self.core.pose_target(state, goal.pose.header.frame_id, motion.pose_matrix(
                (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
        else:
            raise StepError(f'unknown target_type {goal.target_type}')
        goal_handle.publish_feedback(Move.Feedback(phase='check'))
        plan = self.core.plan_move(state, target, goal.speed or self.default_speed,
                                   goal.allowed_contacts, report)
        self._add_path_marker(report, plan.check)
        if report.status == StepReport.FAILED:
            return
        end = plan.goal
        if not goal.check_only:
            goal_handle.publish_feedback(Move.Feedback(phase='execute'))
            self._execute(plan, goal_handle)
            end = self.current_joints()
            error = float(np.max(np.abs(np.asarray(end) - np.asarray(plan.goal))))
            if error > self.execution_tolerance:
                report.status = max(report.status, StepReport.WARNING)
                report.message += f'; tracking error {math.degrees(error):.1f} deg'
        self.core.complete_move(state, end)

    def _execute(self, plan, goal_handle):
        times, positions, velocities, accelerations = motion.sample_ptp(
            plan.start, plan.goal, plan.profile, 0.02)
        trajectory = JointTrajectory(joint_names=JOINTS)
        for t, q, v, a in list(zip(times, positions, velocities, accelerations))[1:]:
            trajectory.points.append(JointTrajectoryPoint(
                positions=list(q), velocities=list(v), accelerations=list(a),
                time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        display = DisplayTrajectory(model_id=self.cell.robot.model)
        display.trajectory_start.joint_state = JointState(name=JOINTS, position=list(plan.start))
        display.trajectory.append(RobotTrajectory(joint_trajectory=trajectory))
        self.display_pub.publish(display)
        if not trajectory.points:
            return
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise StepError(f'controller {self.controller} is not available')
        send = self.trajectory_client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=trajectory))
        handle = self._wait(send, 10.0)
        if handle is None or not handle.accepted:
            raise StepError('the trajectory was rejected by the controller')
        result_future = handle.get_result_async()
        deadline = time.monotonic() + plan.profile.duration + 15.0
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                raise StepError('cancelled')
            if time.monotonic() > deadline:
                raise StepError('the controller did not finish the trajectory')
            time.sleep(0.02)
        if result_future.result().result.error_code != 0:
            raise StepError('controller error: ' + result_future.result().result.error_string)

    def _wait(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    # ------------------------------------------------------------------ gripper
    def _on_gripper(self, request, response):
        kind = 'grip' if request.close else 'release'
        with self.lock:
            self.step_count += 1
            report = new_report(request.name or kind, kind, request.check_only)
            try:
                state = self._state(request.check_only)
                report.start_joints = report.end_joints = list(state.joints)
                if request.close:
                    name = self.core.grip(state, report)
                    if not request.check_only:
                        self._move_group_attach(name, attach=True)
                else:
                    released = self.core.release(state, report)
                    if released and not request.check_only:
                        self._move_group_attach(released[0], attach=False)
                        self._gazebo_pose(*released)
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of failing the service
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            self._finish(report)
        response.success = report.status != StepReport.FAILED
        response.report = report
        return response

    def _move_group_attach(self, name, attach):
        attached = AttachedCollisionObject(link_name='tcp', touch_links=HELD_TOUCH_LINKS)
        attached.object.id = name
        attached.object.operation = attached.object.ADD if attach else attached.object.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        response = self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene), 10.0)
        if response is None or not response.success:
            self.get_logger().warning(f"MoveIt did not {'attach' if attach else 'detach'} {name}")

    # ------------------------------------------------------------------ objects, IO, reports
    def _publish_objects(self):
        """Publish the TF of the movable objects; held ones follow the TCP (also in Gazebo)."""
        try:
            state = self.run_state
            try:
                joints = self.current_joints()
            except StepError:
                joints = state.joints
            poses, held = self.core.object_poses(state, joints)
            stamp = self.get_clock().now().to_msg()
            transforms = []
            for name, pose in poses.items():
                transforms.append(transform_msg('world', name, pose, stamp))
                transforms.extend(frame_transforms(self.core.objects[name], stamp))
                if name in held:
                    self._gazebo_pose(name, pose)
            if transforms:
                self.tf_pub.publish(TFMessage(transforms=transforms))
        except Exception as exc:  # keep publishing even if one cycle fails (e.g. during reload)
            self.get_logger().debug(f'object publishing skipped: {exc}')

    def _gazebo_pose(self, name, matrix):
        if self.gz is None:
            return
        request = GzPose()
        request.name = name
        request.position.x, request.position.y, request.position.z = (
            float(v) for v in matrix[:3, 3])
        q = motion.matrix_quaternion(matrix[:3, :3])
        request.orientation.x, request.orientation.y = q[0], q[1]
        request.orientation.z, request.orientation.w = q[2], q[3]
        self.gz.request(f'/world/{self.gazebo_world}/set_pose', request, GzPose, Boolean, 200)

    def _on_set_io(self, request, response):
        self.io[request.name] = request.value
        self.io_pub.publish(IOState(names=list(self.io), values=list(self.io.values())))
        response.success, response.message = True, f'{request.name} = {request.value}'
        return response

    def _on_check_begin(self, request, response):
        with self.lock:
            try:
                self._begin_check()
                response.success, response.message = True, 'check session started'
            except StepError as exc:
                response.success, response.message = False, str(exc)
        return response

    def _on_run_begin(self, request, response):
        with self.lock:
            self._clear_markers()
        response.success, response.message = True, 'run started'
        return response

    def _finish(self, report):
        state = self.check_state if report.check_only else self.run_state
        if state is not None:
            report.held_object = next(iter(state.held), '')
        self.report_pub.publish(report)
        text = f"{'[check] ' if report.check_only else ''}{report.name}: {report.message}"
        # rclpy needs one severity per logging call site
        if report.status == StepReport.FAILED:
            self.get_logger().error(text)
        elif report.status == StepReport.WARNING:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def _add_path_marker(self, report, check):
        if not check.tcp_path:
            return
        r, g, b = STATUS_COLORS[report.status]
        line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, id=self.step_count,
                      ns='check' if report.check_only else 'run')
        line.header.frame_id = 'world'
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006
        line.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        line.points = [Point(x=x, y=y, z=z) for x, y, z in check.tcp_path]
        self.markers.markers.append(line)
        if check.collision_at >= 0.0:
            hit = Marker(type=Marker.SPHERE, action=Marker.ADD, id=self.step_count,
                         ns='collision')
            hit.header.frame_id = 'world'
            hit.pose.position = line.points[-1]
            hit.pose.orientation.w = 1.0
            hit.scale.x = hit.scale.y = hit.scale.z = 0.05
            hit.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            self.markers.markers.append(hit)
        self.marker_pub.publish(self.markers)


def _spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WorkcellExecutor()
    except LOAD_ERRORS as exc:
        print(f'workcell_executor: {exc}')
        rclpy.try_shutdown()
        return 1
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        _spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
