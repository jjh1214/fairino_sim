"""
Run and check recipe steps in the work cell, and run the conveyors that bring the parts.

Steps are PTP moves, grip and release, waits and virtual IO.

Every step is checked before it runs (see fairino_workcell.robot_cell): the target must be
reachable and within the joint limits, and the robot, the tool and the held object must not
collide with the cell, with parts that conveyors move meanwhile, or with themselves. Steps that
pass closer than clearance_warning are reported as warnings. Check-only steps use a separate
check session in virtual time, so a recipe can be validated without moving the simulation.

The conveyors run in simulation time. During a recipe run (/workcell/run/begin to
/workcell/run/end) they only run while a step takes time: a move for its planned duration, a
wait until its end. Checking a step, and the gaps between steps, take no cell time, as on a
robot controller, so a run follows the timeline of its check. After /workcell/reset the
conveyors wait for the next run; outside runs they run freely.

Interfaces (fairino_workcell_msgs):
  action  /workcell/move          Move       check the path, then move (or only check)
  action  /workcell/pick_moving   PickMoving pick a part on the move from a continuous conveyor
  service /workcell/gripper       Gripper    grip the object at the TCP, or release it
  service /workcell/wait          Wait       wait for a time or for an IO value
  service /workcell/set_io        SetIO      set a virtual IO
  service /workcell/check/begin   std_srvs/Trigger  start a check session from the current state
  service /workcell/run/begin     std_srvs/Trigger  start a recipe run (clears the step markers)
  service /workcell/run/end       std_srvs/Trigger  end the run: the conveyors run freely again
  service /workcell/reset         std_srvs/Trigger  put objects and conveyors back to the start
  topic   /workcell/io            IOState    virtual IO (latched)
  topic   /workcell/step_report   StepReport one message per step
  topic   /workcell/check_markers visualization_msgs/MarkerArray  TCP paths of the steps
  topic   /workcell/part_markers  visualization_msgs/MarkerArray  parts put in by conveyors
"""

from dataclasses import replace
import math
import threading
import time
import traceback

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from fairino_sim import description
from fairino_workcell import motion
from fairino_workcell.cell import CellError, load_cell, Pose
from fairino_workcell.gazebo import Gazebo
from fairino_workcell.kinematics import KinematicsError
from fairino_workcell.robot_cell import (DEFAULT_SPEED, HELD_TOUCH_LINKS, JOINTS, new_report,
                                         RobotCell, StepError)
from fairino_workcell.scene import (collision_object, frame_transforms, gazebo_sdf,
                                    object_color, object_markers, pose_msg, transform_msg)
from fairino_workcell_msgs.action import Move, PickMoving
from fairino_workcell_msgs.msg import IOState, StepReport
from fairino_workcell_msgs.srv import Gripper, SetIO, Wait
from geometry_msgs.msg import Point
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, DisplayTrajectory,
                             PlanningScene, RobotTrajectory)
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
SCENE_PERIOD = 0.1        # seconds between part pose updates sent to MoveIt


def _matrix_pose(matrix):
    return Pose(xyz=tuple(float(v) for v in matrix[:3, 3]), rpy=motion.matrix_rpy(matrix[:3, :3]))


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
            'tcp_speed': float(p('max_tcp_speed', 1.0).value),
            'tcp_acceleration': float(p('max_tcp_acceleration', 2.0).value),
        }
        self.execution_tolerance = float(p('execution_tolerance', 0.02).value)
        self.gazebo_world = p('gazebo_world', 'empty').value
        self.use_gazebo = p('use_gazebo', True).value
        self.wait_timeout = float(p('wait_timeout', 120.0).value)

        self.lock = threading.Lock()                     # one recipe step at a time
        self.changed = threading.Condition(threading.Lock())   # run state; notified on changes
        self.joint_lock = threading.Lock()
        self.joint_positions = {}
        self.ready = False        # Gazebo and MoveIt are up and show the parts
        self.in_run = False       # a recipe run is going on: steps grant the conveyor time
        self.budget = math.inf    # cell time the conveyors may still run (inf: freely)
        self.last_tick = None
        self.gz_poses, self.scene_poses = {}, {}   # last poses sent to Gazebo and MoveIt
        self.scene_sent = 0.0
        self.io_sent = None
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
        self.part_marker_pub = self.create_publisher(MarkerArray, '/workcell/part_markers',
                                                     latched)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 20)
        self.display_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 5)
        ActionServer(self, Move, '/workcell/move', self._on_move, callback_group=group,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        ActionServer(self, PickMoving, '/workcell/pick_moving', self._on_pick_moving,
                     callback_group=group, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(Gripper, '/workcell/gripper', self._on_gripper, callback_group=group)
        self.create_service(Wait, '/workcell/wait', self._on_wait, callback_group=group)
        self.create_service(SetIO, '/workcell/set_io', self._on_set_io, callback_group=group)
        self.create_service(Trigger, '/workcell/check/begin', self._on_check_begin,
                            callback_group=group)
        self.create_service(Trigger, '/workcell/run/begin', self._on_run_begin,
                            callback_group=group)
        self.create_service(Trigger, '/workcell/run/end', self._on_run_end, callback_group=group)
        self.create_service(Trigger, '/workcell/reset', self._on_reset, callback_group=group)
        self.gazebo = Gazebo(self.gazebo_world) if self.use_gazebo else None
        self.create_timer(0.05, self._tick, callback_group=group)
        threading.Thread(target=self._bring_up, daemon=True).start()
        self.get_logger().info(
            f"workcell executor ready for '{self.cell.name}' ({self.cell.robot.model}, "
            f'controller {self.controller}, {len(self.core.conveyors.models)} conveyors)')

    # ------------------------------------------------------------------ cell and state
    def _load_cell(self, path):
        cell = load_cell(path)
        self.core = RobotCell(cell, **self.settings)
        self.cell = cell
        self.run_state = self.core.initial_state()
        self.check_state = None
        self.gz_poses = {name: pose.copy() for name, pose in self.core.movable_poses.items()}
        self.scene_poses = dict(self.gz_poses)

    def _bring_up(self):
        """Wait for Gazebo and MoveIt, show the first parts, then start the conveyors."""
        if self.gazebo is not None and not self.gazebo.wait(self.wait_timeout):
            self.get_logger().error('Gazebo is not running: parts are not shown there')
            self.gazebo = None
        if not self.scene_client.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().error('move_group is not running: parts are not in its scene')
        with self.changed:
            parts = list(self.run_state.parts)
        self._show_parts(parts)
        with self.changed:
            self.last_tick = None
            self.ready = True

    def _on_cell(self, msg):
        path, _, version = msg.data.rpartition('#')
        if self.cell_version is None:
            self.cell_version = version
            return
        if version == self.cell_version:
            return
        self.cell_version = version
        with self.lock:
            self._clear_cell_state()
            try:
                with self.changed:
                    self._load_cell(path)
                    self.last_tick = None
                self.get_logger().info(f'cell reloaded: {path}')
            except LOAD_ERRORS as exc:
                self.get_logger().error(f'cannot reload the cell: {exc}')
            with self.changed:
                parts = list(self.run_state.parts)
            self._show_parts(parts)

    def _clear_cell_state(self):
        """Detach held objects and remove the parts from Gazebo, MoveIt and RViz."""
        with self.changed:
            held, parts = list(self.run_state.held), list(self.run_state.parts)
        for name in held:
            self._move_group_attach(name, attach=False)
            self.scene_poses[name] = None
        self._hide_parts(parts)

    def _on_joint_states(self, msg):
        with self.joint_lock:
            self.joint_positions.update(zip(msg.name, msg.position))

    def current_joints(self):
        with self.joint_lock:
            if not all(name in self.joint_positions for name in JOINTS):
                raise StepError('no joint states received yet')
            return tuple(self.joint_positions[name] for name in JOINTS)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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

    # ------------------------------------------------------------------ conveyor clock
    def _tick(self):
        """Run the conveyors in simulation time and show where everything is."""
        try:
            now = self._now()
            with self.changed:
                dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
                self.last_tick = now
                new, gone = [], []
                step = min(dt, self.budget)
                if self.ready and step > 0.0:
                    new, gone = self._advance_run(step)
                    self.budget -= step
                state = self.run_state
                try:
                    joints = self.current_joints()
                except StepError:
                    joints = state.joints
                poses, held = self.core.object_poses(state, joints)
                parts, io = dict(state.parts), dict(state.io)
            if new:
                self._show_parts(new)
            if gone:
                self._hide_parts(gone)
            self._publish_objects(poses, held, parts)
            if io != self.io_sent:
                self.io_pub.publish(IOState(names=list(io), values=list(io.values())))
                self.io_sent = io
        except Exception as exc:  # keep running even if one cycle fails (e.g. during reload)
            self.get_logger().debug(f'tick skipped: {exc}')

    def _publish_objects(self, poses, held, parts):
        stamp = self.get_clock().now().to_msg()
        transforms, moved = [], []
        for name, pose in poses.items():
            transforms.append(transform_msg('world', name, pose, stamp))
            transforms.extend(frame_transforms(self.core.definition(name, parts), stamp))
            if self.gazebo is not None and name in self.gz_poses and \
                    not np.array_equal(self.gz_poses[name], pose):
                self.gazebo.set_pose(name, pose)
                self.gz_poses[name] = pose
            if name not in held and name in self.scene_poses and (
                    self.scene_poses[name] is None
                    or not np.array_equal(self.scene_poses[name], pose)):
                moved.append(name)
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))
        if moved and time.monotonic() - self.scene_sent >= SCENE_PERIOD:
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            for name in moved:
                obj = CollisionObject(id=name, operation=CollisionObject.MOVE)
                obj.header.frame_id = 'world'
                obj.pose = pose_msg(_matrix_pose(poses[name]))
                scene.world.collision_objects.append(obj)
                self.scene_poses[name] = poses[name]
            self.scene_pub.publish(scene)
            self.scene_sent = time.monotonic()

    def _advance_run(self, dt):
        """Run the conveyors of the run state for dt (lock held); return parts put in, gone."""
        before = set(self.run_state.poses)
        new = self.core.advance(self.run_state, dt)
        gone = [name for name in before
                if name not in self.run_state.poses and name not in self.run_state.held]
        self.changed.notify_all()
        return new, gone

    def _flush_budget(self):
        """
        Let the rest of the step's planned time pass when the robot has finished.

        The clock grants a step its time in ticks; without this, the part of the last tick
        the step did not use would be lost and the conveyors would fall behind the plan.
        """
        new, gone = [], []
        with self.changed:
            if self.ready and 0.0 < self.budget < math.inf:
                new, gone = self._advance_run(self.budget)
                self.budget = 0.0
        if new:
            self._show_parts(new)
        if gone:
            self._hide_parts(gone)

    def _show_parts(self, names):
        """Put new parts into Gazebo, the MoveIt scene and RViz."""
        with self.changed:
            state = self.run_state
            items = [(name, replace(self.core.definition(name, state.parts),
                                    pose=_matrix_pose(state.poses[name])), state.poses[name])
                     for name in names if name in state.poses]
            all_parts = self._present_parts()
        if items:
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            for name, obj, pose in items:
                if self.gazebo is not None and self.gazebo.spawn(gazebo_sdf(obj)):
                    self.gz_poses[name] = pose
                scene.world.collision_objects.append(collision_object(obj))
                scene.object_colors.append(object_color(obj))
                self.scene_poses[name] = pose
            response = self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene), 10.0)
            if response is None or not response.success:
                names = ', '.join(name for name, _, _ in items)
                self.get_logger().warning(f'MoveIt did not add {names}')
        self.part_marker_pub.publish(object_markers(all_parts, 'parts'))

    def _hide_parts(self, names):
        scene = PlanningScene(is_diff=True)
        for name in names:
            if self.gazebo is not None and name in self.gz_poses:
                self.gazebo.remove(name)
            self.gz_poses.pop(name, None)
            if name in self.scene_poses:
                del self.scene_poses[name]
                scene.world.collision_objects.append(
                    CollisionObject(id=name, operation=CollisionObject.REMOVE))
        if scene.world.collision_objects:
            self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene), 10.0)
        with self.changed:
            present = self._present_parts()
        self.part_marker_pub.publish(object_markers(present, 'parts'))

    def _present_parts(self):
        """Return the definitions of the parts in the cell (on a belt, put down or held)."""
        state = self.run_state
        return [self.core.definition(name, state.parts) for name in state.parts
                if name in state.poses or name in state.held]

    # ------------------------------------------------------------------ sessions
    def _step_state(self, check_only, snapshot=False):
        """Return the state a step works on; the conveyors wait while the step is checked."""
        with self.changed:
            if check_only:
                if self.check_state is None:
                    self._begin_check()
                return self.check_state
            self.budget = 0.0
            self.run_state.joints = self.current_joints()
            return self.run_state.copy() if snapshot else self.run_state

    def _grant(self, seconds):
        """Let the conveyors run while a step takes time (freely outside a run)."""
        with self.changed:
            self.budget = seconds if self.in_run else math.inf

    def _end_step(self):
        self._grant(0.0)

    def _begin_check(self):
        self.run_state.joints = self.current_joints()
        self.check_state = self.run_state.copy()
        self._clear_markers()

    def _clear_markers(self):
        self.markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        self.marker_pub.publish(self.markers)
        self.markers = MarkerArray()

    def _on_check_begin(self, request, response):
        with self.lock, self.changed:
            try:
                self._begin_check()
                response.success, response.message = True, 'check session started'
            except StepError as exc:
                response.success, response.message = False, str(exc)
        return response

    def _on_run_begin(self, request, response):
        with self.lock, self.changed:
            self.in_run, self.budget = True, 0.0
            self._clear_markers()
        response.success, response.message = True, 'run started'
        return response

    def _on_run_end(self, request, response):
        with self.lock, self.changed:
            self.in_run, self.budget = False, math.inf
        response.success, response.message = True, 'run ended; the conveyors run freely'
        return response

    def _on_reset(self, request, response):
        with self.lock:
            self._clear_cell_state()
            with self.changed:
                try:
                    joints = self.current_joints()
                except StepError:
                    joints = None
                self.run_state = self.core.initial_state(joints)
                self.check_state = None
                self.in_run, self.budget = True, 0.0     # wait for the next run
                self.last_tick = None
                for name, pose in self.core.movable_poses.items():
                    self.scene_poses[name] = None     # send the start poses again
                    if name in self.gz_poses:
                        self.gz_poses[name] = None
                parts = list(self.run_state.parts)
                self._clear_markers()
                self.changed.notify_all()
            self._show_parts(parts)
        response.success = True
        response.message = 'cell reset; the conveyors wait for the next run'
        return response

    # ------------------------------------------------------------------ move
    def _on_move(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            self.step_count += 1
            report = new_report(goal.name or 'move', 'move', goal.check_only)
            state, missed = None, []
            try:
                state = self._step_state(goal.check_only, snapshot=True)
                missed = list(self.core.conveyors.missed(state))
                report.start_time, report.start_joints = state.time, list(state.joints)
                self._move(goal, state, report, goal_handle)
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of dropping the goal
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            finally:
                if not goal.check_only:
                    self._end_step()
            if state is not None:
                report.end_joints = list(self._state_of(goal.check_only).joints)
            self._finish(report, missed)
        result = Move.Result(success=report.status != StepReport.FAILED, report=report)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _state_of(self, check_only):
        return self.check_state if check_only else self.run_state

    def _move(self, goal, state, report, goal_handle):
        goal_handle.publish_feedback(Move.Feedback(phase='check'))
        plan = self.core.plan_goal(state, goal, self.default_speed, report)
        self._add_path_marker(report, plan.check)
        if report.status == StepReport.FAILED:
            return
        if goal.check_only:
            self.core.advance(state, plan.profile.duration)
            self.core.complete_move(state, plan.goal)
            return
        self._grant(plan.profile.duration)      # the conveyors run while the robot moves
        goal_handle.publish_feedback(Move.Feedback(phase='execute'))
        self._execute(plan, goal_handle)
        if self.in_run:
            self._flush_budget()
        end = self.current_joints()
        error = float(np.max(np.abs(np.asarray(end) - np.asarray(plan.goal))))
        if error > self.execution_tolerance:
            report.status = max(report.status, StepReport.WARNING)
            report.message += f'; tracking error {math.degrees(error):.1f} deg'
        with self.changed:
            self.core.complete_move(self.run_state, end)

    def _execute(self, plan, goal_handle):
        sent = self._send_trajectory(plan)
        if sent is not None:
            self._wait_trajectory(*sent, goal_handle)

    def _send_trajectory(self, plan):
        """Send the trajectory of a plan to the controller; return what to wait for, or None."""
        times, positions, velocities, accelerations = plan.trajectory(0.02)
        trajectory = JointTrajectory(joint_names=JOINTS)
        for index in range(1, len(times)):
            t = float(times[index])
            point = JointTrajectoryPoint(
                positions=list(positions[index]), velocities=list(velocities[index]),
                time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9)))
            if accelerations is not None:
                point.accelerations = list(accelerations[index])
            trajectory.points.append(point)
        display = DisplayTrajectory(model_id=self.cell.robot.model)
        display.trajectory_start.joint_state = JointState(name=JOINTS, position=list(plan.start))
        display.trajectory.append(RobotTrajectory(joint_trajectory=trajectory))
        self.display_pub.publish(display)
        if not trajectory.points:
            return None
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise StepError(f'controller {self.controller} is not available')
        send = self.trajectory_client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=trajectory))
        handle = self._wait(send, 10.0)
        if handle is None or not handle.accepted:
            raise StepError('the trajectory was rejected by the controller')
        return handle, handle.get_result_async(), time.monotonic() + plan.profile.duration + 15.0

    def _wait_trajectory(self, handle, result_future, deadline, goal_handle):
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                raise StepError('cancelled')
            if time.monotonic() > deadline:
                raise StepError('the controller did not finish the trajectory')
            time.sleep(0.02)
        if result_future.result().result.error_code != 0:
            raise StepError('controller error: ' + result_future.result().result.error_string)

    def _on_pick_moving(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            self.step_count += 1
            report = new_report(goal.name or 'pick on the move', 'pick', goal.check_only)
            state, missed = None, []
            try:
                state = self._step_state(goal.check_only, snapshot=True)
                missed = list(self.core.conveyors.missed(state))
                report.start_time, report.start_joints = state.time, list(state.joints)
                goal_handle.publish_feedback(PickMoving.Feedback(phase='check'))
                plan = self.core.plan_pick_goal(state, goal, self.default_speed, report)
                self._add_path_marker(report, plan.check)
                if report.status != StepReport.FAILED:
                    if goal.check_only:
                        self.core.complete_pick(state, plan)
                    else:
                        goal_handle.publish_feedback(PickMoving.Feedback(phase='execute'))
                        self._execute_pick(plan, goal_handle, report)
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of dropping the goal
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            finally:
                if not goal.check_only:
                    self._end_step()
            if state is not None:
                report.end_joints = list(self._state_of(goal.check_only).joints)
            self._finish(report, missed)
        result = PickMoving.Result(success=report.status != StepReport.FAILED, report=report)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_pick(self, plan, goal_handle, report):
        """Run a pick on the move: the gripper closes at the planned cell time."""
        sent = self._send_trajectory(plan)
        with self.changed:
            start = self.run_state.time
            self.budget = plan.grip_at       # the belt runs exactly up to the grip
            while self.run_state.time < start + plan.grip_at - 1e-9:
                if goal_handle.is_cancel_requested:
                    if sent is not None:
                        sent[0].cancel_goal_async()
                    raise StepError('cancelled')
                self.changed.wait(0.1)
            self.core.hold(self.run_state, plan.part, plan.offset)
        self._move_group_attach(plan.part, attach=True)
        self._grant(plan.path.duration - plan.grip_at)
        if sent is not None:
            self._wait_trajectory(*sent, goal_handle)
        if self.in_run:
            self._flush_budget()
        end = self.current_joints()
        error = float(np.max(np.abs(np.asarray(end) - np.asarray(plan.goal))))
        if error > self.execution_tolerance:
            report.status = max(report.status, StepReport.WARNING)
            report.message += f'; tracking error {math.degrees(error):.1f} deg'
        with self.changed:
            self.core.complete_move(self.run_state, end)

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
            missed = None
            try:
                state = self._step_state(request.check_only)
                missed = list(self.core.conveyors.missed(state))
                with self.changed:
                    report.start_time = state.time
                    report.start_joints = report.end_joints = list(state.joints)
                    if request.close:
                        result = self.core.grip(state, report)
                    else:
                        result = self.core.release(state, report)
                if result and not request.check_only:
                    if request.close:
                        self._move_group_attach(result, attach=True)
                    else:
                        name, pose = result
                        self._move_group_attach(name, attach=False)
                        self.scene_poses[name] = None   # send the release pose to MoveIt
                        if self.gazebo is not None and name in self.gz_poses:
                            self.gazebo.set_pose(name, pose)
                            self.gz_poses[name] = pose
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of failing the service
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            finally:
                if not request.check_only:
                    self._end_step()
            self._finish(report, missed)
        response.success = report.status != StepReport.FAILED
        response.report = report
        return response

    def _move_group_attach(self, name, attach):
        """Attach an object to the TCP in the MoveIt scene, where the executor holds it."""
        attached = AttachedCollisionObject(link_name='tcp', touch_links=HELD_TOUCH_LINKS)
        offset = self.run_state.held.get(name) if attach else None
        if offset is not None:
            # give the shape and the pose: MoveIt's own robot state lags behind a fast robot
            attached.object = collision_object(self.core.definition(name, self.run_state.parts))
            attached.object.header.frame_id = 'tcp'
            attached.object.pose = pose_msg(_matrix_pose(offset))
        attached.object.id = name
        attached.object.operation = attached.object.ADD if attach else attached.object.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        response = self._call(self.scene_client, ApplyPlanningScene.Request(scene=scene), 10.0)
        if response is None or not response.success:
            self.get_logger().warning(f"MoveIt did not {'attach' if attach else 'detach'} {name}")

    # ------------------------------------------------------------------ waits and IO
    def _on_wait(self, request, response):
        report = new_report(request.name or 'wait', 'wait', request.check_only)
        with self.lock:
            self.step_count += 1
            state, missed = None, None
            try:
                state = self._step_state(request.check_only, snapshot=True)
                missed = list(self.core.conveyors.missed(state))
                report.start_time = state.time
                report.start_joints = report.end_joints = list(state.joints)
                plan = self.core.plan_wait(state, report, request.io, request.value,
                                           request.seconds, request.check_only)
                if plan.check is not None:
                    self._add_path_marker(report, plan.check)
                if report.status != StepReport.FAILED:
                    if request.check_only:
                        self.core.advance(state, plan.duration)
                    else:
                        self._grant(math.inf if plan.duration is None else plan.duration)
                        self._wait_run(request, plan, report)
            except StepError as exc:
                report.status, report.message = StepReport.FAILED, str(exc)
            except Exception as exc:  # report internal errors instead of failing the service
                self.get_logger().error(traceback.format_exc())
                report.status, report.message = StepReport.FAILED, f'internal error: {exc}'
            finally:
                if not request.check_only:
                    self._end_step()
            self._finish(report, missed)
        response.success = report.status != StepReport.FAILED
        response.report = report
        return response

    def _wait_run(self, request, plan, report):
        """Wait in the simulation until the time has passed or the IO has the value."""
        target = 'true' if request.value else 'false'
        with self.changed:
            start = self.run_state.time
            while True:
                elapsed = self.run_state.time - start
                if request.io and self.run_state.io.get(request.io, False) == request.value:
                    break
                if not request.io and elapsed >= request.seconds - 1e-6:
                    break
                if request.io and request.seconds > 0.0 and elapsed >= request.seconds:
                    raise StepError(f'timed out after {request.seconds:.1f} s waiting for '
                                    f'{request.io} = {target}')
                self.changed.wait(0.2)
            report.duration = self.run_state.time - start
        if request.io and plan.duration is None:
            report.message = f'{request.io} = {target} after {report.duration:.2f} s'

    def _on_set_io(self, request, response):
        report = new_report(request.name or f'set {request.io}', 'io', request.check_only)
        try:
            with self.changed:
                if request.check_only:
                    if self.check_state is None:
                        self._begin_check()
                    state = self.check_state
                else:
                    state = self.run_state
                report.start_time = state.time
                self.core.set_io(state, request.io, request.value, report)
                self.changed.notify_all()
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        self._finish(report)
        response.success = report.status != StepReport.FAILED
        response.report = report
        return response

    # ------------------------------------------------------------------ reports
    def _finish(self, report, missed_before=None):
        state = self._state_of(report.check_only)
        if state is not None:
            report.held_object = next(iter(state.held), '')
            if missed_before is not None:
                report.missed_parts = [part for part in self.core.conveyors.missed(state)
                                       if part not in missed_before]
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
    executor = MultiThreadedExecutor(num_threads=8)
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
