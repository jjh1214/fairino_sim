"""
Python API for work cell recipes.

A recipe is a Python function that receives a Workcell::

    def recipe(cell):
        cell.move_joints([0, -90, 0, 0, 0, 0], name='home')            # degrees
        cell.move_to('part/grasp', xyz=(0, 0, 0.15), name='above part')  # TCP above a cell frame
        cell.move_to('part/grasp', speed=0.1, allow_contact=['part'], name='pick')
        cell.grip()

Run it in the simulation, or with --check to validate every step without moving the robot::

    ros2 run fairino_workcell run_recipe my_recipe.py [--check] [--report report.json]

Every step is checked before it runs and returns a report (a dict). A failed step raises
RecipeError unless stop_on_failure is False. Workcell only uses the ROS interfaces of
fairino_workcell_msgs, so other programs (vision, IO, upper systems) can drive the same cell.

OfflineWorkcell checks the same recipe against a cell file without ROS or the simulation
(run_recipe my_recipe.py --offline cell.yaml), for example in automated tests.
"""

import argparse
import importlib.util
import json
import math
import os
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from fairino_workcell.cell import load_cell
from fairino_workcell.motion import matrix_quaternion, pose_matrix, rpy_matrix
from fairino_workcell.robot_cell import DEFAULT_SPEED, new_report, RobotCell, StepError
from fairino_workcell_msgs.action import Move
from fairino_workcell_msgs.msg import IOState, StepReport
from fairino_workcell_msgs.srv import Gripper, SetIO
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

STATUS = {StepReport.OK: 'OK', StepReport.WARNING: 'WARNING', StepReport.FAILED: 'FAILED'}


class RecipeError(RuntimeError):
    """A recipe step failed; .report holds its report."""

    def __init__(self, report):
        super().__init__(f"step '{report['name']}' failed: {report['message']}")
        self.report = report


def report_dict(msg):
    """Return a StepReport message as a plain dict."""
    return {
        'name': msg.name, 'kind': msg.kind, 'status': STATUS[msg.status],
        'message': msg.message, 'check_only': msg.check_only,
        'duration': round(msg.duration, 3),
        'min_clearance': round(msg.min_clearance, 4) if msg.min_clearance >= 0 else None,
        'closest_link': msg.closest_link, 'closest_object': msg.closest_object,
        'collisions': list(msg.collisions),
        'collision_at': round(msg.collision_at, 3) if msg.collision_at >= 0 else None,
        'allowed_contacts': list(msg.allowed_contacts),
        'start_joints_deg': [round(math.degrees(v), 2) for v in msg.start_joints],
        'end_joints_deg': [round(math.degrees(v), 2) for v in msg.end_joints],
        'held_object': msg.held_object,
    }


class Workcell:
    """Client of the work cell executor; one instance per recipe run."""

    def __init__(self, check_only=False, speed=None, stop_on_failure=True, timeout=30.0):
        self._start_run('check' if check_only else 'simulation', speed, stop_on_failure)
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self.node = rclpy.create_node('workcell_recipe')
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._io = {}
        self._io_changed = threading.Condition()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(IOState, '/workcell/io', self._on_io, latched)
        self._move = ActionClient(self.node, Move, '/workcell/move')
        self._gripper = self.node.create_client(Gripper, '/workcell/gripper')
        self._set_io = self.node.create_client(SetIO, '/workcell/set_io')
        if not (self._move.wait_for_server(timeout_sec=timeout)
                and self._gripper.wait_for_service(timeout_sec=timeout)):
            self.close()
            raise RuntimeError('the work cell executor is not running; start '
                               'fairino_workcell.launch.py first')
        begin = self.node.create_client(
            Trigger, '/workcell/check/begin' if check_only else '/workcell/run/begin')
        response = self._call(begin, Trigger.Request())
        if response is None or not response.success:
            self.close()
            raise RuntimeError(f'could not start the {self.mode}: '
                               + (response.message if response else 'no answer'))

    def _start_run(self, mode, speed, stop_on_failure):
        self.mode = mode
        self.check_only = mode != 'simulation'
        self.speed = speed
        self.stop_on_failure = stop_on_failure
        self.reports = []
        self.waits = 0.0
        self.on_step = None     # called with (step number, report) after every step

    # ------------------------------------------------------------------ steps
    def move_joints(self, joints, degrees=True, speed=None, name=None, allow_contact=()):
        """Move to joint angles j1..j6 (degrees by default) with a PTP motion."""
        values = [math.radians(v) if degrees else float(v) for v in joints]
        goal = Move.Goal(target_type=Move.Goal.JOINTS, joints=values)
        return self._run_move(goal, speed, name, allow_contact)

    def move_to(self, frame, xyz=(0.0, 0.0, 0.0), rpy_deg=(180.0, 0.0, 0.0), speed=None,
                name=None, allow_contact=()):
        """
        Move the TCP to a pose given in a cell frame, such as 'part/grasp'.

        The default orientation points the tool down onto the frame (180 degrees about x).
        """
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(v) for v in xyz)
        q = matrix_quaternion(rpy_matrix(*(math.radians(v) for v in rpy_deg)))
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        return self.move_pose(pose, speed, name or f'to {frame}', allow_contact)

    def move_pose(self, pose, speed=None, name=None, allow_contact=()):
        """Move the TCP to a geometry_msgs/PoseStamped (frame: world or a cell frame)."""
        goal = Move.Goal(target_type=Move.Goal.POSE, pose=pose)
        return self._run_move(goal, speed, name, allow_contact)

    def grip(self, name=None):
        """Close the gripper on the object whose grasp frame is at the TCP."""
        return self._gripper_step(True, name or 'grip')

    def release(self, name=None):
        """Open the gripper and leave the held object where it is."""
        return self._gripper_step(False, name or 'release')

    def set_io(self, name, value):
        """Set a virtual digital output (no effect in a check run)."""
        if self.check_only:
            return True
        response = self._call(self._set_io, SetIO.Request(name=name, value=bool(value)))
        return bool(response and response.success)

    def wait_io(self, name, value=True, timeout=None):
        """Wait until a virtual IO has the value (returns at once in a check run)."""
        if self.check_only:
            return True
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._io_changed:
            while self._io.get(name) != value:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return False
                self._io_changed.wait(remaining if remaining is not None else 0.5)
        return True

    def wait(self, seconds):
        """Wait (counted in the cycle time; not slept in a check run)."""
        self.waits += seconds
        if not self.check_only:
            time.sleep(seconds)

    # ------------------------------------------------------------------ reports
    @property
    def cycle_time(self):
        return sum(r['duration'] for r in self.reports) + self.waits

    def summary(self):
        counts = {status: sum(r['status'] == status for r in self.reports)
                  for status in STATUS.values()}
        return {'mode': self.mode,
                'cycle_time': round(self.cycle_time, 3), 'waits': round(self.waits, 3),
                'steps': len(self.reports), **{k.lower(): v for k, v in counts.items()}}

    def report_header(self):
        """Return the title and the column header of the report table."""
        title = {'simulation': 'simulation', 'check': 'check only, the robot did not move',
                 'offline': 'offline check against the cell file'}[self.mode]
        return (f'Recipe report ({title})\n'
                f"{'#':>3}  {'step':<16}{'kind':<9}{'result':<9}{'time':>7}  {'clearance':>9}  "
                f"{'closest':<26}message")

    @staticmethod
    def report_row(index, r):
        """Return the report table row of a step."""
        clearance = '-' if r['min_clearance'] is None else f"{r['min_clearance'] * 1000:.0f} mm"
        closest = f"{r['closest_link']} / {r['closest_object']}" if r['closest_object'] else ''
        return (f"{index:>3}  {r['name'][:15]:<16}{r['kind']:<9}{r['status']:<9}"
                f"{r['duration']:>6.2f}s  {clearance:>9}  {closest[:25]:<26}{r['message']}")

    def report_summary(self):
        s = self.summary()
        return (f"Cycle time {s['cycle_time']:.2f} s | {s['steps']} steps: {s['ok']} OK, "
                f"{s['warning']} warnings, {s['failed']} failed")

    def print_report(self, stream=sys.stdout):
        print('\n' + self.report_header(), file=stream)
        for index, report in enumerate(self.reports, 1):
            print(self.report_row(index, report), file=stream)
        print(self.report_summary(), file=stream)

    def save_report(self, path):
        with open(path, 'w') as stream:
            json.dump({'summary': self.summary(), 'steps': self.reports}, stream, indent=2)

    # ------------------------------------------------------------------ internals
    def _run_move(self, goal, speed, name, allow_contact):
        goal.speed = float(speed or self.speed or 0.0)
        goal.allowed_contacts = list(allow_contact)
        goal.check_only = self.check_only
        goal.name = name or f'move {len(self.reports) + 1}'
        return self._record(self._send_move(goal))

    def _gripper_step(self, close, name):
        return self._record(self._send_gripper(close, name))

    def _send_move(self, goal):
        handle = self._wait(self._move.send_goal_async(goal), 30.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(f"step '{goal.name}' was not accepted by the executor")
        result = self._wait(handle.get_result_async(), 600.0)
        if result is None:
            raise RuntimeError(f"step '{goal.name}' did not finish")
        if result.status != GoalStatus.STATUS_SUCCEEDED and not result.result.report.name:
            raise RuntimeError(f"step '{goal.name}' was aborted by the executor (see its log)")
        return result.result.report

    def _send_gripper(self, close, name):
        response = self._call(self._gripper, Gripper.Request(
            close=close, check_only=self.check_only, name=name), 60.0)
        if response is None:
            raise RuntimeError(f"step '{name}' got no answer from the executor")
        return response.report

    def _record(self, msg):
        report = report_dict(msg)
        self.reports.append(report)
        if self.on_step is not None:
            self.on_step(len(self.reports), report)
        if report['status'] == 'FAILED' and self.stop_on_failure:
            raise RecipeError(report)
        return report

    def _call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        return self._wait(client.call_async(request), timeout)

    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    def _on_io(self, msg):
        with self._io_changed:
            self._io = dict(zip(msg.names, msg.values))
            self._io_changed.notify_all()

    def _spin(self):
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def close(self):
        self._executor.shutdown()
        self.node.destroy_node()
        if self._own_context:
            rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class OfflineWorkcell(Workcell):
    """
    Check a recipe against a cell file without ROS or the simulation.

    The steps follow the same rules as in workcell_executor (see fairino_workcell.robot_cell),
    from the initial joints of the cell file unless joints (radians) are given. IO steps are
    skipped, like in a check run.
    """

    def __init__(self, cell_file, speed=None, stop_on_failure=True, joints=None, **settings):
        self._start_run('offline', speed, stop_on_failure)
        self.core = RobotCell(load_cell(cell_file), **settings)
        self.state = self.core.initial_state(joints)

    def _send_move(self, goal):
        report = new_report(goal.name, 'move', True)
        report.start_joints = list(self.state.joints)
        try:
            if goal.target_type == Move.Goal.JOINTS:
                target = tuple(goal.joints)
            else:
                p, o = goal.pose.pose.position, goal.pose.pose.orientation
                target = self.core.pose_target(self.state, goal.pose.header.frame_id,
                                               pose_matrix((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
            plan = self.core.plan_move(self.state, target, goal.speed or DEFAULT_SPEED,
                                       goal.allowed_contacts, report)
            if report.status != StepReport.FAILED:
                self.core.complete_move(self.state, plan.goal)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _send_gripper(self, close, name):
        report = new_report(name, 'grip' if close else 'release', True)
        report.start_joints = list(self.state.joints)
        try:
            if close:
                self.core.grip(self.state, report)
            else:
                self.core.release(self.state, report)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _finish(self, report):
        report.end_joints = list(self.state.joints)
        report.held_object = next(iter(self.state.held), '')
        return report

    def object_pose(self, name):
        """Return the 4x4 world pose of a movable object (at the TCP while held)."""
        return self.core.object_poses(self.state, self.state.joints)[0][name]

    def close(self):
        pass


def run(recipe, argv=None):
    """Run a recipe function from the command line (--check, --speed, --report, --keep-going)."""
    parser = argparse.ArgumentParser(description='Run a work cell recipe.')
    parser.add_argument('--check', action='store_true',
                        help='check every step without moving the robot')
    parser.add_argument('--offline', metavar='CELL_FILE',
                        help='check against a cell file, without ROS or the simulation')
    parser.add_argument('--speed', type=float, help='default speed, 0..1 of the joint limits')
    parser.add_argument('--report', help='write the report to this JSON file')
    parser.add_argument('--keep-going', action='store_true',
                        help='continue after a failed step (check runs)')
    args = parser.parse_args(argv)
    code = 0
    if args.offline:
        cell = OfflineWorkcell(args.offline, speed=args.speed,
                               stop_on_failure=not args.keep_going)
    else:
        cell = Workcell(check_only=args.check, speed=args.speed,
                        stop_on_failure=not args.keep_going)
    with cell:
        print(cell.report_header(), flush=True)     # the rows follow as the steps finish
        cell.on_step = lambda index, report: print(cell.report_row(index, report), flush=True)
        stopped = None
        try:
            recipe(cell)
        except RecipeError as exc:
            stopped = exc
        finally:
            print(cell.report_summary(), flush=True)
            if args.report:
                cell.save_report(args.report)
        if stopped is not None:
            print(f'Stopped: {stopped}', file=sys.stderr)
        if any(r['status'] == 'FAILED' for r in cell.reports):
            code = 1
    return code


def main(argv=None):
    """Entry point of run_recipe: load a recipe file and run its recipe(cell) function."""
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0].startswith('-'):
        print('usage: run_recipe RECIPE.py [--check | --offline CELL_FILE] [--speed S] '
              '[--report FILE] [--keep-going]', file=sys.stderr)
        return 2
    path = os.path.abspath(argv[0])
    spec = importlib.util.spec_from_file_location('workcell_recipe', path)
    if spec is None or not os.path.isfile(path):
        print(f'run_recipe: cannot load {path}', file=sys.stderr)
        return 2
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, 'recipe', None)):
        print(f'run_recipe: {path} has no recipe(cell) function', file=sys.stderr)
        return 2
    return run(module.recipe, argv[1:])
