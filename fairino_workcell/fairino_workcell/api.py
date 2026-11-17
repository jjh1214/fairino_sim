"""
Python API for work cell recipes.

A recipe is a Python function that receives a Workcell::

    def recipe(cell):
        cell.move_joints([-45, -90, -90, -90, 90, 0], name='home')        # degrees
        cell.wait_io('conveyor/part_ready')                               # a part at the station
        cell.move_to('conveyor/part/grasp', name='pick')                  # wherever it lies
        cell.grip()
        cell.move_line('tcp', xyz=(0, 0, -0.1), name='lift')              # 10 cm up the tool axis

Run it in the simulation, or with --check to validate every step without moving the robot::

    ros2 run fairino_workcell run_recipe my_recipe.py [--check] [--reset] [--report report.json]

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

from action_msgs.msg import GoalStatus
from fairino_workcell.cell import load_cell
from fairino_workcell.motion import matrix_quaternion, rpy_matrix
from fairino_workcell.robot_cell import DEFAULT_SPEED, new_report, RobotCell, StepError
from fairino_workcell_msgs.action import Move, PickMoving
from fairino_workcell_msgs.msg import StepReport
from fairino_workcell_msgs.srv import Gripper, SetIO, Wait
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from std_srvs.srv import Trigger

STATUS = {StepReport.OK: 'OK', StepReport.WARNING: 'WARNING', StepReport.FAILED: 'FAILED'}
DOWN = (180.0, 0.0, 0.0)          # default orientation: the tool points down onto the frame


def _rpy(frame, rpy_deg):
    """Return the orientation of a target: by default down onto a frame, unchanged on 'tcp'."""
    if rpy_deg is not None:
        return rpy_deg
    return (0.0, 0.0, 0.0) if frame == 'tcp' else DOWN


def _pose_stamped(frame, xyz, rpy_deg):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in xyz)
    o = pose.pose.orientation
    o.x, o.y, o.z, o.w = matrix_quaternion(rpy_matrix(*(math.radians(v) for v in rpy_deg)))
    return pose


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
        'start_time': round(msg.start_time, 3), 'duration': round(msg.duration, 3),
        'min_clearance': round(msg.min_clearance, 4) if msg.min_clearance >= 0 else None,
        'closest_link': msg.closest_link, 'closest_object': msg.closest_object,
        'collisions': list(msg.collisions),
        'collision_at': round(msg.collision_at, 3) if msg.collision_at >= 0 else None,
        'allowed_contacts': list(msg.allowed_contacts),
        'start_joints_deg': [round(math.degrees(v), 2) for v in msg.start_joints],
        'end_joints_deg': [round(math.degrees(v), 2) for v in msg.end_joints],
        'held_object': msg.held_object, 'missed_parts': list(msg.missed_parts),
    }


class Workcell:
    """Client of the work cell executor; one instance per recipe run."""

    def __init__(self, check_only=False, speed=None, stop_on_failure=True, timeout=30.0,
                 reset=False):
        """Connect to the executor; reset first puts the cell back to its start."""
        self._start_run('check' if check_only else 'simulation', speed, stop_on_failure)
        self._own_context = not rclpy.ok()
        if self._own_context:
            rclpy.init()
        self.node = rclpy.create_node('workcell_recipe')
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._move = ActionClient(self.node, Move, '/workcell/move')
        self._pick_moving = ActionClient(self.node, PickMoving, '/workcell/pick_moving')
        self._gripper = self.node.create_client(Gripper, '/workcell/gripper')
        self._wait_service = self.node.create_client(Wait, '/workcell/wait')
        self._set_io = self.node.create_client(SetIO, '/workcell/set_io')
        if not (self._move.wait_for_server(timeout_sec=timeout)
                and self._gripper.wait_for_service(timeout_sec=timeout)):
            self.close()
            raise RuntimeError('the work cell executor is not running; start '
                               'fairino_workcell.launch.py first')
        services = (['/workcell/reset'] if reset else []) + [
            '/workcell/check/begin' if check_only else '/workcell/run/begin']
        for service in services:
            response = self._call(self.node.create_client(Trigger, service), Trigger.Request())
            if response is None or not response.success:
                self.close()
                raise RuntimeError(f'{service} failed: '
                                   + (response.message if response else 'no answer'))
        self._started = True

    def _start_run(self, mode, speed, stop_on_failure):
        self.mode = mode
        self.check_only = mode != 'simulation'
        self.speed = speed
        self.stop_on_failure = stop_on_failure
        self.reports = []
        self.on_step = None     # called with (step number, report) after every step

    # ------------------------------------------------------------------ steps
    def move_joints(self, joints, degrees=True, speed=None, name=None, allow_contact=()):
        """Move to joint angles j1..j6 (degrees by default) with a PTP motion."""
        values = [math.radians(v) if degrees else float(v) for v in joints]
        goal = Move.Goal(target_type=Move.Goal.JOINTS, joints=values)
        return self._run_move(goal, speed, name, allow_contact)

    def move_to(self, frame, xyz=(0.0, 0.0, 0.0), rpy_deg=None, speed=None, name=None,
                allow_contact=()):
        """
        Move the TCP to a pose given in a cell frame, with a PTP motion.

        Frames are 'world', objects and parts, 'object/frame' such as 'table/place',
        'conveyor/part' (the part a conveyor has ready, wherever it lies) with its frames such
        as 'conveyor/part/grasp', and 'tcp' (the TCP where the move starts, for moves in the
        tool frame: its z axis points out of the tool). The default orientation points the tool
        down onto the frame (180 degrees about x), and keeps it on 'tcp'.
        """
        return self.move_pose(_pose_stamped(frame, xyz, _rpy(frame, rpy_deg)), speed,
                              name or f'to {frame}', allow_contact)

    def move_pose(self, pose, speed=None, name=None, allow_contact=(), motion=Move.Goal.PTP):
        """Move the TCP to a geometry_msgs/PoseStamped (frame: world or a cell frame)."""
        goal = Move.Goal(target_type=Move.Goal.POSE, pose=pose, motion=motion)
        return self._run_move(goal, speed, name, allow_contact)

    def move_line(self, frame, xyz=(0.0, 0.0, 0.0), rpy_deg=None, speed=None, name=None,
                  allow_contact=()):
        """
        Move the TCP in a straight line (like MoveL) to a pose given in a cell frame.

        Frames and the default orientation are as for move_to; move_line('tcp', xyz=(0, 0,
        -0.1)) moves the tool 10 cm back along its axis. speed is a fraction of the TCP speed
        limits. The orientation turns uniformly on the way; the move fails where the arm would
        pass a singularity or leave a joint limit.
        """
        goal = Move.Goal(target_type=Move.Goal.POSE, motion=Move.Goal.LIN,
                         pose=_pose_stamped(frame, xyz, _rpy(frame, rpy_deg)))
        return self._run_move(goal, speed, name or f'line to {frame}', allow_contact)

    def move_arc(self, frame, via, xyz, rpy_deg=None, speed=None, name=None, allow_contact=()):
        """Move the TCP along the arc through via to xyz, both in frame (like MoveC)."""
        goal = Move.Goal(target_type=Move.Goal.POSE, motion=Move.Goal.CIRC,
                         pose=_pose_stamped(frame, xyz, _rpy(frame, rpy_deg)),
                         via=_pose_stamped(frame, via, (0.0, 0.0, 0.0)))
        return self._run_move(goal, speed, name or f'arc to {frame}', allow_contact)

    def pick_moving(self, conveyor, approach=0.1, lift=0.1, rpy_deg=DOWN, speed=None,
                    grip_time=None, part='', name=None):
        """
        Pick a part on the move from a continuous conveyor (conveyor tracking).

        One checked motion: the TCP comes in step with the part approach metres above its
        grasp pose (rpy_deg relative to the part's grasp frame, so it turns with the part),
        goes down to it while following it, follows it while the gripper closes (grip_time
        seconds; by default as long as the fingers need), lifts it lift metres while still
        following the belt, and stops. The part is the first one past the conveyor sensor
        unless part is given; afterwards it is in the gripper.
        """
        goal = PickMoving.Goal(conveyor=conveyor, part=part, approach=float(approach),
                               lift=float(lift),
                               grip_time=-1.0 if grip_time is None else float(grip_time),
                               speed=float(speed or self.speed or 0.0),
                               check_only=self.check_only, name=name or f'pick from {conveyor}')
        goal.grasp = _pose_stamped('', (0.0, 0.0, 0.0), rpy_deg).pose
        return self._record(self._send_pick_moving(goal))

    def grip(self, name=None):
        """
        Close the gripper on the object at the TCP.

        Fingers close until they touch the object between them (the one whose grasp frame is
        nearest to the TCP) and centre it; a tool without fingers takes the object whose grasp
        frame is within the grasp tolerance of the TCP.
        """
        return self._gripper_step(True, name or 'grip')

    def release(self, name=None):
        """Open the gripper (fully) and leave the held object where it is."""
        return self._gripper_step(False, name or 'release')

    def set_io(self, io, value, name=None):
        """Set a virtual IO, such as an output to another machine (check: in the session only)."""
        return self._record(self._send_set_io(io, bool(value), name or f'set {io}'))

    def wait_io(self, io, value=True, timeout=None, name=None):
        """
        Wait until a virtual IO has the value, such as 'conveyor/part_ready'.

        Conveyor sensors are simulated, also in a check. IO that other programs set is
        assumed in a check. A timeout (seconds) fails the step when it expires.
        """
        return self._record(self._send_wait(io, bool(value), float(timeout or 0.0),
                                            name or f'wait {io}'))

    def wait(self, seconds, name=None):
        """Wait for a time (virtual in a check); conveyors run on meanwhile."""
        return self._record(self._send_wait('', True, float(seconds), name or 'wait'))

    # ------------------------------------------------------------------ reports
    @property
    def cycle_time(self):
        return sum(r['duration'] for r in self.reports)

    def summary(self):
        """Return counts and times of the run: cycle time, waiting, parts gripped."""
        counts = {status: sum(r['status'] == status for r in self.reports)
                  for status in STATUS.values()}
        grips = [r['start_time'] for r in self.reports if r['kind'] in ('grip', 'pick')
                 and r['status'] != 'FAILED']
        intervals = [b - a for a, b in zip(grips, grips[1:])]
        return {'mode': self.mode, 'cycle_time': round(self.cycle_time, 3),
                'waiting': round(sum(r['duration'] for r in self.reports
                                     if r['kind'] == 'wait'), 3),
                'grips': len(grips),
                'missed': sum(len(r['missed_parts']) for r in self.reports),
                'grip_interval': round(sum(intervals) / len(intervals), 3) if intervals else None,
                'steps': len(self.reports), **{k.lower(): v for k, v in counts.items()}}

    def report_header(self):
        """Return the title and the column header of the report table."""
        title = {'simulation': 'simulation', 'check': 'check only, the robot did not move',
                 'offline': 'offline check against the cell file'}[self.mode]
        return (f'Recipe report ({title})\n'
                f"{'#':>3}  {'step':<19}{'kind':<9}{'result':<9}{'time':>7}  {'clearance':>9}  "
                f"{'closest':<26}message")

    @staticmethod
    def report_row(index, r):
        """Return the report table row of a step."""
        clearance = '-' if r['min_clearance'] is None else f"{r['min_clearance'] * 1000:.0f} mm"
        closest = f"{r['closest_link']} / {r['closest_object']}" if r['closest_object'] else ''
        return (f"{index:>3}  {r['name'][:18]:<19}{r['kind']:<9}{r['status']:<9}"
                f"{r['duration']:>6.2f}s  {clearance:>9}  {closest[:25]:<26}{r['message']}")

    def report_summary(self):
        s = self.summary()
        text = (f"Cycle time {s['cycle_time']:.2f} s | {s['steps']} steps: {s['ok']} OK, "
                f"{s['warning']} warnings, {s['failed']} failed")
        if s['waiting']:
            text += f" | waiting {s['waiting']:.2f} s"
        if s['grip_interval'] is not None:
            text += f" | {s['grips']} parts gripped, one every {s['grip_interval']:.2f} s"
        if s['missed']:
            text += f" | {s['missed']} part{'s' if s['missed'] > 1 else ''} missed"
        return text

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

    def _send_pick_moving(self, goal):
        handle = self._wait(self._pick_moving.send_goal_async(goal), 30.0)
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

    def _send_wait(self, io, value, seconds, name):
        request = Wait.Request(io=io, value=value, seconds=seconds, check_only=self.check_only,
                               name=name)
        timeout = None if self.mode == 'simulation' else 120.0   # the simulation may wait long
        response = self._call(self._wait_service, request, timeout)
        if response is None:
            raise RuntimeError(f"step '{name}' got no answer from the executor")
        return response.report

    def _send_set_io(self, io, value, name):
        response = self._call(self._set_io, SetIO.Request(
            io=io, value=value, check_only=self.check_only, name=name))
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
        if not client.wait_for_service(timeout_sec=30.0):
            return None
        return self._wait(client.call_async(request), timeout)

    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    def _spin(self):
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def close(self):
        """End the run (the conveyors run freely again) and disconnect."""
        if self.mode == 'simulation' and getattr(self, '_started', False):
            self._started = False
            self._call(self.node.create_client(Trigger, '/workcell/run/end'), Trigger.Request(),
                       5.0)
        self._executor.shutdown(timeout_sec=5.0)
        self._thread.join(timeout=5.0)      # spin must return before the node goes away
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
    from the start of the cell (initial joints unless joints in radians are given), in virtual
    time like a check session.
    """

    def __init__(self, cell_file, speed=None, stop_on_failure=True, joints=None, **settings):
        self._start_run('offline', speed, stop_on_failure)
        self.core = RobotCell(load_cell(cell_file), **settings)
        self.state = self.core.initial_state(joints)

    def _send_move(self, goal):
        report = self._report(goal.name, 'move')
        try:
            plan = self.core.plan_goal(self.state, goal, DEFAULT_SPEED, report)
            if report.status != StepReport.FAILED:
                self.core.advance(self.state, plan.profile.duration)
                self.core.complete_move(self.state, plan.goal)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _send_pick_moving(self, goal):
        report = self._report(goal.name, 'pick')
        try:
            plan = self.core.plan_pick_goal(self.state, goal, DEFAULT_SPEED, report)
            if report.status != StepReport.FAILED:
                self.core.complete_pick(self.state, plan)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _send_gripper(self, close, name):
        report = self._report(name, 'grip' if close else 'release')
        try:
            if close:
                self.core.grip(self.state, report)
            else:
                self.core.release(self.state, report)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _send_wait(self, io, value, seconds, name):
        report = self._report(name, 'wait')
        try:
            plan = self.core.plan_wait(self.state, report, io, value, seconds, check_only=True)
            if report.status != StepReport.FAILED:
                self.core.advance(self.state, plan.duration)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _send_set_io(self, io, value, name):
        report = self._report(name, 'io')
        try:
            self.core.set_io(self.state, io, value, report)
        except StepError as exc:
            report.status, report.message = StepReport.FAILED, str(exc)
        return self._finish(report)

    def _report(self, name, kind):
        report = new_report(name, kind, True, self.state.time)
        report.start_joints = list(self.state.joints)
        self._missed_before = list(self.core.conveyors.missed(self.state))
        return report

    def _finish(self, report):
        report.end_joints = list(self.state.joints)
        report.held_object = next(iter(self.state.held), '')
        report.missed_parts = [part for part in self.core.conveyors.missed(self.state)
                               if part not in self._missed_before]
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
    parser.add_argument('--reset', action='store_true',
                        help='first put the cell back to its start (objects, parts, conveyors)')
    args = parser.parse_args(argv)
    code = 0
    if args.offline:
        cell = OfflineWorkcell(args.offline, speed=args.speed,
                               stop_on_failure=not args.keep_going)
    else:
        cell = Workcell(check_only=args.check, speed=args.speed,
                        stop_on_failure=not args.keep_going, reset=args.reset)
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
        print('usage: run_recipe RECIPE.py [--check | --offline CELL_FILE] [--reset] '
              '[--speed S] [--report FILE] [--keep-going]', file=sys.stderr)
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
