"""
A robot in its work cell, without ROS: kinematics, path checks, conveyors and step rules.

RobotCell checks recipe steps against a CellState (cell time, joints, object poses, held
object, virtual IO, conveyor belts). The workcell_executor node wraps it for ROS and executes
the steps in the simulation; OfflineWorkcell and the tests use it directly.

Rules of the steps:
- move: the target joints (given, or solved from a TCP pose nearest to the current joints) must
  be within the joint limits, and the PTP path must be free of collisions of the robot, the
  tool and the held object with the cell and with themselves. Parts on running conveyors are
  checked where they will be at each moment of the move. Contacts can be allowed per move
  with allowed_contacts (tool and held object against named objects, parts or part types).
- grip: takes the movable object whose 'grasp' frame is nearest to the TCP, within
  grasp_tolerance; a part taken from a conveyor station lets the conveyor run on. Lifting the
  object off its support is allowed in the next move.
- release: leaves the held object where it is. Moving the tool away from it is allowed in the
  next move.
- wait: for a time, or until an IO has a value. Conveyor sensors follow the conveyor model;
  other IO is set by the recipe or by other programs. Parts that move while the robot waits
  are checked against the robot.
- io: sets a virtual IO; conveyor sensors cannot be set.
"""

from dataclasses import dataclass, replace
import math
import xml.etree.ElementTree as ET

from fairino_sim import description
from fairino_workcell import cartesian, motion, tracking
from fairino_workcell.checker import Checker, PathCheck
from fairino_workcell.conveyor import ConveyorModel, Conveyors
from fairino_workcell.kinematics import Arm
from fairino_workcell.state import CellState
from fairino_workcell_msgs.action import Move
from fairino_workcell_msgs.msg import StepReport
import numpy as np

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
HELD_TOUCH_LINKS = ['tcp', 'tool_link', 'flange', 'wrist3_link']
DEFAULT_SPEED = 0.3       # fraction of the joint velocity and acceleration limits
WAIT_LIMIT = 600.0        # longest wait a check simulates when no timeout is given [s]
PART_STEP = 0.005         # largest distance a part moves between two check samples [m]
FAR = motion.transform((0.0, 0.0, -1000.0))    # where parts are before they are put in
TURN_SPEED, TURN_ACCELERATION = math.pi, 2.0 * math.pi    # TCP orientation limits (rad/s, /s2)
WRIST_WARNING = math.sin(math.radians(5.0))    # Cartesian moves closer to a singularity warn
ELBOW_WARNING = math.sin(math.radians(5.0))
SHOULDER_WARNING = 0.02                        # m
CONTACT_DISTANCE = 1e-4   # m: closer than this counts as touching, a collision unless allowed


class StepError(RuntimeError):
    """A step cannot be done; the message goes into the report."""


@dataclass
class MovePlan:
    """A checked move: PTP in joint space, or LIN and CIRC along a TCP path."""

    start: tuple
    goal: tuple
    profile: motion.Profile
    check: PathCheck
    path: cartesian.CartesianPath = None
    samples: list = None          # [(path fraction, joints)] of a LIN or CIRC move
    arm: Arm = None

    def trajectory(self, dt):
        """Return times, joint positions, velocities and accelerations (or None) every dt."""
        if self.path is None:
            return motion.sample_ptp(self.start, self.goal, self.profile, dt)
        times, positions, velocities = cartesian.trajectory(self.arm, self.path, self.profile,
                                                            self.samples, dt)
        return times, positions, velocities, None


@dataclass
class WaitPlan:
    """A checked wait."""

    duration: float          # predicted waiting time; None: until another program sets the IO
    check: PathCheck = None  # parts moving during the wait; None when nothing moves


@dataclass
class PickPlan:
    """A checked pick on the move from a continuous conveyor."""

    part: str
    start: tuple
    goal: tuple
    profile: tracking.Timeline
    check: PathCheck
    path: tracking.TrackingMotion
    samples: list                 # [(time fraction, joints)]
    arm: Arm
    grip_at: float                # s from the start of the step
    offset: np.ndarray            # part pose relative to the TCP once gripped

    def trajectory(self, dt):
        """Return times, joint positions, velocities and None (no accelerations) every dt."""
        times, positions, velocities = cartesian.trajectory(self.arm, self.path, self.profile,
                                                            self.samples, dt)
        return times, positions, velocities, None


def new_report(name, kind, check_only=False, start_time=0.0):
    return StepReport(name=name, kind=kind, status=StepReport.OK, check_only=check_only,
                      min_clearance=-1.0, collision_at=-1.0, start_time=float(start_time))


class RobotCell:
    """The robot of a cell file with its tool, and the objects and conveyors around it."""

    def __init__(self, cell, joint_acceleration=3.0, clearance_warning=0.01,
                 grasp_tolerance=0.01, check_step=math.radians(0.5), tcp_speed=1.0,
                 tcp_acceleration=2.0):
        robot = cell.robot
        tool = description.load_tool(robot.tool_file) if robot.tool_file else None
        package = description.moveit_config_package(robot.model)
        controllers = f'{description.share_dir(package)}/config/ros2_controllers.yaml'
        self.urdf = description.build_urdf(robot.model, controllers, base_xyz=robot.base.xyz,
                                           base_rpy=robot.base.rpy, tool=tool)
        joints = {j.get('name'): j for j in ET.fromstring(self.urdf).findall('joint')}
        limits = [joints[name].find('limit') for name in JOINTS]
        self.lower = [float(lim.get('lower')) for lim in limits]
        self.upper = [float(lim.get('upper')) for lim in limits]
        self.max_velocity = [float(lim.get('velocity')) for lim in limits]
        self.joint_acceleration = joint_acceleration
        self.clearance_warning = clearance_warning
        self.grasp_tolerance = grasp_tolerance
        self.check_step = check_step
        self.tcp_limits = (tcp_speed, tcp_acceleration, TURN_SPEED, TURN_ACCELERATION)

        self.cell = cell
        self.group = f'{robot.model}_group'
        self.tool_bodies = ['tool_link'] if tool is not None else ['wrist3_link']
        self.arm = Arm(self.urdf, JOINTS)    # world -> tcp
        self.checker = Checker(cell, self.urdf, description.build_srdf(robot.model, tool),
                               self.group, self.tool_bodies)
        self.objects = {obj.name: obj for obj in cell.objects}
        self.part_types = {part.name: part for part in cell.parts}
        poses = {obj.name: motion.transform(obj.pose.xyz, obj.pose.rpy) for obj in cell.objects}
        self.static_poses = {n: p for n, p in poses.items() if not self.objects[n].movable}
        self.movable_poses = {n: p for n, p in poses.items() if self.objects[n].movable}
        models = {}
        for obj in cell.objects:
            if obj.conveyor is not None:
                frames = {name: poses[obj.name] @ motion.transform(pose.xyz, pose.rpy)
                          for name, pose in obj.frames}
                spec = obj.conveyor
                models[obj.name] = ConveyorModel(obj.name, spec, frames[spec.entry],
                                                 frames[spec.station], frames.get(spec.exit_frame))
        self.conveyors = Conveyors(models)
        self.sensors = self.conveyors.sensors
        self._parts = {}

    def initial_state(self, joints=None):
        """Return the start of the cell: initial joints, objects where the file puts them."""
        joints = self.cell.robot.initial_positions if joints is None else joints
        state = CellState(tuple(float(v) for v in joints),
                          {n: p.copy() for n, p in self.movable_poses.items()})
        self.conveyors.start(state)     # the first parts go on the belts
        return state

    def definition(self, name, parts):
        """Return the CellObject of a cell object, or of a part put in by a conveyor."""
        if name in self.objects:
            return self.objects[name]
        key = (name, parts[name])
        if key not in self._parts:
            self._parts[key] = replace(self.part_types[parts[name]], name=name)
        return self._parts[key]

    # ---------------------------------------------------------------- conveyors
    def advance(self, state, dt):
        """Run the conveyors for dt seconds of cell time; return the names of new parts."""
        return self.conveyors.advance(state, dt)

    # ---------------------------------------------------------------- frames and targets
    def all_poses(self, state):
        poses = dict(self.static_poses)
        poses.update(state.poses)
        return poses

    def sync(self, state):
        parts = {name: self.definition(name, state.parts)
                 for name in list(state.poses) + list(state.held) if name in state.parts}
        self.checker.sync(self.all_poses(state), state.held, HELD_TOUCH_LINKS, parts)

    def frame_matrix(self, frame, state):
        """Return the 4x4 world pose of 'world', an object or part, or 'object/frame'."""
        if frame in ('', 'world'):
            return np.eye(4)
        name, _, sub = frame.partition('/')
        if name not in self.objects and name not in state.parts:
            raise StepError(f"unknown frame '{frame}'")
        if name in state.held:
            raise StepError(f"frame '{frame}' moves with the held object")
        matrix = self.all_poses(state)[name]
        if not sub:
            return matrix
        pose = self.definition(name, state.parts).frame(sub)
        if pose is None:
            raise StepError(f"object '{name}' has no frame '{sub}'")
        return matrix @ motion.transform(pose.xyz, pose.rpy)

    def solve_ik(self, target, reference):
        """
        Return the joints for a 4x4 TCP pose in world, nearest to the reference joints.

        Like the robot controller, the configuration (shoulder, elbow, wrist) follows from the
        reference, so the same step always gives the same joints. Collisions are not avoided
        here: the path check reports them.
        """
        solutions, outside = self.arm.inverse(target, reference, self.lower, self.upper)
        if solutions:
            return solutions[0]
        where = 'TCP at ({:.3f}, {:.3f}, {:.3f}) m'.format(*target[:3, 3])
        if outside:
            raise StepError(f'target is reachable only outside the joint limits: {where}')
        raise StepError(f'target is not reachable: {where}')

    def pose_target(self, state, frame, pose):
        """Return the joints that put the TCP at pose (4x4) relative to frame."""
        return self.solve_ik(self.frame_matrix(frame, state) @ pose, state.joints)

    # ---------------------------------------------------------------- steps
    def plan_goal(self, state, goal, default_speed, report):
        """Check the move of a Move goal (PTP, LIN or CIRC; joint or pose target)."""
        speed = goal.speed or default_speed
        pose = None
        if goal.target_type == Move.Goal.JOINTS:
            joints = tuple(float(v) for v in goal.joints)
            if len(joints) != 6:
                raise StepError(f'joints must have 6 values, got {len(joints)}')
        elif goal.target_type == Move.Goal.POSE:
            joints = None
            p, o = goal.pose.pose.position, goal.pose.pose.orientation
            pose = self.frame_matrix(goal.pose.header.frame_id, state) @ motion.pose_matrix(
                (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        else:
            raise StepError(f'unknown target_type {goal.target_type}')
        if goal.motion == Move.Goal.PTP:
            if joints is None:
                joints = self.solve_ik(pose, state.joints)
            return self.plan_move(state, joints, speed, goal.allowed_contacts, report)
        if goal.motion not in (Move.Goal.LIN, Move.Goal.CIRC):
            raise StepError(f'unknown motion {goal.motion}')
        via = None
        if goal.motion == Move.Goal.CIRC:
            v = goal.via.pose.position
            via = (self.frame_matrix(goal.via.header.frame_id, state)
                   @ motion.transform((v.x, v.y, v.z)))[:3, 3]
        if pose is None:
            problems = motion.outside_limits(joints, self.lower, self.upper, JOINTS)
            if problems:
                raise StepError('outside the joint limits: ' + ', '.join(problems))
            pose = self.arm.fk(joints)
        return self.plan_cartesian(state, pose, speed, goal.allowed_contacts, report, via, joints)

    def plan_cartesian(self, state, target, speed, allowed_contacts, report, via=None,
                       joints=None):
        """
        Check a LIN move (via None) or a CIRC move through via of the TCP to target.

        target is a 4x4 world pose, via a world point. joints, if given, are the joints the
        move must end at. The joints follow the path continuously from the current joints;
        passing a singularity or a joint limit fails, coming close to a singularity warns, and
        the move slows down where a joint would turn faster than its limit.
        """
        if not 0.0 < speed <= 1.0:
            raise StepError(f'speed must be in (0, 1], got {speed}')
        start = np.asarray(state.joints, dtype=float)
        try:
            path = cartesian.CartesianPath(self.arm.fk(start), target, via)
            samples = cartesian.follow(self.arm, path, start, self.lower, self.upper,
                                       self.check_step)
        except cartesian.PathError as exc:
            raise StepError(str(exc)) from None
        except ValueError as exc:
            raise StepError(str(exc)) from None
        end = samples[-1][1]
        if joints is not None and np.max(np.abs(end - np.asarray(joints))) > 1e-3:
            raise StepError('the path ends in another arm configuration than the given joints')

        profile = path.profile(self.tcp_limits, speed)
        notes = []
        _, _, velocities = cartesian.trajectory(self.arm, path, profile, samples, 0.02)
        ratios = np.max(np.abs(velocities), axis=0) / np.asarray(self.max_velocity)
        if ratios.max() > 1.0:
            factor = float(ratios.max())
            profile = motion.Profile(profile.duration * factor, profile.accel_time * factor,
                                     profile.peak_rate / factor, profile.accel / factor ** 2)
            notes.append(f'slowed down to {100 / factor:.0f}% by the speed limit of '
                         f'j{int(np.argmax(ratios)) + 1}')
        report.duration = profile.duration
        notes += self._singularity_notes(samples)

        check = self._check_motion(
            state, [profile.time_at(s) for s, _ in samples],
            lambda t: cartesian.interpolate(samples, profile.at(t)[0]),
            lambda t: profile.at(t)[0], allowed_contacts, report)
        if check.collision_at >= 0.0:
            report.status = StepReport.FAILED
            report.message = 'collision at {:.0f}% of the path: {}'.format(
                100 * check.collision_at, ', '.join(check.collisions) or 'self collision')
        else:
            self._clearance_status(check, report, f'{path.kind} is free')
            if notes:
                report.status = max(report.status, StepReport.WARNING)
                report.message = '; '.join([report.message] + notes)
        return MovePlan(tuple(start), tuple(float(v) for v in end), profile, check, path,
                        samples, self.arm)

    def _singularity_notes(self, samples):
        """Return a note for every singularity the joints along a path come close to."""
        notes = []
        margins = np.array([self.arm.singularity(q) for _, q in samples])
        for index, (limit, name, joint) in enumerate((
                (WRIST_WARNING, 'wrist', 4), (ELBOW_WARNING, 'elbow', 2),
                (SHOULDER_WARNING, 'shoulder', None))):
            closest = int(np.argmin(margins[:, index]))
            if margins[closest, index] < limit:
                at, q = samples[closest]
                value = (f'j{joint + 1} {math.degrees(q[joint]):.1f} deg' if joint is not None
                         else f'{margins[closest, index] * 1000:.0f} mm')
                notes.append(f'passes close to the {name} singularity at {100 * at:.0f}% '
                             f'({value})')
        return notes

    def plan_pick_goal(self, state, goal, default_speed, report):
        """Check the pick of a PickMoving goal."""
        p, o = goal.grasp.position, goal.grasp.orientation
        quaternion = (o.x, o.y, o.z, o.w) if any((o.x, o.y, o.z, o.w)) else (1.0, 0.0, 0.0, 0.0)
        return self.plan_pick_moving(
            state, goal.conveyor, goal.part, motion.pose_matrix((p.x, p.y, p.z), quaternion),
            goal.approach, goal.lift, goal.speed or default_speed, goal.grip_time, report)

    def plan_pick_moving(self, state, conveyor, part, grasp, approach, lift, speed, grip_time,
                         report):
        """
        Check picking a part on the move from a continuous conveyor; fill the report.

        grasp is the TCP pose relative to the part's grasp frame. The part is the given one, or
        the first one past the sensor that can still be caught, as a tracking system skips
        parts that are already too far (they leave the cell as missed parts). The motion is
        checked in two phases: up to the grip with the part moving on the belt (the tool may
        touch it), then with the part held. The state is not changed.
        """
        model = self.conveyors.models.get(conveyor)
        if model is None or not model.continuous:
            raise StepError(f"'{conveyor}' is not a continuous conveyor")
        if state.held:
            raise StepError(f'already holding {next(iter(state.held))}')
        if not 0.0 < speed <= 1.0:
            raise StepError(f'speed must be in (0, 1], got {speed}')
        if approach <= 0.0 or lift <= 0.0 or grip_time < 0.0:
            raise StepError('approach and lift must be positive, grip_time not negative')
        belt = state.belts[conveyor]
        if part:
            if part not in [name for name, _ in belt.items]:
                raise StepError(f'{part} is not on conveyor {conveyor}')
            candidates = [part]
        else:
            candidates = [name for name, distance in belt.items
                          if distance >= model.sensor - 1e-9]
            if not candidates:
                raise StepError(f'no part has passed the sensor of conveyor {conveyor}: '
                                f'wait for {model.io} first')
        problems = []
        for candidate in candidates:
            try:
                path, timeline, samples = self._catch(state, model, candidate, grasp, approach,
                                                      lift, speed, grip_time)
                part = candidate
                break
            except StepError as exc:
                problems.append(exc)
        else:
            raise problems[0]
        skipped = candidates[:candidates.index(part)]
        report.duration = path.duration
        notes = self._singularity_notes(samples)

        grip_u = path.grip_at / path.duration
        q_grip = cartesian.interpolate(samples, grip_u)
        exact = self.arm.follow(path.pose_at(path.grip_at), q_grip)    # exact at the grip
        if exact is not None:
            q_grip = exact
        first = [u for u, _ in samples if u < grip_u] + [grip_u]
        second = [grip_u] + [u for u, _ in samples if u > grip_u]
        before, after = new_report('', 'move'), new_report('', 'move')
        check1 = self._check_motion(
            state, [timeline.time_at(u) for u in first],
            lambda t: cartesian.interpolate(samples, t / path.duration),
            lambda t: t / path.duration, [part], before)
        held = state.copy()
        self.conveyors.advance(held, path.grip_at)
        offset = motion.inverse(path.pose_at(path.grip_at)) @ held.poses[part]
        self.hold(held, part, offset)
        held.joints = tuple(q_grip)
        self.sync(held)
        self.checker.set_joints(held.joints)
        held.next_allowed = {(part, obj)
                             for obj in self.checker.touching(part, CONTACT_DISTANCE)}
        check2 = new_report('', 'move')
        result2 = PathCheck()
        if check1.collision_at < 0.0:
            result2 = self._check_motion(
                held, [timeline.time_at(u) - path.grip_at for u in second],
                lambda t: cartesian.interpolate(samples, (t + path.grip_at) / path.duration),
                lambda t: (t + path.grip_at) / path.duration, (), after)
            check2 = after
        report.allowed_contacts = sorted(set(before.allowed_contacts)
                                         | set(check2.allowed_contacts))
        failed = before if check1.collision_at >= 0.0 else (
            check2 if result2.collision_at >= 0.0 else None)
        clear = [r for r in (before, check2) if r.min_clearance >= 0.0]
        closest = min(clear, key=lambda r: r.min_clearance) if clear else before
        report.min_clearance = closest.min_clearance
        report.closest_link, report.closest_object = closest.closest_link, closest.closest_object
        check = PathCheck(tcp_path=check1.tcp_path + result2.tcp_path,
                          collision_at=max(check1.collision_at, result2.collision_at))
        if failed is not None:
            report.status = StepReport.FAILED
            report.collisions, report.collision_at = failed.collisions, failed.collision_at
            report.message = 'collision at {:.0f}% of the pick: {}'.format(
                100 * failed.collision_at, ', '.join(failed.collisions))
        else:
            message = (f'picked {part} on the move: in step after {path.sync_time:.2f} s, '
                       f'gripped at {path.grip_at:.2f} s')
            if skipped:
                message += f' ({", ".join(skipped)} too far to catch)'
            check.min_clearance = report.min_clearance
            check.closest_body, check.closest_object = report.closest_link, report.closest_object
            self._clearance_status(check, report, message)
            if notes:
                report.status = max(report.status, StepReport.WARNING)
                report.message = '; '.join([report.message] + notes)
        return PickPlan(part, tuple(state.joints), tuple(float(v) for v in samples[-1][1]),
                        timeline, check, path, samples, self.arm, path.grip_at, offset)

    def _catch(self, state, model, part, grasp, approach, lift, speed, grip_time):
        """Plan the TCP motion and joints that catch part; raise StepError if they cannot."""
        frame = self.definition(part, state.parts).frame('grasp')
        if frame is None:
            raise StepError(f'{part} has no grasp frame')
        try:
            path = tracking.TrackingMotion(
                self.arm.fk(state.joints), state.poses[part], model.velocity,
                motion.transform(frame.xyz, frame.rpy) @ grasp, approach, lift, self.tcp_limits,
                speed, grip_time)
        except ValueError as exc:
            raise StepError(str(exc)) from None
        timeline = tracking.Timeline(path.duration)
        if part not in self.conveyors.predict(state, [path.grip_at])[0][0]:
            raise StepError(f'{part} leaves the conveyor before it can be gripped '
                            f'(at {path.grip_at:.2f} s)')
        try:
            samples = cartesian.follow(self.arm, path, state.joints, self.lower, self.upper,
                                       self.check_step)
        except cartesian.PathError as exc:
            raise StepError(f'{part}: ' + str(exc).replace('of the path', 'of the pick')) \
                from None
        _, _, velocities = cartesian.trajectory(self.arm, path, timeline, samples, 0.02)
        ratios = np.max(np.abs(velocities), axis=0) / np.asarray(self.max_velocity)
        if ratios.max() > 1.0:
            raise StepError(f'j{int(np.argmax(ratios)) + 1} would have to turn at '
                            f'{100 * ratios.max():.0f}% of its speed limit to follow {part}')
        return path, timeline, samples

    def hold(self, state, part, offset):
        """Take a part from where it is into the gripper (offset: its pose relative to the TCP)."""
        state.poses.pop(part, None)
        self.conveyors.take(state, part)
        state.held[part] = np.asarray(offset)

    def complete_pick(self, state, plan):
        """Record a checked pick on the move in a check state (virtual time)."""
        self.advance(state, plan.grip_at)
        self.hold(state, plan.part, plan.offset)
        self.advance(state, plan.path.duration - plan.grip_at)
        self.complete_move(state, plan.goal)

    def plan_move(self, state, target, speed, allowed_contacts, report):
        """
        Check the PTP move from the joints of state to target; fill the report.

        Raise StepError when the move cannot be planned at all; a planned move that collides
        has report.status FAILED. The state is not changed.
        """
        start = np.asarray(state.joints, dtype=float)
        target = np.asarray(target, dtype=float)
        if target.shape != (6,):
            raise StepError(f'joints must have 6 values, got {target.size}')
        problems = motion.outside_limits(target, self.lower, self.upper, JOINTS)
        if problems:
            raise StepError('outside the joint limits: ' + ', '.join(problems))
        if not 0.0 < speed <= 1.0:
            raise StepError(f'speed must be in (0, 1], got {speed}')
        profile = motion.ptp_profile(start, target, self.max_velocity,
                                     [self.joint_acceleration] * 6, speed)
        report.duration = profile.duration
        delta = target - start
        fractions = motion.path_fractions(start, target, self.check_step)
        check = self._check_motion(
            state, [profile.time_at(f) for f in fractions],
            lambda t: start + profile.at(t)[0] * delta, lambda t: profile.at(t)[0],
            allowed_contacts, report)
        if check.collision_at >= 0.0:
            report.status = StepReport.FAILED
            report.message = 'collision at {:.0f}% of the path: {}'.format(
                100 * check.collision_at, ', '.join(check.collisions) or 'self collision')
        else:
            self._clearance_status(check, report, 'path is free')
        return MovePlan(tuple(start), tuple(target), profile, check)

    @staticmethod
    def complete_move(state, joints):
        """Record the end of a move: the robot is at joints, one-move allowances end."""
        state.joints = tuple(float(v) for v in joints)
        state.next_allowed = set()

    def plan_wait(self, state, report, io='', value=True, seconds=0.0, check_only=True):
        """
        Check a wait in state; fill the report and return a WaitPlan. The state is not changed.

        A wait for a time or for a conveyor sensor has a predicted duration. IO that other
        programs set is assumed in a check; in the simulation the wait lasts until it is set
        (duration None). Parts that move meanwhile are checked against the waiting robot.
        """
        target = 'true' if value else 'false'
        if not io:
            if seconds < 0.0:
                raise StepError(f'seconds must not be negative, got {seconds}')
            duration, message = float(seconds), f'waited {seconds:.2f} s'
        elif io in self.sensors:
            limit = seconds if seconds > 0.0 else WAIT_LIMIT
            duration, still = self.conveyors.time_until(state, io, value, limit)
            if duration is None and still:
                raise StepError(f'{io} will not become {target}: {self._why_still(io, value)}')
            if duration is None:
                raise StepError(f'timed out after {limit:.1f} s waiting for {io} = {target}')
            message = f'{io} = {target} after {duration:.2f} s'
        elif state.io.get(io, False) == value:
            duration, message = 0.0, f'{io} is already {target}'
        elif check_only:
            duration, message = 0.0, f'assumed {io} = {target} (set by another program)'
        else:
            duration, message = None, f'{io} = {target} (set by another program)'

        report.duration = duration or 0.0
        horizon = (duration if duration is not None
                   else self.conveyors.time_to_still(state, WAIT_LIMIT))
        if horizon <= 0.0 or not self.conveyors.moving(state):
            report.message = message
            return WaitPlan(duration)
        joints = np.asarray(state.joints, dtype=float)
        check = self._check_motion(state, [0.0, horizon], lambda t: joints,
                                   lambda t: t / horizon, (), report)
        if check.collision_at >= 0.0:
            report.status = StepReport.FAILED
            report.message = 'collision after {:.2f} s of waiting: {}'.format(
                check.collision_at * horizon, ', '.join(check.collisions))
        else:
            self._clearance_status(check, report, message)
        return WaitPlan(duration, check)

    def _why_still(self, io, value):
        model = self.sensors[io]
        if value:
            return f'conveyor {model.name} has no more parts'
        return f'the part at the station of conveyor {model.name} has to be taken first'

    def set_io(self, state, io, value, report):
        """Set a virtual IO in state."""
        if not io:
            raise StepError('the IO name is empty')
        if io in self.sensors:
            raise StepError(f'{io} is the part sensor of conveyor {self.sensors[io].name}; '
                            'it cannot be set')
        state.io[io] = bool(value)
        report.message = f"{io} = {'true' if value else 'false'}"

    def grip(self, state, report):
        """Grip the movable object at the TCP; return its name."""
        if state.held:
            raise StepError(f'already holding {next(iter(state.held))}')
        tcp = self.arm.fk(state.joints)
        grasps = {}
        for name, pose in state.poses.items():
            frame = self.definition(name, state.parts).frame('grasp')
            if frame is not None:
                grasps[name] = (pose @ motion.transform(frame.xyz, frame.rpy))[:3, 3]
        name, distance = motion.nearest_within(tcp[:3, 3], grasps, self.grasp_tolerance)
        if name is None:
            if distance is None:
                raise StepError('nothing to grip: no movable object with a grasp frame')
            nearest = min(grasps, key=lambda n: np.linalg.norm(grasps[n] - tcp[:3, 3]))
            raise StepError(f'nothing to grip: {nearest}/grasp is {distance * 1000:.0f} mm from '
                            f'the TCP (tolerance {self.grasp_tolerance * 1000:.0f} mm)')
        state.held[name] = motion.inverse(tcp) @ state.poses.pop(name)
        self.conveyors.take(state, name)
        self.sync(state)
        self.checker.set_joints(state.joints)
        support = self.checker.touching(name, CONTACT_DISTANCE)
        state.next_allowed = {(name, obj) for obj in support}
        report.held_object = name
        report.message = f'gripped {name}, TCP {distance * 1000:.1f} mm from {name}/grasp'
        if support:
            report.message += f', lifting it from {", ".join(support)}'
        return name

    def release(self, state, report):
        """Release the held object where it is; return (name, 4x4 world pose) or None."""
        if not state.held:
            report.status, report.message = StepReport.WARNING, 'nothing to release'
            return None
        name, offset = next(iter(state.held.items()))
        pose = self.arm.fk(state.joints) @ offset
        del state.held[name]
        state.poses[name] = pose
        state.next_allowed = {(body, name) for body in self.tool_bodies}
        report.message = 'released {} at ({:.3f}, {:.3f}, {:.3f}) m'.format(name, *pose[:3, 3])
        return name, pose

    def object_poses(self, state, joints):
        """Return the world poses of the movable objects, held ones at the TCP for joints."""
        poses, held = dict(state.poses), dict(state.held)
        if held:
            tcp = self.arm.fk(joints)
            for name, offset in held.items():
                poses[name] = tcp @ offset
        return poses, set(held)

    # ---------------------------------------------------------------- checks
    def _check_motion(self, state, times, joints_at, at, allowed_contacts, report):
        """Check the robot at joints_at(t) for the times while the conveyors run."""
        duration = times[-1]
        frames, parts = None, state.parts
        if duration > 0.0 and self.conveyors.moving(state):
            step = PART_STEP / max(model.speed for model in self.conveyors.models.values())
            grid = np.arange(0.0, duration, step).tolist()
            times = sorted({float(t) for t in times} | set(grid) | {float(duration)})
            frames, parts = self.conveyors.predict(state, times)
        allowed = self._allowed(state, allowed_contacts, parts)
        report.allowed_contacts = sorted(f'{body} <-> {obj}' for body, obj in allowed)

        poses = self.all_poses(state)
        moving = set().union(*frames) if frames else set()
        for name in moving:
            poses.setdefault(name, FAR)
        definitions = {name: self.definition(name, parts)
                       for name in list(poses) + list(state.held) if name in parts}
        self.checker.sync(poses, state.held, HELD_TOUCH_LINKS, definitions)
        previous = {name: poses[name] for name in moving}
        samples = []
        for index, t in enumerate(times):
            moved = {}
            for name in moving:
                pose = frames[index].get(name, FAR)
                if not np.array_equal(previous[name], pose):
                    moved[name] = previous[name] = pose
            samples.append((at(t), joints_at(t), moved))
        check = self.checker.check_motion(samples, allowed)
        if check.collision_at < 0.0 and 0.0 <= check.min_clearance < CONTACT_DISTANCE:
            # touching, as when a part is set down exactly on a surface: a contact to allow
            check.collision_at = check.clearance_at
            check.collisions = [f'{check.closest_body} <-> {check.closest_object}']
        report.min_clearance = check.min_clearance
        report.closest_link, report.closest_object = check.closest_body, check.closest_object
        report.collisions, report.collision_at = check.collisions, check.collision_at
        return check

    def _allowed(self, state, allowed_contacts, parts):
        allowed = set(state.next_allowed)
        bodies = self.tool_bodies + list(state.held)
        for name in allowed_contacts:
            if name in self.objects or name in parts:
                targets = [name]
            elif name in self.part_types:
                targets = [p for p, kind in parts.items() if kind == name and p not in state.held]
            else:
                raise StepError(f"allowed_contacts: unknown object '{name}'")
            allowed |= {(body, target) for body in bodies for target in targets}
        return allowed

    def _clearance_status(self, check, report, message):
        if 0.0 <= check.min_clearance < self.clearance_warning:
            report.status = StepReport.WARNING
            report.message = (f'{check.min_clearance * 1000:.1f} mm from {check.closest_object} '
                              f'({check.closest_body})')
        else:
            report.message = message
