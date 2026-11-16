"""
A robot in its work cell, without ROS: kinematics, path checks and the rules of recipe steps.

RobotCell checks steps against a CellState (joints, object poses, held object). The
workcell_executor node wraps it for ROS and executes the steps in the simulation; the tests use
it directly.

Rules of the steps:
- move: the target joints (given, or solved from a TCP pose nearest to the current joints) must
  be within the joint limits, and the PTP path must be free of collisions of the robot, the
  tool and the held object with the cell and with themselves. Contacts can be allowed per move
  with allowed_contacts (tool and held object against the named objects).
- grip: takes the movable object whose 'grasp' frame is nearest to the TCP, within
  grasp_tolerance. Lifting it off its support is allowed in the next move.
- release: leaves the held object where it is. Moving the tool away from it is allowed in the
  next move.
"""

from dataclasses import dataclass, field
import math
import xml.etree.ElementTree as ET

from fairino_sim import description
from fairino_workcell import motion
from fairino_workcell.checker import Checker, PathCheck
from fairino_workcell.kinematics import Arm
from fairino_workcell_msgs.msg import StepReport
import numpy as np

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
HELD_TOUCH_LINKS = ['tcp', 'tool_link', 'flange', 'wrist3_link']
DEFAULT_SPEED = 0.3       # fraction of the joint velocity and acceleration limits


class StepError(RuntimeError):
    """A step cannot be done; the message goes into the report."""


@dataclass
class CellState:
    """Robot and object state of the simulation, or of a check session."""

    joints: tuple
    poses: dict                                      # object -> 4x4 world pose (not held)
    held: dict = field(default_factory=dict)         # object -> 4x4 pose relative to the TCP
    next_allowed: set = field(default_factory=set)   # contacts allowed in the next move only

    def copy(self):
        return CellState(tuple(self.joints), {k: v.copy() for k, v in self.poses.items()},
                         {k: v.copy() for k, v in self.held.items()}, set(self.next_allowed))


@dataclass
class MovePlan:
    """A checked PTP move."""

    start: tuple
    goal: tuple
    profile: motion.Profile
    check: PathCheck


def new_report(name, kind, check_only=False):
    return StepReport(name=name, kind=kind, status=StepReport.OK, check_only=check_only,
                      min_clearance=-1.0, collision_at=-1.0)


class RobotCell:
    """The robot of a cell file with its tool, and the objects around it."""

    def __init__(self, cell, joint_acceleration=3.0, clearance_warning=0.01,
                 grasp_tolerance=0.01, check_step=math.radians(0.5)):
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

        self.cell = cell
        self.group = f'{robot.model}_group'
        self.tool_bodies = ['tool_link'] if tool is not None else ['wrist3_link']
        self.arm = Arm(self.urdf, JOINTS)    # world -> tcp
        self.checker = Checker(cell, self.urdf, description.build_srdf(robot.model, tool),
                               self.group, self.tool_bodies)
        self.objects = {obj.name: obj for obj in cell.objects}
        poses = {obj.name: motion.transform(obj.pose.xyz, obj.pose.rpy) for obj in cell.objects}
        self.static_poses = {n: p for n, p in poses.items() if not self.objects[n].movable}
        self.movable_poses = {n: p for n, p in poses.items() if self.objects[n].movable}

    def initial_state(self, joints=None):
        """Return the state of the cell file: initial joints, objects where the file puts them."""
        joints = self.cell.robot.initial_positions if joints is None else joints
        return CellState(tuple(float(v) for v in joints),
                         {n: p.copy() for n, p in self.movable_poses.items()})

    # ---------------------------------------------------------------- frames and targets
    def all_poses(self, state):
        poses = dict(self.static_poses)
        poses.update(state.poses)
        return poses

    def sync(self, state):
        self.checker.sync(self.all_poses(state), state.held, HELD_TOUCH_LINKS)

    def frame_matrix(self, frame, state):
        """Return the 4x4 world pose of 'world', an object or 'object/frame'."""
        if frame in ('', 'world'):
            return np.eye(4)
        name, _, sub = frame.partition('/')
        if name not in self.objects:
            raise StepError(f"unknown frame '{frame}'")
        if name in state.held:
            raise StepError(f"frame '{frame}' moves with the held object")
        matrix = self.all_poses(state)[name]
        if not sub:
            return matrix
        pose = self.objects[name].frame(sub)
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
    def plan_move(self, state, target, speed, allowed_contacts, report):
        """
        Check the PTP move from the joints of state to target; fill the report.

        Raise StepError when the move cannot be planned at all; a planned move that collides
        has report.status FAILED.
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
        allowed = set(state.next_allowed)
        for obj in allowed_contacts:
            if obj not in self.objects:
                raise StepError(f"allowed_contacts: unknown object '{obj}'")
            allowed |= {(body, obj) for body in self.tool_bodies + list(state.held)}
        report.allowed_contacts = sorted(f'{b} <-> {o}' for b, o in allowed)

        profile = motion.ptp_profile(start, target, self.max_velocity,
                                     [self.joint_acceleration] * 6, speed)
        report.duration = profile.duration
        self.sync(state)
        check = self.checker.check_path(start, target, allowed, self.check_step)
        report.min_clearance = check.min_clearance
        report.closest_link, report.closest_object = check.closest_body, check.closest_object
        report.collisions, report.collision_at = check.collisions, check.collision_at
        if check.collision_at >= 0.0:
            report.status = StepReport.FAILED
            report.message = 'collision at {:.0f}% of the path: {}'.format(
                100 * check.collision_at, ', '.join(check.collisions) or 'self collision')
        elif 0.0 <= check.min_clearance < self.clearance_warning:
            report.status = StepReport.WARNING
            report.message = (f'{check.min_clearance * 1000:.1f} mm from {check.closest_object} '
                              f'({check.closest_body})')
        else:
            report.message = 'path is free'
        return MovePlan(tuple(start), tuple(target), profile, check)

    @staticmethod
    def complete_move(state, joints):
        """Record the end of a move: the robot is at joints, one-move allowances end."""
        state.joints = tuple(float(v) for v in joints)
        state.next_allowed = set()

    def grip(self, state, report):
        """Grip the movable object at the TCP; return its name."""
        if state.held:
            raise StepError(f'already holding {next(iter(state.held))}')
        tcp = self.arm.fk(state.joints)
        grasps = {}
        for name, pose in state.poses.items():
            frame = self.objects[name].frame('grasp')
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
        self.sync(state)
        self.checker.set_joints(state.joints)
        support = self.checker.touching(name)
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
