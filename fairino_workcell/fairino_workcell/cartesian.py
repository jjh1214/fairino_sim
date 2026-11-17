"""
Cartesian TCP moves: straight lines (LIN, like MoveL) and arcs through a point (CIRC, like MoveC).

The TCP position follows the line or the arc while its orientation turns uniformly (slerp) from
the start to the end orientation, with a trapezoidal speed profile along the path. The joints
follow from the inverse kinematics point by point, continuously from the start joints: where
the arm would have to change its configuration (a singularity) or leave its joint limits, the
move fails.
"""

import math

from fairino_workcell import motion
import numpy as np


WRIST_NEAR = math.sin(math.radians(10.0))    # |sin j5| below this: near the wrist singularity
ELBOW_NEAR = math.sin(math.radians(10.0))    # |sin j3|
SHOULDER_NEAR = 0.05                         # m from the shoulder singularity


def singularity_note(arm, joints):
    """Return ', near the ... singularity (...)' when the joints are close to one, else ''."""
    wrist, elbow, shoulder = arm.singularity(joints)
    notes = []
    if wrist < WRIST_NEAR:
        notes.append(f'the wrist singularity (j5 {math.degrees(joints[4]):.1f} deg)')
    if elbow < ELBOW_NEAR:
        notes.append(f'the elbow singularity (j3 {math.degrees(joints[2]):.1f} deg)')
    if shoulder < SHOULDER_NEAR:
        notes.append(f'the shoulder singularity ({shoulder * 1000:.0f} mm)')
    return ', near ' + ' and '.join(notes) if notes else ''


def _failure_note(arm, samples, joints):
    """Explain a failure by a singularity at the failing joints, or passed on the way."""
    note = singularity_note(arm, joints)
    if note:
        return note
    margins = [(arm.singularity(q), s, q) for s, q in samples]
    for index, (limit, name) in enumerate(((WRIST_NEAR, 'wrist'), (ELBOW_NEAR, 'elbow'),
                                           (SHOULDER_NEAR, 'shoulder'))):
        margin, s, q = min(margins, key=lambda item: item[0][index])
        if margin[index] < limit:
            if index == 2:
                value = f'{margin[index] * 1000:.0f} mm'
            else:
                joint = 4 if index == 0 else 2
                value = f'j{joint + 1} {math.degrees(q[joint]):.1f} deg'
            return f', after passing close to the {name} singularity at {100 * s:.0f}% ({value})'
    return ''


class PathError(ValueError):
    """The TCP path cannot be followed; at is the path fraction (0..1) where it fails."""

    def __init__(self, message, at):
        super().__init__(message)
        self.at = at


def _quaternion(rotation):
    return np.array(motion.matrix_quaternion(rotation))


def slerp(q0, q1, s):
    """Interpolate two unit quaternions (x, y, z, w) along the shorter arc."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1.0 - s) * theta) * q0 + math.sin(s * theta) * q1) / math.sin(theta)


class CartesianPath:
    """TCP poses along a line, or an arc through via, from start to end (4x4 world poses)."""

    def __init__(self, start, end, via=None):
        self.start, self.end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        self.q0, self.q1 = _quaternion(self.start[:3, :3]), _quaternion(self.end[:3, :3])
        self.angle = 2.0 * math.acos(min(1.0, abs(float(np.dot(self.q0, self.q1)))))
        p0, p1 = self.start[:3, 3], self.end[:3, 3]
        if via is None:
            self.kind = 'line'
            self.length = float(np.linalg.norm(p1 - p0))
            return
        self.kind = 'arc'
        via = np.asarray(via, dtype=float)
        ab, ac = via - p0, p1 - p0
        normal = np.cross(ab, ac)
        if np.linalg.norm(normal) < 1e-6 * max(1e-9, np.linalg.norm(ab) * np.linalg.norm(ac)):
            raise ValueError('the via point is on the line from start to end: no arc')
        twice_area = 2.0 * np.dot(normal, normal)
        self.center = p0 + (np.dot(ac, ac) * np.cross(normal, ab)
                            + np.dot(ab, ab) * np.cross(ac, normal)) / twice_area
        self.radius = float(np.linalg.norm(p0 - self.center))
        self.u = (p0 - self.center) / self.radius
        self.v = np.cross(normal / np.linalg.norm(normal), self.u)
        offset = p1 - self.center
        self.sweep = math.atan2(np.dot(offset, self.v), np.dot(offset, self.u)) % (2.0 * math.pi)
        self.length = self.radius * self.sweep

    def position(self, s):
        if self.kind == 'line':
            return self.start[:3, 3] + s * (self.end[:3, 3] - self.start[:3, 3])
        angle = self.sweep * s
        return self.center + self.radius * (math.cos(angle) * self.u + math.sin(angle) * self.v)

    def pose(self, s):
        """Return the 4x4 TCP pose at the path fraction s."""
        matrix = np.eye(4)
        matrix[:3, :3] = motion.quaternion_matrix(*slerp(self.q0, self.q1, s))
        matrix[:3, 3] = self.position(s)
        return matrix

    def profile(self, limits, speed):
        """Return the speed profile of the path; limits are (speed, accel, turn, turn accel)."""
        speed_limit, accel_limit, turn_limit, turn_accel_limit = limits
        return motion.ptp_profile([0.0, 0.0], [self.length, self.angle],
                                  [speed_limit, turn_limit], [accel_limit, turn_accel_limit],
                                  speed)


def follow(arm, path, start_joints, lower, upper, max_step, linear_step=0.002,
           angle_step=math.radians(0.5)):
    """
    Return the joints along the path as [(s, joints)], from start_joints.

    Samples are at most linear_step and angle_step apart on the path and max_step apart in
    every joint. Raise PathError where the joints would have to jump (singularity), where the
    pose cannot be reached, or where a joint leaves its limits.
    """
    steps = max(1, math.ceil(max(path.length / linear_step, path.angle / angle_step)))
    samples = [(0.0, np.asarray(start_joints, dtype=float))]
    for s in np.linspace(0.0, 1.0, steps + 1)[1:]:
        _extend(arm, path, samples, float(s), lower, upper, max_step)
    return samples


def _extend(arm, path, samples, s, lower, upper, max_step):
    """Append the joints at s, inserting samples where the joints move too much."""
    pending = [s]
    while pending:
        s = pending[-1]
        s_prev, q_prev = samples[-1]
        q = arm.follow(path.pose(s), q_prev)
        if q is None:
            raise PathError(f'the TCP cannot follow the path at {100 * s:.0f}%: out of reach'
                            + _failure_note(arm, samples, q_prev), s)
        if np.max(np.abs(q - q_prev)) > max_step:
            if s - s_prev < 1e-7:
                raise PathError(f'the path passes a singularity at {100 * s:.0f}%: the arm '
                                'would have to change its configuration'
                                + _failure_note(arm, samples, q_prev), s)
            pending.append(0.5 * (s_prev + s))
            continue
        for index, value in enumerate(q):
            if not lower[index] - 1e-9 <= value <= upper[index] + 1e-9:
                raise PathError(f'j{index + 1} leaves its limits at {100 * s:.0f}% of the path '
                                f'({math.degrees(value):.1f} deg)'
                                + _failure_note(arm, samples, q), s)
        samples.append((s, q))
        pending.pop()


def interpolate(samples, s):
    """Return the joints at the path fraction s between the samples of follow()."""
    fractions = np.array([f for f, _ in samples])
    joints = np.array([q for _, q in samples])
    return np.array([np.interp(s, fractions, joints[:, i]) for i in range(joints.shape[1])])


def trajectory(arm, path, profile, samples, dt):
    """Return times, joint positions and velocities every dt along the path."""
    # evenly spaced, at most dt apart (np.arange would add a point a hair before the end when
    # the duration is a whole number of steps, which the controller rejects)
    count = max(1, math.ceil(profile.duration / dt)) if profile.duration > 0.0 else 0
    times = np.linspace(0.0, profile.duration, count + 1).tolist()
    positions, q = [], samples[0][1]
    for t in times:
        s = profile.at(t)[0]
        guess = interpolate(samples, s)
        q = arm.follow(path.pose(s), guess)
        positions.append(guess if q is None else q)
    positions = np.array(positions)
    if len(times) > 1:
        velocities = np.gradient(positions, np.array(times), axis=0)
    else:
        velocities = np.zeros_like(positions)
    velocities[0] = velocities[-1] = 0.0
    return np.array(times), positions, velocities
