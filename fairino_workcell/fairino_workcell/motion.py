"""
PTP trajectories and pose math for the work cell executor (numpy only).

A PTP move follows a straight line in joint space, like MoveJ on a teach pendant. All joints
start and stop together with a trapezoidal (or, for short moves, triangular) velocity profile
that keeps every joint within its velocity and acceleration limits.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Profile:
    """Normalised motion s(t) from 0 to 1 shared by all joints of a PTP move."""

    duration: float
    accel_time: float
    peak_rate: float
    accel: float

    def at(self, t):
        """Return (s, ds/dt, d2s/dt2) at time t."""
        if self.duration <= 0.0 or t >= self.duration:
            return 1.0, 0.0, 0.0
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t < self.accel_time:
            return 0.5 * self.accel * t * t, self.accel * t, self.accel
        if t <= self.duration - self.accel_time:
            s = 0.5 * self.accel * self.accel_time ** 2 + self.peak_rate * (t - self.accel_time)
            return s, self.peak_rate, 0.0
        remaining = self.duration - t
        return 1.0 - 0.5 * self.accel * remaining ** 2, self.accel * remaining, -self.accel


def ptp_profile(start, goal, max_velocity, max_acceleration, speed=1.0):
    """Return the shortest profile that keeps every joint within speed x its limits."""
    if not 0.0 < speed <= 1.0:
        raise ValueError(f'speed must be in (0, 1], got {speed}')
    delta = np.abs(np.asarray(goal, dtype=float) - np.asarray(start, dtype=float))
    moving = delta > 1e-9
    if not moving.any():
        return Profile(0.0, 0.0, 0.0, 0.0)
    rate = float(np.min(np.asarray(max_velocity, dtype=float)[moving] * speed / delta[moving]))
    accel = float(np.min(np.asarray(max_acceleration, dtype=float)[moving] * speed
                         / delta[moving]))
    if rate * rate / accel >= 1.0:  # too short to reach the peak rate: triangular profile
        accel_time = math.sqrt(1.0 / accel)
        return Profile(2.0 * accel_time, accel_time, accel * accel_time, accel)
    accel_time = rate / accel
    return Profile(1.0 / rate + accel_time, accel_time, rate, accel)


def sample_ptp(start, goal, profile, dt):
    """Return times, positions, velocities and accelerations every dt, ending at the goal."""
    start = np.asarray(start, dtype=float)
    delta = np.asarray(goal, dtype=float) - start
    count = max(1, math.ceil(profile.duration / dt)) if profile.duration > 0 else 1
    times = np.linspace(0.0, profile.duration, count + 1)
    rates = np.array([profile.at(t) for t in times])
    positions = start + np.outer(rates[:, 0], delta)
    positions[-1] = start + delta
    return times, positions, np.outer(rates[:, 1], delta), np.outer(rates[:, 2], delta)


def path_fractions(start, goal, max_step):
    """Return fractions of the joint-space line so that no joint moves more than max_step."""
    span = float(np.max(np.abs(np.asarray(goal, dtype=float) - np.asarray(start, dtype=float))))
    return np.linspace(0.0, 1.0, max(1, math.ceil(span / max_step)) + 1)


def outside_limits(joints, lower, upper, names):
    """Return a description of every joint outside [lower, upper]."""
    return [f'{name} {math.degrees(q):.1f} deg (limits {math.degrees(lo):.1f} to '
            f'{math.degrees(hi):.1f})'
            for name, q, lo, hi in zip(names, joints, lower, upper)
            if q < lo - 1e-6 or q > hi + 1e-6]


def rpy_matrix(roll, pitch, yaw):
    """Return the rotation matrix of fixed-axis roll, pitch, yaw angles."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quaternion_matrix(x, y, z, w):
    """Return the rotation matrix of a quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quaternion(rotation):
    """Return the (x, y, z, w) quaternion of a rotation matrix."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    return tuple(float(v) for v in q)


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    """Return the 4x4 homogeneous transform of a position and roll, pitch, yaw."""
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


def pose_matrix(position, orientation):
    """Return the 4x4 transform of a position and an (x, y, z, w) quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(*orientation)
    matrix[:3, 3] = position
    return matrix


def inverse(matrix):
    """Return the inverse of a rigid 4x4 transform."""
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return result


def nearest_within(position, candidates, tolerance):
    """
    Return (name, distance) of the candidate position nearest to position, if within tolerance.

    The name is None when nothing is within tolerance; the distance is then that of the
    nearest candidate (or None when there are no candidates).
    """
    best, best_distance = None, None
    for name, candidate in candidates.items():
        distance = float(np.linalg.norm(np.asarray(candidate) - np.asarray(position)))
        if best_distance is None or distance < best_distance:
            best, best_distance = name, distance
    if best_distance is None or best_distance > tolerance:
        return None, best_distance
    return best, best_distance
