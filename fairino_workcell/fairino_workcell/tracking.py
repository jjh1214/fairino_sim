"""
Picking a part on the move from a continuous conveyor (conveyor tracking).

The pick is one motion of the TCP, planned as a whole:

1. sync: from rest to a point above the moving part, arriving there at the belt's velocity;
2. down: a straight line to the grasp pose, relative to the part (it keeps moving);
3. grip: following the part while the gripper closes;
4. lift: a straight line up, relative to the belt, with the part in the gripper;
5. stop: slowing down from the belt's velocity to rest.

The part moves in a straight line at constant velocity, so where it will be is known at every
moment and the TCP pose is a function of time. TrackingMotion offers it like a CartesianPath
(pose(u) for u = t / duration), so the joints follow from cartesian.follow.
"""

import math

from fairino_workcell import motion
from fairino_workcell.cartesian import slerp
import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def smooth(u):
    """Rise from 0 to 1 with zero velocity and acceleration at both ends (quintic)."""
    u = min(max(u, 0.0), 1.0)
    return u ** 3 * (10.0 - 15.0 * u + 6.0 * u * u)


def _end_velocity(u):
    """Quintic that has velocity 1 and position, acceleration 0 at u = 1 (0 at u = 0)."""
    u = min(max(u, 0.0), 1.0)
    return u ** 3 * (-4.0 + 7.0 * u - 3.0 * u * u)


def _stop_distance(u):
    """Distance covered while the speed falls smoothly from 1 to 0 (0.5 at u = 1)."""
    u = min(max(u, 0.0), 1.0)
    return u - 2.5 * u ** 4 + 3.0 * u ** 5 - u ** 6


class TrackingMotion:
    """TCP poses over time for picking a part on the move."""

    def __init__(self, start, part_at_start, velocity, grasp, approach, lift, limits, speed,
                 grip_time):
        """
        Plan the motion.

        start: 4x4 TCP pose at rest at time 0; part_at_start: 4x4 world pose of the part at
        time 0, moving with velocity (world 3-vector); grasp: TCP pose relative to the part
        when gripping; approach and lift: heights in metres above the grasp pose; limits:
        TCP (speed, acceleration, turn speed, turn acceleration); speed: fraction of them.
        """
        self.start = np.asarray(start, dtype=float)
        self.part0 = np.asarray(part_at_start, dtype=float)
        self.velocity = np.asarray(velocity, dtype=float)
        self.grasp_world0 = self.part0 @ grasp            # grasp pose at time 0
        self.approach, self.lift_height = approach, lift
        speed_limit, accel_limit, turn_limit, turn_accel_limit = (v * speed for v in limits)
        self.q0 = np.array(motion.matrix_quaternion(self.start[:3, :3]))
        self.q1 = np.array(motion.matrix_quaternion(self.grasp_world0[:3, :3]))
        self.angle = 2.0 * math.acos(min(1.0, abs(float(np.dot(self.q0, self.q1)))))

        self.sync_time = self._sync_time(speed_limit, accel_limit, turn_limit, turn_accel_limit)
        self.down_profile = motion.ptp_profile([0.0], [approach], [speed_limit], [accel_limit])
        self.lift_profile = motion.ptp_profile([0.0], [lift], [speed_limit], [accel_limit])
        belt_speed = float(np.linalg.norm(self.velocity))
        self.stop_time = max(0.2, 1.875 * belt_speed / accel_limit)
        self.grip_at = self.sync_time + self.down_profile.duration + grip_time
        self.lift_end = self.grip_at + self.lift_profile.duration
        self.duration = self.lift_end + self.stop_time
        self.length = (np.linalg.norm(self._approach_point(self.sync_time) - self.start[:3, 3])
                       + approach + lift + belt_speed * (self.duration - self.sync_time))

    def _approach_point(self, t):
        return self.grasp_world0[:3, 3] + self.velocity * t + self.approach * UP

    def _sync_position(self, t, total):
        u = t / total
        return (self.start[:3, 3] + (self._approach_point(total) - self.start[:3, 3]) * smooth(u)
                + self.velocity * total * _end_velocity(u))

    def _sync_time(self, speed_limit, accel_limit, turn_limit, turn_accel_limit):
        """Return the shortest sync time within the limits (bisection on sampled motion)."""
        u = np.linspace(0.0, 1.0, 201)

        def within(total):
            positions = np.array([self._sync_position(v * total, total) for v in u])
            dt = total / (len(u) - 1)
            speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1) / dt
            accels = np.linalg.norm(np.diff(positions, 2, axis=0), axis=1) / dt ** 2
            turn = self.angle * 1.875 / total              # peak rate of smooth()
            turn_accel = self.angle * 5.774 / total ** 2   # peak acceleration of smooth()
            return (speeds.max() <= speed_limit and accels.max() <= accel_limit
                    and turn <= turn_limit and turn_accel <= turn_accel_limit)

        low, high = 0.05, 1.0
        while not within(high):
            low, high = high, high * 2.0
            if high > 120.0:
                raise ValueError('the part cannot be caught within the TCP speed limits')
        for _ in range(30):
            middle = 0.5 * (low + high)
            low, high = (low, middle) if within(middle) else (middle, high)
        return high

    def position(self, t):
        if t <= self.sync_time:
            return self._sync_position(t, self.sync_time)
        grasp_now = self.grasp_world0[:3, 3] + self.velocity * min(t, self.grip_at)
        if t <= self.grip_at:
            down = self.down_profile.at(t - self.sync_time)[0]
            return grasp_now + self.approach * (1.0 - down) * UP
        held = grasp_now + self.velocity * (min(t, self.lift_end) - self.grip_at)
        up = self.lift_profile.at(t - self.grip_at)[0]
        if t <= self.lift_end:
            return held + self.lift_height * up * UP
        stopping = _stop_distance((t - self.lift_end) / self.stop_time)
        return held + self.lift_height * UP + self.velocity * self.stop_time * stopping

    def pose_at(self, t):
        """Return the 4x4 TCP pose at time t."""
        matrix = np.eye(4)
        turn = smooth(t / self.sync_time) if self.sync_time > 0 else 1.0
        matrix[:3, :3] = motion.quaternion_matrix(*slerp(self.q0, self.q1, turn))
        matrix[:3, 3] = self.position(min(max(t, 0.0), self.duration))
        return matrix

    def pose(self, u):
        """Return the TCP pose at the fraction u of the duration (as CartesianPath.pose)."""
        return self.pose_at(u * self.duration)


class Timeline:
    """Profile of a TrackingMotion: the fraction u runs linearly with time."""

    def __init__(self, duration):
        self.duration = duration

    def at(self, t):
        return min(max(t / self.duration, 0.0), 1.0), 1.0 / self.duration, 0.0

    def time_at(self, u):
        return u * self.duration
