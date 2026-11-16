"""Tests of the PTP profile, sampling and pose math."""

import math

from fairino_workcell import motion
import numpy as np
import pytest

VMAX = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2]
AMAX = [3.0] * 6


def test_zero_move():
    profile = motion.ptp_profile([0.1] * 6, [0.1] * 6, VMAX, AMAX)
    assert profile.duration == 0.0
    times, positions, _, _ = motion.sample_ptp([0.1] * 6, [0.1] * 6, profile, 0.02)
    assert len(times) == 2 and positions[-1] == pytest.approx([0.1] * 6)


@pytest.mark.parametrize('goal, speed', [
    ([1.5, -0.4, 0.2, 0.0, 0.3, -2.0], 1.0),     # long move: trapezoid
    ([1.5, -0.4, 0.2, 0.0, 0.3, -2.0], 0.3),
    ([0.05, 0.0, -0.02, 0.0, 0.0, 0.0], 1.0),    # short move: triangle
])
def test_ptp_stays_within_limits_and_is_synchronised(goal, speed):
    start = np.zeros(6)
    profile = motion.ptp_profile(start, goal, VMAX, AMAX, speed)
    times, positions, velocities, accelerations = motion.sample_ptp(
        start, goal, profile, 0.001)
    assert positions[0] == pytest.approx(start) and positions[-1] == pytest.approx(goal)
    assert times[-1] == pytest.approx(profile.duration)
    assert np.all(np.abs(velocities) <= np.array(VMAX) * speed + 1e-6)
    assert np.all(np.abs(accelerations) <= np.array(AMAX) * speed + 1e-6)
    # straight line in joint space: every sample is start + s * (goal - start)
    direction = np.asarray(goal) / np.linalg.norm(goal)
    offsets = positions - np.outer(positions @ direction, direction)
    assert np.abs(offsets).max() < 1e-9
    # the limiting joint actually reaches its velocity or acceleration limit
    leading = int(np.argmax(np.abs(goal)))
    assert (np.abs(velocities[:, leading]).max() >= VMAX[leading] * speed - 1e-3
            or np.abs(accelerations[:, leading]).max() >= AMAX[leading] * speed - 1e-3)


def test_trapezoid_duration_matches_closed_form():
    # one joint, 1.5 rad at 1 rad/s and 2 rad/s^2: 0.5 s ramps, total 1.5/1 + 0.5 = 2.0 s
    profile = motion.ptp_profile([0.0], [1.5], [1.0], [2.0])
    assert profile.duration == pytest.approx(2.0)
    assert profile.accel_time == pytest.approx(0.5)


def test_invalid_speed():
    with pytest.raises(ValueError):
        motion.ptp_profile([0.0], [1.0], [1.0], [1.0], speed=0.0)


def test_path_fractions_limit_the_joint_step():
    fractions = motion.path_fractions([0.0, 0.0], [math.radians(10), 0.1], math.radians(0.5))
    steps = np.diff(fractions) * math.radians(10)
    assert fractions[0] == 0.0 and fractions[-1] == 1.0
    assert steps.max() <= math.radians(0.5) + 1e-12


def test_outside_limits():
    problems = motion.outside_limits([0.0, 3.2], [-3.0, -3.0], [3.0, 3.0], ['j1', 'j2'])
    assert len(problems) == 1 and problems[0].startswith('j2 183.3 deg')


def test_pose_math_round_trips():
    rpy = (0.3, -0.7, 2.1)
    rotation = motion.rpy_matrix(*rpy)
    assert motion.quaternion_matrix(*motion.matrix_quaternion(rotation)) == pytest.approx(rotation)
    for angles in [(math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi), (math.pi, 0.1, -0.2)]:
        r = motion.rpy_matrix(*angles)
        assert motion.quaternion_matrix(*motion.matrix_quaternion(r)) == pytest.approx(r)
    a = motion.transform((1, 2, 3), (0.1, 0.2, 0.3))
    assert motion.inverse(a) @ a == pytest.approx(np.eye(4))
    b = motion.pose_matrix((0, 0, 1), motion.matrix_quaternion(motion.rpy_matrix(math.pi, 0, 0)))
    assert (b @ np.array([0, 0, 1, 1]))[:3] == pytest.approx([0, 0, 0])


def test_nearest_within():
    candidates = {'a': (0.0, 0.0, 0.05), 'b': (0.0, 0.0, 0.004)}
    assert motion.nearest_within((0, 0, 0), candidates, 0.01) == ('b', pytest.approx(0.004))
    name, distance = motion.nearest_within((0, 0, 0), {'a': (0, 0, 0.05)}, 0.01)
    assert name is None and distance == pytest.approx(0.05)
    assert motion.nearest_within((0, 0, 0), {}, 0.01) == (None, None)
