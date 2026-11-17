import math

from fairino_workcell import cartesian
from fairino_workcell.kinematics import Arm, pose_error
from fairino_workcell.motion import transform
import numpy as np
import pytest
from test_kinematics import _limits, _urdf, JOINTS

HOME = np.radians([-45, -90, -90, -90, 90, 0])
STEP = math.radians(0.5)


@pytest.fixture(scope='module')
def arm():
    urdf = _urdf('fairino5_v6')
    return Arm(urdf, JOINTS), _limits(urdf)


def test_a_line_keeps_the_tcp_on_the_line(arm):
    arm, (lower, upper) = arm
    start = arm.fk(HOME)
    end = start @ transform((0.1, 0.05, 0.2), (0.0, 0.0, math.radians(40)))
    path = cartesian.CartesianPath(start, end)
    samples = cartesian.follow(arm, path, HOME, lower, upper, STEP)
    direction = (end[:3, 3] - start[:3, 3]) / path.length
    for s, q in samples:
        tcp = arm.fk(q)
        offset = tcp[:3, 3] - start[:3, 3]
        assert np.linalg.norm(np.cross(offset, direction)) < 1e-6          # on the line
        assert pose_error(path.pose(s), tcp)[1] < 1e-6                     # slerped orientation
    assert pose_error(end, arm.fk(samples[-1][1])) < (1e-6, 1e-6)
    steps = np.abs(np.diff([q for _, q in samples], axis=0))
    assert steps.max() <= STEP + 1e-9
    assert path.angle == pytest.approx(math.radians(40), abs=1e-9)


def test_an_arc_passes_its_via_point():
    start, end = transform((0.4, 0.0, 0.3)), transform((0.4, 0.2, 0.3))
    via = np.array([0.5, 0.1, 0.3])
    path = cartesian.CartesianPath(start, end, via)
    assert path.radius == pytest.approx(0.1) and path.sweep == pytest.approx(math.pi)
    assert np.linalg.norm(path.center - via) == pytest.approx(0.1)
    assert min(np.linalg.norm(path.position(s) - via) for s in np.linspace(0, 1, 1001)) < 1e-3
    assert np.allclose(path.position(1.0), end[:3, 3])
    other = cartesian.CartesianPath(start, end, np.array([0.3, 0.1, 0.3]))   # the other way
    assert other.sweep == pytest.approx(math.pi) and other.position(0.5)[0] < 0.35
    corner = np.array([0.4 + 0.1 * (1 - math.sqrt(0.5)), 0.1 - 0.1 * math.sqrt(0.5), 0.3])
    short = cartesian.CartesianPath(start, end, corner)
    assert short.length < path.length
    with pytest.raises(ValueError, match='on the line'):
        cartesian.CartesianPath(start, end, np.array([0.4, 0.1, 0.3]))


def test_slerp_takes_the_short_way():
    q0, q1 = np.array([0, 0, 0, 1.0]), np.array([0, 0, -math.sin(0.2), -math.cos(0.2)])
    middle = cartesian.slerp(q0, q1, 0.5)
    assert np.linalg.norm(middle) == pytest.approx(1.0)
    assert abs(middle[2]) == pytest.approx(math.sin(0.1))        # half of the 0.4 rad turn


def test_passing_the_wrist_singularity_fails(arm):
    arm, (lower, upper) = arm
    start = np.radians([0, -90, 90, -90, 30, 0])
    path = cartesian.CartesianPath(arm.fk(start), arm.fk(np.radians([0, -90, 90, -90, -30, 0])))
    with pytest.raises(cartesian.PathError, match='wrist singularity') as error:
        cartesian.follow(arm, path, start, lower, upper, STEP)
    assert 0.5 < error.value.at < 1.0


def test_out_of_reach(arm):
    arm, (lower, upper) = arm
    start = arm.fk(HOME)
    path = cartesian.CartesianPath(start, transform((1.5, 0.0, 1.2)) @ np.diag([1, -1, -1, 1]))
    with pytest.raises(cartesian.PathError, match='out of reach'):
        cartesian.follow(arm, path, HOME, lower, upper, STEP)


def test_trajectory_follows_the_profile(arm):
    arm, (lower, upper) = arm
    start = arm.fk(HOME)
    path = cartesian.CartesianPath(start, start @ transform((0.0, 0.0, 0.15)))
    profile = path.profile((1.0, 2.0, math.pi, 2 * math.pi), 0.1)    # 0.1 m/s, 0.2 m/s2
    assert profile.duration == pytest.approx(0.15 / 0.1 + 0.1 / 0.2)
    samples = cartesian.follow(arm, path, HOME, lower, upper, STEP)
    times, positions, velocities = cartesian.trajectory(arm, path, profile, samples, 0.02)
    assert times[0] == 0.0 and times[-1] == pytest.approx(profile.duration)
    assert np.all(np.diff(times) > 0) and not velocities[0].any() and not velocities[-1].any()
    assert np.allclose(positions[-1], samples[-1][1], atol=1e-6)
    tcp_speed = np.linalg.norm(np.diff([arm.fk(q)[:3, 3] for q in positions], axis=0),
                               axis=1) / np.diff(times)
    assert tcp_speed.max() == pytest.approx(0.1, abs=2e-3)


class _Linear:
    """A profile that lasts a hair over a whole number of 20 ms steps."""

    duration = 1.0000000001

    def at(self, t):
        return min(max(t / self.duration, 0.0), 1.0), 1.0 / self.duration, 0.0


def test_trajectory_times_increase_by_whole_nanoseconds(arm):
    arm, (lower, upper) = arm
    start = arm.fk(HOME)
    path = cartesian.CartesianPath(start, start @ transform((0.0, 0.0, 0.05)))
    samples = cartesian.follow(arm, path, HOME, lower, upper, STEP)
    times, _, _ = cartesian.trajectory(arm, path, _Linear(), samples, 0.02)
    # as the controller receives them: whole seconds and nanoseconds
    stamps = [(int(t), int((t % 1) * 1e9)) for t in times]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    assert max(np.diff(times)) <= 0.02 and times[-1] == _Linear.duration
