import math

from fairino_workcell.motion import transform
from fairino_workcell.tracking import smooth, Timeline, TrackingMotion
import numpy as np
import pytest

LIMITS = (1.0, 2.0, math.pi, 2.0 * math.pi)
BELT = np.array([0.06, 0.0, 0.0])
DOWN = (math.pi, 0.0, math.pi)


@pytest.fixture(scope='module')
def pick():
    start = transform((0.2, -0.6, 1.0), DOWN)
    part = transform((-0.4, -0.6, 0.8))
    return TrackingMotion(start, part, BELT, transform((0.0, 0.0, 0.03), DOWN), approach=0.1,
                          lift=0.12, limits=LIMITS, speed=0.3, grip_time=0.3), part


def velocity(motion, t, dt=1e-4):
    return (motion.position(t + dt) - motion.position(t - dt)) / (2 * dt)


def test_the_tcp_comes_in_step_with_the_part_above_it(pick):
    motion, part = pick
    t = motion.sync_time
    assert velocity(motion, t) == pytest.approx(BELT, abs=1e-4)
    above = (part @ transform((0.0, 0.0, 0.03)))[:3, 3] + BELT * t + (0.0, 0.0, 0.1)
    assert motion.position(t) == pytest.approx(above, abs=1e-9)


def test_the_tcp_follows_the_part_while_gripping(pick):
    motion, part = pick
    grasp = (part @ transform((0.0, 0.0, 0.03), DOWN))
    for t in np.linspace(motion.grip_at - 0.3, motion.grip_at, 5):
        expected = grasp[:3, 3] + BELT * t
        assert motion.pose_at(t)[:3, 3] == pytest.approx(expected, abs=1e-9)
        assert np.allclose(motion.pose_at(t)[:3, :3], grasp[:3, :3], atol=1e-9)
    assert velocity(motion, motion.grip_at - 0.1) == pytest.approx(BELT, abs=1e-6)


def test_the_lift_follows_the_belt_and_the_tcp_stops(pick):
    motion, _ = pick
    lifted = motion.position(motion.lift_end) - motion.position(motion.grip_at)
    belt_travel = BELT * (motion.lift_end - motion.grip_at)
    assert lifted == pytest.approx(belt_travel + (0.0, 0.0, 0.12), abs=1e-9)
    assert velocity(motion, motion.duration - 1e-3) == pytest.approx([0, 0, 0], abs=1e-3)
    assert motion.position(motion.duration + 1.0) == pytest.approx(
        motion.position(motion.duration))


def test_speed_and_acceleration_stay_within_the_limits(pick):
    motion, _ = pick
    times = np.linspace(0.0, motion.duration, 4001)
    dt = times[1] - times[0]
    positions = np.array([motion.position(t) for t in times])
    speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1) / dt
    accels = np.linalg.norm(np.diff(positions, 2, axis=0), axis=1) / dt ** 2
    assert speeds.max() <= 0.3 + 1e-3 and accels.max() <= 0.6 + 1e-2
    assert Timeline(motion.duration).at(motion.duration / 2)[0] == pytest.approx(0.5)


def test_smooth_rise():
    assert smooth(0.0) == 0.0 and smooth(1.0) == 1.0 and smooth(0.5) == pytest.approx(0.5)
    assert smooth(-1.0) == 0.0 and smooth(2.0) == 1.0
