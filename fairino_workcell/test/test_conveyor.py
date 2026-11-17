import math

from fairino_workcell.cell import Conveyor, PlacementError
from fairino_workcell.conveyor import ConveyorModel, Conveyors
from fairino_workcell.motion import transform
from fairino_workcell.state import CellState
import numpy as np
import pytest

READY = 'belt/part_ready'


def line(speed=0.5, spacing=0.25, count=0, length=1.0, error=None):
    """Return conveyors with one belt from x=0 to x=length, and its start state."""
    spec = Conveyor(part='box', entry='in', station='stop', speed=speed, spacing=spacing,
                    count=count, placement_error=error)
    model = ConveyorModel('belt', spec, transform((0.0, 0.0, 0.8)),
                          transform((length, 0.0, 0.8), (0.0, 0.0, math.pi / 2)))
    conveyors = Conveyors({'belt': model})
    state = CellState(joints=(0.0,) * 6, poses={})
    conveyors.start(state)
    return conveyors, state


def distances(state):
    return {name: round(distance, 6) for name, distance in state.belts['belt'].items}


def test_parts_are_put_on_at_the_spacing_and_stop_at_the_station():
    conveyors, state = line()
    assert distances(state) == {'box_1': 0.0} and state.io[READY] is False
    conveyors.advance(state, 1.0)                      # 0.5 m of belt travel
    assert distances(state) == {'box_1': 0.5, 'box_2': 0.25, 'box_3': 0.0}
    assert state.parts == {'box_1': 'box', 'box_2': 'box', 'box_3': 'box'}
    conveyors.advance(state, 5.0)                      # box_1 arrives after 2 s, then all stop
    assert state.time == pytest.approx(6.0)
    assert distances(state) == {'box_1': 1.0, 'box_2': 0.75, 'box_3': 0.5, 'box_4': 0.25,
                                'box_5': 0.0}
    assert state.io[READY] is True and not conveyors.moving(state)
    # parts take the orientation of the station frame and follow the belt
    assert np.allclose(state.poses['box_1'], transform((1.0, 0.0, 0.8), (0, 0, math.pi / 2)))
    assert np.allclose(state.poses['box_3'][:3, 3], (0.5, 0.0, 0.8))


def test_taking_the_part_lets_the_belt_run_on():
    conveyors, state = line(count=2)
    conveyors.advance(state, 10.0)
    assert distances(state) == {'box_1': 1.0, 'box_2': 0.75}
    state.poses.pop('box_1')
    conveyors.take(state, 'box_1')
    assert state.io[READY] is False and conveyors.moving(state)
    assert conveyors.time_until(state, READY, True, 60.0) == (pytest.approx(0.5), False)
    conveyors.advance(state, 0.5)
    assert distances(state) == {'box_2': 1.0} and state.io[READY] is True


def test_waiting_for_parts_that_never_come():
    conveyors, state = line(count=1)
    conveyors.advance(state, 10.0)
    state.poses.pop('box_1')
    conveyors.take(state, 'box_1')
    assert not conveyors.moving(state)
    assert conveyors.time_until(state, READY, True, 60.0) == (None, True)      # never
    assert conveyors.time_to_still(state, 60.0) == 0.0
    slow, state = line(speed=0.01)
    assert slow.time_until(state, READY, True, 60.0) == (None, False)         # not in time
    assert slow.time_until(state, READY, True, 200.0) == (pytest.approx(100.0), False)


def test_prediction_leaves_the_state_alone():
    conveyors, state = line()
    frames, parts = conveyors.predict(state, [0.0, 0.25, 1.0, 3.0])
    assert [sorted(frame) for frame in frames] == [
        ['box_1'], ['box_1'], ['box_1', 'box_2', 'box_3'],
        ['box_1', 'box_2', 'box_3', 'box_4', 'box_5']]
    assert frames[1]['box_1'][0, 3] == pytest.approx(0.125)
    assert frames[3]['box_1'][0, 3] == pytest.approx(1.0)       # stopped at the station
    assert set(parts) == {'box_1', 'box_2', 'box_3', 'box_4', 'box_5'}
    assert state.time == 0.0 and distances(state) == {'box_1': 0.0}


def test_two_conveyors_share_the_part_numbers():
    spec = Conveyor(part='box', entry='in', station='stop', speed=1.0, spacing=0.5, count=1)
    conveyors = Conveyors({
        'a': ConveyorModel('a', spec, transform((0, 0, 0)), transform((1, 0, 0))),
        'b': ConveyorModel('b', spec, transform((0, 1, 0)), transform((2, 1, 0)))})
    state = CellState(joints=(0.0,) * 6, poses={})
    conveyors.start(state)
    assert sorted(state.parts) == ['box_1', 'box_2']
    assert conveyors.time_until(state, 'b/part_ready', True, 10.0) == (pytest.approx(2.0), False)
    conveyors.advance(state, 1.5)
    assert state.io == {'a/part_ready': True, 'b/part_ready': False}


def continuous_line(speed=0.5, spacing=0.4, count=3):
    """Return conveyors with a continuous belt: sensor at x=0.5, exit at x=1.0."""
    spec = Conveyor(part='box', entry='in', station='eye', speed=speed, spacing=spacing,
                    count=count, mode='continuous', exit_frame='out')
    model = ConveyorModel('belt', spec, transform((0.0, 0.0, 0.8)), transform((0.5, 0.0, 0.8)),
                          transform((1.0, 0.0, 0.8)))
    conveyors = Conveyors({'belt': model})
    state = CellState(joints=(0.0,) * 6, poses={})
    conveyors.start(state)
    return conveyors, model, state


def test_a_continuous_belt_keeps_running_and_loses_unpicked_parts():
    conveyors, model, state = continuous_line()
    assert state.io[READY] is False
    assert conveyors.time_until(state, READY, True, 60.0) == (pytest.approx(1.0), False)
    conveyors.advance(state, 1.0)                 # box_1 reaches the sensor
    assert state.io[READY] is True and distances(state) == {'box_1': 0.5, 'box_2': 0.1}
    conveyors.advance(state, 1.0)                 # box_1 reaches the exit and leaves
    assert distances(state) == {'box_2': 0.6, 'box_3': 0.2}
    assert conveyors.missed(state) == ['box_1'] and 'box_1' not in state.poses
    assert model.ready_part(state.belts['belt']) == 'box_2'
    state.poses.pop('box_2')
    conveyors.take(state, 'box_2')               # picked on the move
    assert state.io[READY] is False and conveyors.moving(state)
    assert conveyors.time_until(state, READY, True, 60.0) == (pytest.approx(0.6), False)
    conveyors.advance(state, 10.0)
    assert conveyors.missed(state) == ['box_1', 'box_3'] and not conveyors.moving(state)
    assert conveyors.time_until(state, READY, True, 60.0) == (None, True)


def test_parts_on_a_continuous_belt_move_at_the_belt_velocity():
    conveyors, model, state = continuous_line(count=1)
    frames, _ = conveyors.predict(state, [0.0, 0.5, 1.0])
    assert [frame['box_1'][0, 3] for frame in frames] == pytest.approx([0.0, 0.25, 0.5])
    assert model.velocity == pytest.approx([0.5, 0.0, 0.0])


def yaw_of(pose):
    return math.atan2(pose[1, 0], pose[0, 0])


def test_placement_errors_are_drawn_per_part_and_the_same_every_time():
    error = PlacementError(across=0.03, along=0.02, yaw=math.radians(25), seed=7)
    conveyors, state = line(count=3, error=error)
    model = conveyors.models['belt']
    conveyors.advance(state, 10.0)                     # all three on the belt, the first waits
    seen = set()
    for number in (1, 2, 3):
        along, across, yaw = model.placement(number)
        assert abs(along) <= 0.02 and abs(across) <= 0.03 and abs(yaw) <= math.radians(25)
        pose = state.poses[f'box_{number}']
        # across the belt is the station frame's -x here (the belt runs along x, z is up)
        assert pose[1, 3] == pytest.approx(across * model.across[1])
        assert yaw_of(pose) == pytest.approx(math.pi / 2 + yaw)        # station frame turned
        seen.add((round(across, 6), round(yaw, 6)))
    assert len(seen) == 3                                              # every part differs
    # the stop sensor still stops the first part at the station along the belt
    assert state.poses['box_1'][0, 3] == pytest.approx(1.0)
    again, other = line(count=3, error=error)
    again.advance(other, 10.0)
    assert all(np.allclose(other.poses[name], state.poses[name]) for name in state.poses)
    shifted, third = line(count=3, error=PlacementError(across=0.03, seed=8))
    shifted.advance(third, 10.0)
    assert not np.allclose(third.poses['box_1'], state.poses['box_1'])


def test_along_the_belt_a_part_arrives_earlier_or_later():
    error = PlacementError(along=0.1, seed=3)
    conveyors, state = line(count=1, error=error)
    along = conveyors.models['belt'].placement(1)[0]
    arrival, _ = conveyors.time_until(state, READY, True, 60.0)
    assert arrival == pytest.approx((1.0 - along) / 0.5)
    assert abs(along) > 0.01
