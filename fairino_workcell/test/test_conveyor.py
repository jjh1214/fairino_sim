import math

from fairino_workcell.cell import Conveyor
from fairino_workcell.conveyor import ConveyorModel, Conveyors
from fairino_workcell.motion import transform
from fairino_workcell.state import CellState
import numpy as np
import pytest

READY = 'belt/part_ready'


def line(speed=0.5, spacing=0.25, count=0, length=1.0):
    """Return conveyors with one belt from x=0 to x=length, and its start state."""
    spec = Conveyor(part='box', entry='in', station='stop', speed=speed, spacing=spacing,
                    count=count)
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
