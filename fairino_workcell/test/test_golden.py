"""
Golden scenarios of recipe checks with the example cell (offline, needs moveit_py).

The steps run through OfflineWorkcell, which applies the rules of the workcell_executor node
without ROS, in virtual time with the conveyor running. Each scenario pins which step fails and
why, so that changes to the motion, the kinematics, the conveyor model or the collision checks
show up here.
"""

import importlib.util
import os
import sys

from fairino_workcell.api import OfflineWorkcell, RecipeError
import numpy as np
import pytest
import yaml

EXAMPLES = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell')
CELL = os.path.join(EXAMPLES, 'cell.yaml')
READY = 'conveyor/part_ready'


def load_example(name):
    sys.path.insert(0, EXAMPLES)
    try:
        spec = importlib.util.spec_from_file_location(name, os.path.join(EXAMPLES, f'{name}.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(EXAMPLES)
    return module


@pytest.fixture(scope='module')
def example():
    return load_example('pick_and_place')


def checked(cell_file=CELL, **kwargs):
    return OfflineWorkcell(cell_file, stop_on_failure=False, **kwargs)


def cell_with(tmp_path, **conveyor):
    """Write the example cell with other conveyor settings (mesh paths made absolute)."""
    data = yaml.safe_load(open(CELL))
    for entry in data['parts'] + data['objects']:
        for key in ('visual', 'collision'):
            if key in entry:
                entry[key] = os.path.abspath(os.path.join(EXAMPLES, entry[key]))
    next(o for o in data['objects'] if o['name'] == 'conveyor')['conveyor'].update(conveyor)
    path = tmp_path / 'cell.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return str(path)


def test_example_recipe_places_the_first_part_on_the_table(example):
    cell = checked()
    example.recipe(cell)
    assert [r['status'] for r in cell.reports] == ['OK'] * 11, cell.reports
    place = cell.core.frame_matrix('table/place', cell.state)
    assert np.allclose(cell.object_pose('part_1')[:3, 3], place[:3, 3], atol=1e-6)
    waited = cell.reports[2]
    assert waited['kind'] == 'wait' and waited['duration'] == pytest.approx(0.64, abs=0.01)
    assert cell.reports[4]['message'] == ('gripped part_1, TCP 0.0 mm from part_1/grasp, '
                                          'lifting it from conveyor')
    # the next move may lift the part off the conveyor, and only that move
    assert cell.reports[5]['allowed_contacts'] == ['part_1 <-> conveyor']
    assert cell.reports[6]['allowed_contacts'] == []
    assert cell.cycle_time == pytest.approx(cell.state.time, abs=0.005)   # ms rounding


def test_index_cycle_places_four_parts():
    cell = checked()
    load_example('index_cycle').recipe(cell)
    assert all(r['status'] == 'OK' for r in cell.reports) and len(cell.reports) == 38
    place = cell.core.frame_matrix('table/place', cell.state)[:3, 3]
    for number, (x, y) in enumerate([(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08),
                                     (0.08, 0.08)], 1):
        assert np.allclose(cell.object_pose(f'part_{number}')[:3, 3], place + (x, y, 0),
                           atol=1e-6)
    summary = cell.summary()
    assert summary['grips'] == 4
    assert summary['cycle_time'] == pytest.approx(57.69, abs=0.05)
    assert summary['waiting'] == pytest.approx(0.64, abs=0.01)
    assert summary['grip_interval'] == pytest.approx(13.39, abs=0.05)
    # the conveyor has brought its four parts and stands still
    assert cell.state.belts['conveyor'].fed == 4 and not cell.state.io[READY]


def test_the_check_is_repeatable(example):
    runs = []
    for _ in range(2):
        cell = checked()
        example.recipe(cell)
        runs.append([(r['start_time'], r['end_joints_deg']) for r in cell.reports])
    assert runs[0] == runs[1]


def test_waiting_at_the_pick_position_is_hit_by_the_arriving_part(example):
    cell = checked()
    cell.move_joints(example.HOME, speed=1.0)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT),
                 rpy_deg=example.AT_CONVEYOR, speed=1.0, name='down too early')
    report = cell.wait_io(READY)
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['tool_link <-> part_1']
    assert report['message'].startswith('collision after ')
    assert report['message'].endswith(' s of waiting: tool_link <-> part_1')


def test_collision_names_the_bodies_and_stops_the_robot(example):
    cell = checked()
    cell.move_joints(example.HOME)
    report = cell.move_to('table/place', xyz=(0, 0, -0.05), rpy_deg=example.AT_TABLE)
    assert report['status'] == 'FAILED'
    assert 'tool_link <-> table' in report['collisions']
    assert 0.5 < report['collision_at'] < 1.0
    assert report['end_joints_deg'] == report['start_joints_deg']   # the move was not done


def test_placing_without_allowed_contact_hits_the_table(example):
    cell = checked()
    cell.move_joints(example.HOME)
    example.pick(cell)
    cell.move_to('table/place', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_TABLE)
    report = cell.move_to('table/place', xyz=(0, 0, example.GRASP_HEIGHT),
                          rpy_deg=example.AT_TABLE, speed=0.1, name='place')
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['part_1 <-> table']
    assert report['held_object'] == 'part_1'


def test_passing_close_is_a_warning(example):
    cell = checked()
    cell.move_joints(example.HOME)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)
    cell.wait_io(READY)
    report = cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT - 0.02),
                          rpy_deg=example.AT_CONVEYOR, allow_contact=['part'])
    assert report['status'] == 'WARNING'
    assert report['closest_object'] == 'conveyor' and report['min_clearance'] < 0.01
    assert report['message'].startswith(f"{report['min_clearance'] * 1000:.1f} mm from conveyor")


def test_waits_for_parts_that_do_not_come(example, tmp_path):
    cell = checked(cell_with(tmp_path, count=1))
    cell.move_joints(example.HOME)
    assert cell.wait_io(READY, timeout=1.0)['message'] == (
        'timed out after 1.0 s waiting for conveyor/part_ready = true')
    example.pick(cell)
    report = cell.wait_io(READY)
    assert report['status'] == 'FAILED'
    assert report['message'] == ('conveyor/part_ready will not become true: '
                                 'conveyor conveyor has no more parts')


def test_io_steps(example):
    cell = checked()
    assert cell.set_io('gripper/air', True)['message'] == 'gripper/air = true'
    assert cell.wait_io('gripper/air')['message'] == 'gripper/air is already true'
    report = cell.wait_io('vision/ok', name='wait for vision')
    assert report['status'] == 'OK' and report['duration'] == 0.0
    assert report['message'] == 'assumed vision/ok = true (set by another program)'
    report = cell.set_io(READY, False)
    assert report['status'] == 'FAILED' and 'part sensor of conveyor conveyor' in report['message']
    assert cell.wait(2.5)['duration'] == 2.5 and cell.state.time == pytest.approx(2.5)


def test_joint_limits():
    report = checked().move_joints([0, 100, 0, 0, 0, 0])
    assert report['status'] == 'FAILED'
    assert report['message'].startswith('outside the joint limits: j2 100.0 deg')


def test_unreachable_target():
    report = checked().move_to('world', xyz=(2.0, 0.0, 1.0))
    assert report['status'] == 'FAILED'
    assert report['message'] == 'target is not reachable: TCP at (2.000, 0.000, 1.000) m'


def test_unknown_frame_and_bad_speed(example):
    cell = checked()
    assert cell.move_to('conveyor/handle')['message'] == "object 'conveyor' has no frame 'handle'"
    assert cell.move_to('robot2/base')['message'] == "unknown frame 'robot2/base'"
    report = cell.move_joints(example.HOME, speed=1.5)
    assert report['message'] == 'speed must be in (0, 1], got 1.5'


def test_grasp_miss(example):
    cell = checked()
    cell.move_joints(example.HOME)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)
    cell.wait_io(READY)
    report = cell.grip()
    assert report['status'] == 'FAILED'
    assert report['message'] == ('nothing to grip: part_1/grasp is 150 mm from the TCP '
                                 '(tolerance 10 mm)')
    assert cell.release()['status'] == 'WARNING'      # nothing to release


def test_a_failed_step_stops_the_recipe(example):
    cell = OfflineWorkcell(CELL)
    cell.move_joints(example.HOME)
    with pytest.raises(RecipeError) as error:
        cell.grip(name='grip too early')
    assert error.value.report['name'] == 'grip too early'
    assert len(cell.reports) == 2
