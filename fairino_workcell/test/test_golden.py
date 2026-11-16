"""
Golden scenarios of recipe checks with the example cell (offline, needs moveit_py).

The steps run through OfflineWorkcell, which applies the rules of the workcell_executor node
without ROS. Each scenario pins which step fails and why, so that changes to the motion, the
kinematics or the collision checks show up here.
"""

import importlib.util
import os

from fairino_workcell.api import OfflineWorkcell, RecipeError
import numpy as np
import pytest

EXAMPLES = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell')
CELL = os.path.join(EXAMPLES, 'cell.yaml')


@pytest.fixture(scope='module')
def example():
    spec = importlib.util.spec_from_file_location(
        'pick_and_place', os.path.join(EXAMPLES, 'pick_and_place.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checked(**kwargs):
    return OfflineWorkcell(CELL, stop_on_failure=False, **kwargs)


def pick_and_lift(cell, example):
    """Do steps 1-5 of the example recipe: grip the part and lift it above the conveyor."""
    cell.move_joints(example.HOME, name='home')
    cell.move_to('part/grasp', xyz=(0, 0, example.APPROACH), rpy_deg=example.AT_CONVEYOR)
    cell.move_to('part/grasp', rpy_deg=example.AT_CONVEYOR, speed=0.1, allow_contact=['part'])
    cell.grip()
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)


def test_example_recipe_places_the_part_on_the_table(example):
    cell = checked()
    example.recipe(cell)
    assert [r['status'] for r in cell.reports] == ['OK'] * 10, cell.reports
    place = cell.core.frame_matrix('table/place', cell.state)
    assert np.allclose(cell.object_pose('part')[:3, 3], place[:3, 3], atol=1e-6)
    assert cell.reports[3]['message'].endswith('lifting it from conveyor')
    assert cell.cycle_time == pytest.approx(17.03, abs=0.05)
    # the next move after grip may lift the part off the conveyor, and only that move
    assert cell.reports[4]['allowed_contacts'] == ['part <-> conveyor']
    assert cell.reports[5]['allowed_contacts'] == []


def test_the_check_is_repeatable(example):
    runs = []
    for _ in range(2):
        cell = checked()
        example.recipe(cell)
        runs.append([r['end_joints_deg'] for r in cell.reports])
    assert runs[0] == runs[1]


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
    pick_and_lift(cell, example)
    cell.move_to('table/place', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_TABLE)
    report = cell.move_to('table/place', xyz=(0, 0, example.GRASP_HEIGHT),
                          rpy_deg=example.AT_TABLE, speed=0.1, name='place')
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['part <-> table']
    assert report['held_object'] == 'part'


def test_passing_close_is_a_warning(example):
    cell = checked()
    cell.move_joints(example.HOME)
    cell.move_to('part/grasp', xyz=(0, 0, example.APPROACH), rpy_deg=example.AT_CONVEYOR)
    report = cell.move_to('part/grasp', xyz=(0, 0, -0.02), rpy_deg=example.AT_CONVEYOR,
                          allow_contact=['part'])
    assert report['status'] == 'WARNING'
    assert report['closest_object'] == 'conveyor' and report['min_clearance'] < 0.01
    assert report['message'].startswith(f"{report['min_clearance'] * 1000:.1f} mm from conveyor")


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
    assert cell.move_to('part/handle')['message'] == "object 'part' has no frame 'handle'"
    assert cell.move_to('robot2/base')['message'] == "unknown frame 'robot2/base'"
    report = cell.move_joints(example.HOME, speed=1.5)
    assert report['message'] == 'speed must be in (0, 1], got 1.5'


def test_grasp_miss(example):
    cell = checked()
    cell.move_joints(example.HOME)
    cell.move_to('part/grasp', xyz=(0, 0, example.APPROACH), rpy_deg=example.AT_CONVEYOR)
    report = cell.grip()
    assert report['status'] == 'FAILED'
    assert report['message'] == ('nothing to grip: part/grasp is 150 mm from the TCP '
                                 '(tolerance 10 mm)')
    assert cell.release()['status'] == 'WARNING'      # nothing to release


def test_a_failed_step_stops_the_recipe(example):
    cell = OfflineWorkcell(CELL)
    cell.move_joints(example.HOME)
    with pytest.raises(RecipeError) as error:
        cell.grip(name='grip too early')
    assert error.value.report['name'] == 'grip too early'
    assert len(cell.reports) == 2
