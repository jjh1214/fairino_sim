"""
Golden scenarios of recipe checks with the example cell (offline, needs moveit_py).

The steps run through OfflineWorkcell, which applies the rules of the workcell_executor node
without ROS, in virtual time with the conveyor running. Each scenario pins which step fails and
why, so that changes to the motion, the kinematics, the conveyor model or the collision checks
show up here.
"""

import importlib.util
import math
import os
import sys


from fairino_workcell.api import OfflineWorkcell, RecipeError
from fairino_workcell.motion import matrix_rpy, transform
from fairino_workcell.robot_cell import new_report
import numpy as np
import pytest
import yaml

EXAMPLES = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell')
CELL = os.path.join(EXAMPLES, 'cell.yaml')
TRACKING = os.path.join(EXAMPLES, 'tracking_cell.yaml')
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


def cell_with(tmp_path, source=CELL, **conveyor):
    """Write an example cell with other conveyor settings (None: leave the key out)."""
    data = yaml.safe_load(open(source))
    for entry in data['parts'] + data['objects']:
        for key in ('visual', 'collision'):
            if key in entry:
                entry[key] = os.path.abspath(os.path.join(EXAMPLES, entry[key]))
    settings = next(o for o in data['objects'] if o['name'] == 'conveyor')['conveyor']
    settings.update(conveyor)
    for key in [key for key, value in settings.items() if value is None]:
        del settings[key]
    path = tmp_path / 'cell.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return str(path)


@pytest.fixture(scope='module')
def exact(tmp_path_factory):
    """Return the example cell with every part exactly at the station (no placement error)."""
    return cell_with(tmp_path_factory.mktemp('exact'), placement_error=None)


def aligned(cell, count=4):
    """Return True if the first count parts lie turned alike."""
    rotations = [cell.object_pose(f'part_{n}')[:3, :3] for n in range(1, count + 1)]
    return all(np.allclose(r, rotations[0], atol=1e-6) for r in rotations)


def test_example_recipe_places_the_first_part_on_the_table(example):
    cell = checked()
    example.recipe(cell)
    assert [r['status'] for r in cell.reports] == ['OK'] * 12, cell.reports
    place = cell.core.frame_matrix('table/place', cell.state)
    assert np.allclose(cell.object_pose('part_1')[:3, 3], place[:3, 3], atol=1e-6)
    waited = cell.reports[2]
    assert waited['kind'] == 'wait' and waited['duration'] == pytest.approx(0.64, abs=0.01)
    # the open fingers come down beside the part, then close on its 80 mm sides
    assert cell.reports[4]['allowed_contacts'] == [] and cell.reports[4]['status'] == 'OK'
    grip, release = cell.reports[5], cell.reports[9]
    assert grip['message'] == ('gripped part_1, TCP 0.0 mm from part_1/grasp, fingers closed '
                               'to 80.0 mm in 0.30 s, lifting it from conveyor')
    assert grip['duration'] == pytest.approx(0.3) and release['duration'] == pytest.approx(0.3)
    assert release['message'].endswith('fingers opened in 0.30 s')
    assert cell.state.fingers == pytest.approx(0.055)          # open again
    # the next move may lift the part off the conveyor, and only that move
    assert cell.reports[6]['allowed_contacts'] == ['part_1 <-> conveyor']
    assert cell.reports[7]['allowed_contacts'] == []
    assert cell.cycle_time == pytest.approx(cell.state.time, abs=0.005)   # ms rounding


def test_index_cycle_places_four_parts():
    cell = checked()
    model = cell.core.conveyors.models['conveyor']
    placements = [model.placement(n) for n in range(1, 5)]      # (along, across, yaw)
    assert min(abs(p[1]) for p in placements) > 0.003 and min(abs(p[2]) for p in placements) > 0.02
    load_example('index_cycle').recipe(cell)
    assert all(r['status'] == 'OK' for r in cell.reports) and len(cell.reports) == 42
    place = cell.core.frame_matrix('table/place', cell.state)[:3, 3]
    for number, (x, y) in enumerate([(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08),
                                     (0.08, 0.08)], 1):
        assert np.allclose(cell.object_pose(f'part_{number}')[:3, 3], place + (x, y, 0),
                           atol=1e-6)
    assert aligned(cell)                  # turned differently on the belt, alike on the table
    summary = cell.summary()
    assert summary['grips'] == 4
    assert summary['cycle_time'] == pytest.approx(59.28, abs=0.05)
    assert summary['waiting'] == pytest.approx(0.64, abs=0.01)
    assert summary['grip_interval'] == pytest.approx(13.69, abs=0.05)
    # the conveyor has brought its four parts and stands still
    assert cell.state.belts['conveyor'].fed == 4 and not cell.state.io[READY]


def test_the_check_is_repeatable(example):
    runs = []
    for _ in range(2):
        cell = checked()
        example.recipe(cell)
        runs.append([(r['start_time'], r['end_joints_deg']) for r in cell.reports])
    assert runs[0] == runs[1]


def test_waiting_at_the_pick_position_is_hit_by_the_arriving_part(example, exact):
    cell = checked(exact)
    cell.move_joints(example.HOME, speed=1.0)
    # the gripper turned so that its fingers stand along the belt, in the way of the part
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT), rpy_deg=(180, 0, 90),
                 speed=1.0, name='down too early')
    report = cell.wait_io(READY)
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['finger_right <-> part_1']
    assert report['message'].startswith('collision after ')
    assert report['message'].endswith(' s of waiting: finger_right <-> part_1')


def test_collision_names_the_bodies_and_stops_the_robot(example):
    cell = checked()
    cell.move_joints(example.HOME)
    report = cell.move_to('table/place', xyz=(0, 0, -0.05), rpy_deg=example.AT_TABLE)
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['finger_left <-> table', 'finger_right <-> table']
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


def test_passing_close_is_a_warning(example, exact):
    cell = checked(exact)
    cell.move_joints(example.HOME)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)
    cell.wait_io(READY)
    # the finger tips come down to 4 mm above the belt, beside the part
    report = cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT - 0.008),
                          rpy_deg=example.AT_CONVEYOR)
    assert report['status'] == 'WARNING'
    assert report['closest_object'] == 'conveyor'
    assert report['min_clearance'] == pytest.approx(0.004, abs=1e-4)
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


def above_station(cell, example):
    cell.move_joints(example.HOME)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)
    cell.wait_io(READY)


def line_to_joints(cell, joints_deg, **kwargs):
    """Move the TCP in a line to where it is at the given joints."""
    target = cell.core.arm.fk(np.radians(joints_deg))
    return cell.move_line('world', xyz=target[:3, 3],
                          rpy_deg=np.degrees(matrix_rpy(target[:3, :3])), **kwargs)


def test_a_line_moves_the_tcp_straight(example):
    cell = checked()
    above_station(cell, example)
    target = cell.core.frame_matrix('conveyor/station', cell.state) @ transform(
        (0, 0, example.GRASP_HEIGHT), np.radians(example.AT_CONVEYOR))
    plan = cell.core.plan_cartesian(cell.state, target, 0.1, ['part'],
                                    new_report('pick', 'move', True))
    path = np.array(plan.check.tcp_path)
    direction = (path[-1] - path[0]) / np.linalg.norm(path[-1] - path[0])
    assert np.linalg.norm(np.cross(path - path[0], direction), axis=1).max() < 1e-6
    assert np.allclose(path[-1], target[:3, 3], atol=1e-6)
    assert plan.profile.duration == pytest.approx(2.0)      # 0.15 m at 0.1 m/s, 0.2 m/s2


def test_an_arc_over_the_table(example):
    cell = checked()
    cell.move_joints(example.HOME)
    cell.move_to('table/place', xyz=(0, 0, 0.18), rpy_deg=example.AT_TABLE)
    report = cell.move_arc('table/place', via=(0.1, 0.1, 0.18), xyz=(0.2, 0, 0.18),
                           rpy_deg=example.AT_TABLE)
    assert report['status'] == 'OK' and report['message'] == 'arc is free'
    place = cell.core.frame_matrix('table/place', cell.state)
    end = cell.core.arm.fk(np.radians(report['end_joints_deg']))
    assert np.allclose(end[:3, 3], (place @ transform((0.2, 0, 0.18)))[:3, 3], atol=3e-4)
    assert report['duration'] == pytest.approx(1.55, abs=0.01)    # half circle of r = 0.1 m


def test_picking_at_the_nominal_station_hits_a_misplaced_part(example):
    cell = checked()           # part_1 lies 23 mm across the belt from the station, turned
    above_station(cell, example)
    report = cell.move_line('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT),
                            rpy_deg=example.AT_CONVEYOR, speed=0.1, name='pick at the station')
    assert report['status'] == 'FAILED'
    assert report['collisions'] == ['finger_right <-> part_1'] and report['collision_at'] > 0.5
    # going where the part lies, the fingers come down beside it
    for xyz in ((0, 0, example.APPROACH), (0, 0, 0)):
        report = cell.move_line('conveyor/part/grasp', xyz=xyz, rpy_deg=example.AT_CONVEYOR)
        assert report['status'] == 'OK', report
    assert cell.grip()['message'].startswith('gripped part_1, TCP 0.0 mm from part_1/grasp')


def test_the_fingers_centre_a_part_between_them(example, exact):
    cell = checked(exact)
    above_station(cell, example)
    # 6 mm off the middle of the part, across the belt (the fingers close across it)
    cell.move_line('conveyor/station', xyz=(0, 0.006, example.GRASP_HEIGHT),
                   rpy_deg=example.AT_CONVEYOR, speed=0.1)
    report = cell.grip()
    assert report['status'] == 'OK'
    assert report['message'] == ('gripped part_1, TCP 6.0 mm from part_1/grasp, fingers closed '
                                 'to 80.0 mm in 0.30 s (centring it by 6.0 mm), lifting it '
                                 'from conveyor')
    tcp = cell.core.arm.fk(cell.state.joints)
    grasp = cell.object_pose('part_1') @ transform((0, 0, example.GRASP_HEIGHT))
    assert np.allclose(grasp[:3, 3], tcp[:3, 3], atol=1e-6)    # pushed to the middle
    # too far off: the fingers close without touching it
    cell = checked(exact)
    above_station(cell, example)
    for z in (example.GRASP_HEIGHT + example.APPROACH, example.GRASP_HEIGHT):   # beside it
        assert cell.move_line('conveyor/station', xyz=(0, 0.13, z),
                              rpy_deg=example.AT_CONVEYOR)['status'] != 'FAILED'
    report = cell.grip()
    assert report['status'] == 'FAILED'
    assert report['message'] == ('nothing to grip: the fingers close without touching part_1: '
                                 'it is not between them (part_1/grasp is 130 mm from the TCP)')


def test_frames_of_the_tool_and_of_the_part_at_the_conveyor(example):
    cell = checked()
    cell.move_joints(example.HOME)
    report = cell.move_to('conveyor/part/grasp')
    assert report['status'] == 'FAILED'
    assert report['message'] == ('conveyor/part/grasp: conveyor conveyor has no part ready '
                                 '(wait for conveyor/part_ready first)')
    cell.wait_io(READY)
    before = cell.core.arm.fk(cell.state.joints)
    report = cell.move_line('tcp', xyz=(0, 0, -0.05), name='back 5 cm')
    after = cell.core.arm.fk(cell.state.joints)
    assert report['status'] == 'OK'
    assert np.allclose(after[:3, 3], before[:3, 3] - 0.05 * before[:3, 2], atol=1e-6)
    assert np.allclose(after[:3, :3], before[:3, :3], atol=1e-6)    # orientation kept
    part = cell.core.frame_matrix('conveyor/part', cell.state)
    assert np.allclose(part, cell.object_pose('part_1'))


def test_a_line_through_the_wrist_singularity_fails():
    cell = checked(joints=np.radians([0, -90, 90, -90, 30, 0]))
    report = line_to_joints(cell, [0, -90, 90, -90, -30, 0])
    assert report['status'] == 'FAILED'
    assert report['message'].startswith('j6 leaves its limits at ')
    assert 'after passing close to the wrist singularity' in report['message']


def test_a_line_close_to_the_wrist_singularity_warns():
    cell = checked(joints=np.radians([0, -90, 90, -90, 12, 0]))
    report = line_to_joints(cell, [0, -90, 90, -90, 2.5, 40], speed=1.0)
    assert report['status'] == 'WARNING'
    assert report['message'] == ('line is free; passes close to the wrist singularity at 100% '
                                 '(j5 2.5 deg)')


def test_a_line_slows_down_for_the_joint_speed_limits(example):
    cell = checked()
    cell.core.max_velocity = [math.radians(30)] * 6       # slow joints: 30 deg/s
    above_station(cell, example)
    report = cell.move_line('conveyor/station', xyz=(0.3, 0, 0.18), rpy_deg=example.AT_CONVEYOR,
                            speed=1.0)
    assert report['status'] == 'WARNING'
    assert 'slowed down to ' in report['message']
    assert 'by the speed limit of j' in report['message']


def test_pick_on_the_move_cycle():
    cell = checked(TRACKING)
    load_example('pick_on_the_move').recipe(cell)
    assert all(r['status'] == 'OK' for r in cell.reports) and len(cell.reports) == 30
    picks = [r for r in cell.reports if r['kind'] == 'pick']
    assert [r['message'].split(':')[0] for r in picks] == [
        f'picked part_{n} on the move' for n in range(1, 5)]
    assert all(r['message'].endswith('fingers closed to 80.0 mm in 0.30 s') for r in picks)
    place = cell.core.frame_matrix('table/place', cell.state)[:3, 3]
    for number, (x, y) in enumerate([(-0.08, -0.08), (0.08, -0.08), (-0.08, 0.08),
                                     (0.08, 0.08)], 1):
        assert np.allclose(cell.object_pose(f'part_{number}')[:3, 3], place + (x, y, 0),
                           atol=1e-6)
    assert aligned(cell)
    summary = cell.summary()
    assert summary['grips'] == 4 and summary['missed'] == 0
    # the belt feeds every 15 s; the parts lie up to 3 cm ahead or behind
    assert summary['grip_interval'] == pytest.approx(15.19, abs=0.01)
    assert summary['cycle_time'] == pytest.approx(67.02, abs=0.05)
    assert summary['waiting'] == pytest.approx(6.18, abs=0.02)


def test_a_pick_on_the_move_gives_the_trajectory_the_robot_runs():
    cell = checked(TRACKING)
    cell.move_joints(load_example('pick_and_place').HOME)
    cell.move_to('conveyor/sensor', xyz=(0.3, 0, 0.2), rpy_deg=(180, 0, 180))
    cell.wait_io(READY)
    report = new_report('pick', 'pick', True)
    plan = cell.core.plan_pick_moving(cell.state, 'conveyor', '', transform(
        (0, 0, 0), np.radians((180, 0, 180))), 0.1, 0.12, 0.3, None, report)
    times, positions, velocities, _ = plan.trajectory(0.02)
    assert times[-1] == pytest.approx(plan.path.duration)
    assert np.allclose(positions[-1], plan.goal, atol=1e-6) and not velocities[-1].any()
    # the fingers start closing when the TCP follows the part, and touch it at the grip
    assert plan.close_at == pytest.approx(plan.grip_at - 0.3)
    assert plan.fingers == pytest.approx(0.04)


def test_picking_on_the_move_with_the_fingers_along_the_part_fails():
    cell = checked(TRACKING)
    cell.move_joints(load_example('pick_and_place').HOME)
    cell.move_to('conveyor/sensor', xyz=(0.3, 0, 0.2), rpy_deg=(180, 0, 180))
    cell.wait_io(READY)
    # turned 90 degrees, the 110 mm opening is too narrow for the 120 mm length of the part
    report = cell.pick_moving('conveyor', rpy_deg=(180, 0, 90))
    assert report['status'] == 'FAILED'
    assert report['message'].startswith('collision at ')
    assert report['collisions'][0].startswith('finger_') and 'part_1' in report['collisions'][0]
    report = cell.pick_moving('conveyor', rpy_deg=(180, 0, 180), grip_time=0.1)
    assert report['message'] == ('grip_time 0.10 s is shorter than the 0.30 s the fingers need '
                                 'to close on part_1')


def test_parts_too_far_to_catch_are_missed(tmp_path):
    cell = checked(cell_with(tmp_path, TRACKING, speed=0.1, spacing=0.3))
    load_example('pick_on_the_move').recipe(cell)
    picks = [r for r in cell.reports if r['kind'] == 'pick' and r['status'] == 'OK']
    picked = [r['message'].split(':')[0] for r in picks]
    assert picked == ['picked part_1 on the move', 'picked part_4 on the move']
    assert picks[1]['message'].endswith('(part_2, part_3 too far to catch)')
    missed = sorted(part for r in cell.reports for part in r['missed_parts'])
    assert missed == ['part_2', 'part_3']
    assert cell.summary()['missed'] == 2


def test_picking_on_the_move_needs_a_part_past_the_sensor():
    report = checked(TRACKING).pick_moving('conveyor')
    assert report['status'] == 'FAILED'
    assert report['message'] == ('no part has passed the sensor of conveyor conveyor: '
                                 'wait for conveyor/part_ready first')
    report = checked().pick_moving('conveyor')
    assert report['message'] == "'conveyor' is not a continuous conveyor"


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


def test_grasp_miss(example, exact):
    cell = checked(exact)
    cell.move_joints(example.HOME)
    cell.move_to('conveyor/station', xyz=(0, 0, example.GRASP_HEIGHT + example.APPROACH),
                 rpy_deg=example.AT_CONVEYOR)
    cell.wait_io(READY)
    report = cell.grip()
    assert report['status'] == 'FAILED'
    assert report['message'] == ('nothing to grip: the fingers close without touching part_1: '
                                 'it is not between them (part_1/grasp is 150 mm from the TCP)')
    assert cell.release()['status'] == 'WARNING'      # nothing to release


def test_a_failed_step_stops_the_recipe(example):
    cell = OfflineWorkcell(CELL)
    cell.move_joints(example.HOME)
    with pytest.raises(RecipeError) as error:
        cell.grip(name='grip too early')
    assert error.value.report['name'] == 'grip too early'
    assert len(cell.reports) == 2
