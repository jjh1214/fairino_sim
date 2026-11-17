"""Tests of the moveit_py path checker with the example cell (needs moveit_py)."""

from dataclasses import replace
import math
import os

from fairino_sim import description
from fairino_workcell import checker as checker_module
from fairino_workcell import motion
from fairino_workcell.cell import load_cell
import numpy as np
import pytest

EXAMPLE = os.path.join(os.path.dirname(__file__), '..', 'examples', 'conveyor_cell',
                       'cell.yaml')
HOME = [0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0]
ABOVE_PART = [-1.3998, -1.8346, -1.5979, -1.2799, 1.5708, -2.9706]
INSIDE_CONVEYOR = [1.3998, -1.013, 1.8508, -2.4087, -1.5708, 2.9706]
STATION = motion.transform((0.0, -0.6, 0.8))          # where parts stop on the conveyor


@pytest.fixture(scope='module')
def setup():
    cell = load_cell(EXAMPLE)
    tool = description.load_tool(cell.robot.tool_file)
    package = description.moveit_config_package(cell.robot.model)
    controllers = os.path.join(description.share_dir(package), 'config', 'ros2_controllers.yaml')
    urdf = description.build_urdf(cell.robot.model, controllers, base_xyz=cell.robot.base.xyz,
                                  base_rpy=cell.robot.base.rpy, tool=tool)
    fingers = dict(zip(description.FINGER_LINKS, description.FINGER_JOINTS))
    checker = checker_module.Checker(cell, urdf, description.build_srdf(cell.robot.model, tool),
                                     f'{cell.robot.model}_group',
                                     ['tool_link'] + list(fingers), fingers)
    checker.fingers = tool.fingers.stroke           # open
    poses = {obj.name: motion.transform(obj.pose.xyz, obj.pose.rpy) for obj in cell.objects}
    return checker, poses


def free(checker, poses):
    checker.sync(poses, {}, [])
    return checker


def test_home_is_free_and_has_clearance(setup):
    checker = free(*setup)
    checker.set_joints(HOME)
    assert not checker.in_collision()
    distance, body, obj = checker.clearance()
    assert 0.02 < distance < 1.0 and body and obj


def test_collision_pairs_are_named(setup):
    checker = free(*setup)
    checker.set_joints(INSIDE_CONVEYOR)
    assert checker.in_collision()
    pairs = checker.collisions()
    assert pairs == ['finger_left <-> conveyor', 'finger_right <-> conveyor']
    allowed = {(body, obj) for body in ('tool_link', 'finger_left', 'finger_right',
                                        'forearm_link', 'upperarm_link', 'wrist1_link',
                                        'wrist2_link', 'wrist3_link')
               for obj in ('conveyor', 'part')}
    assert checker.collisions(allowed_pairs=allowed) == []


def test_tcp_pose_above_the_part(setup):
    checker, poses = setup
    free(checker, poses)
    tcp = checker.tcp_pose(ABOVE_PART)
    assert tcp[:3, 3] == pytest.approx([0.0, -0.6, 0.966], abs=2e-3)
    assert tcp[:3, 2] == pytest.approx([0.0, 0.0, -1.0], abs=1e-3)   # tool pointing down


def test_check_path(setup):
    checker = free(*setup)
    clear = checker.check_path(HOME, ABOVE_PART)
    assert clear.collision_at == -1.0 and clear.min_clearance > 0.0
    assert len(clear.tcp_path) > 10 and clear.end_joints == pytest.approx(ABOVE_PART)
    blocked = checker.check_path(HOME, INSIDE_CONVEYOR)
    assert 0.0 < blocked.collision_at <= 1.0 and blocked.collisions
    assert blocked.min_clearance == 0.0


def test_held_part_touches_its_support(setup):
    checker, poses = setup
    part = {'part_1': replace(checker.cell.parts[0], name='part_1')}
    free(checker, poses)
    tcp = checker.tcp_pose(ABOVE_PART)
    offset = motion.inverse(tcp) @ STATION
    checker.sync(poses, {'part_1': offset}, ['tcp', 'tool_link', 'wrist3_link'], part)
    checker.set_joints(ABOVE_PART)
    assert checker.touching('part_1') == ['conveyor']
    assert checker.in_collision()
    assert not checker.in_collision(allowed_pairs={('part_1', 'conveyor')})
    checker.set_joints(HOME)
    assert not checker.in_collision()
    assert np.isfinite(checker.clearance()[0])


def test_moving_objects_are_checked_at_every_sample(setup):
    checker, poses = setup
    part = {'part_1': replace(checker.cell.parts[0], name='part_1')}

    def sliding(z):
        """Slide a part along the belt direction under the TCP, at height z."""
        path = [motion.transform((-0.5 + 0.01 * i, -0.6, z)) for i in range(51)]
        checker.sync(dict(poses, part_1=path[0]), {}, [], part)
        return checker.check_motion([(i / 50, ABOVE_PART, {'part_1': pose})
                                     for i, pose in enumerate(path)])

    hit = sliding(1.0)                    # up to the finger carriages
    assert hit.collision_at > 0.6 and hit.collisions
    assert all(pair.endswith(' <-> part_1') for pair in hit.collisions)
    between = sliding(0.95)               # the open fingers let it pass between them
    assert between.collision_at == -1.0 and between.closest_object == 'part_1'
    passing = sliding(0.84)               # 5 cm below the finger tips
    assert passing.collision_at == -1.0 and passing.closest_object == 'part_1'
    assert 0.03 < passing.min_clearance < 0.07


def test_finger_contact_finds_both_sides_of_a_part(setup):
    checker, poses = setup
    part = {'part_1': replace(checker.cell.parts[0], name='part_1')}
    free(checker, poses)
    tcp = checker.tcp_pose(ABOVE_PART)
    y = tcp[:3, 1]                        # the fingers close along the tool's y axis
    for side in (0.0, 0.008):
        # the part with its grasp frame at the TCP, but its middle side metres towards +y
        pose = _turned_down(tcp, side * y)
        checker.sync(dict(poses, part_1=pose), {}, [], part)
        checker.set_joints(ABOVE_PART)
        contacts = checker.finger_contact('part_1', 0.055)
        # the part is 80 mm wide across the fingers
        assert contacts['finger_left'] == pytest.approx(0.04 + side, abs=1e-6)
        assert contacts['finger_right'] == pytest.approx(0.04 - side, abs=1e-6)
    assert checker.finger_contact('part_1', 0.03) == {'finger_left': 0.03, 'finger_right': 0.03}
    checker.set_joints(list(ABOVE_PART) + [0.04])       # a 7th value sets the fingers
    assert checker.link_pose('finger_left')[:3, 3] == pytest.approx(
        checker.link_pose('finger_right')[:3, 3] + 0.08 * y, abs=1e-9)


def _turned_down(tcp, shift):
    """Return the part pose whose grasp frame (3 cm up) is at the TCP, shifted by shift."""
    pose = np.eye(4)
    pose[:3, :3] = tcp[:3, :3] @ np.diag([1.0, -1.0, -1.0])     # part z up, x like the tool
    pose[:3, 3] = tcp[:3, 3] + shift - 0.03 * pose[:3, 2]
    return pose
