"""Tests of the moveit_py path checker with the example cell (needs moveit_py)."""

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


@pytest.fixture(scope='module')
def setup():
    cell = load_cell(EXAMPLE)
    tool = description.load_tool(cell.robot.tool_file)
    package = description.moveit_config_package(cell.robot.model)
    controllers = os.path.join(description.share_dir(package), 'config', 'ros2_controllers.yaml')
    urdf = description.build_urdf(cell.robot.model, controllers, base_xyz=cell.robot.base.xyz,
                                  base_rpy=cell.robot.base.rpy, tool=tool)
    checker = checker_module.Checker(cell, urdf, description.build_srdf(cell.robot.model, tool),
                                     f'{cell.robot.model}_group', ['tool_link'])
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
    assert 'tool_link <-> conveyor' in pairs
    assert checker.collisions(allowed_pairs={('tool_link', 'conveyor'), ('tool_link', 'part'),
                                             ('forearm_link', 'conveyor'),
                                             ('upperarm_link', 'conveyor'),
                                             ('forearm_link', 'part')}) == []


def test_tcp_pose_above_the_part(setup):
    checker, poses = setup
    free(checker, poses)
    tcp = checker.tcp_pose(ABOVE_PART)
    assert tcp[:3, 3] == pytest.approx([0.0, -0.6, 0.98], abs=2e-3)
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
    free(checker, poses)
    tcp = checker.tcp_pose(ABOVE_PART)
    offset = motion.inverse(tcp) @ poses['part']
    others = {name: pose for name, pose in poses.items() if name != 'part'}
    checker.sync(others, {'part': offset}, ['tcp', 'tool_link', 'wrist3_link'])
    checker.set_joints(ABOVE_PART)
    assert checker.touching('part') == ['conveyor']
    assert checker.in_collision()
    assert not checker.in_collision(allowed_pairs={('part', 'conveyor')})
    checker.set_joints(HOME)
    assert not checker.in_collision()
    assert np.isfinite(checker.clearance()[0])
