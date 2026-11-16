import math

from fairino_sim import description
from fairino_workcell.kinematics import Arm, fit_limits, KinematicsError, pose_error
from fairino_workcell.motion import transform
import numpy as np
import pytest

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']


def _urdf(model, base_xyz=(0.1, -0.2, 0.75), base_rpy=(0.0, 0.0, 0.4)):
    tool = description.load_tool(description.resolve_tool_file('example_gripper'))
    package = description.moveit_config_package(model)
    controllers = f'{description.share_dir(package)}/config/ros2_controllers.yaml'
    return description.build_urdf(model, controllers, base_xyz=base_xyz, base_rpy=base_rpy,
                                  tool=tool)


def _limits(urdf):
    import xml.etree.ElementTree as ET
    joints = {j.get('name'): j for j in ET.fromstring(urdf).findall('joint')}
    lower = [float(joints[n].find('limit').get('lower')) for n in JOINTS]
    upper = [float(joints[n].find('limit').get('upper')) for n in JOINTS]
    return lower, upper


def _regular_samples(arm, rng, lower, upper, count):
    """Joint positions away from the singularities (where solutions are not isolated)."""
    samples = []
    while len(samples) < count:
        q = rng.uniform(lower, upper)
        wrist = (np.linalg.inv(arm.origins[0]) @ arm.fk(q) @ np.linalg.inv(arm.suffix))[:3, 3]
        shoulder = math.hypot(wrist[0], wrist[1]) ** 2 - arm.parameters[3] ** 2
        if abs(math.sin(q[2])) > 0.1 and abs(math.sin(q[4])) > 0.1 and shoulder > 0.05 ** 2:
            samples.append(q)
    return samples


@pytest.mark.parametrize('model', description.SUPPORTED_MODELS)
def test_round_trip_returns_the_reference_solution(model):
    urdf = _urdf(model)
    arm, (lower, upper) = Arm(urdf, JOINTS), _limits(urdf)
    rng = np.random.default_rng(7)
    for q in _regular_samples(arm, rng, lower, upper, 40):
        solutions, _ = arm.inverse(arm.fk(q), q, lower, upper)
        assert solutions, f'no solution for {np.degrees(q).round(1)}'
        assert np.max(np.abs(np.array(solutions[0]) - q)) < 1e-6
        for joints in solutions:
            distance, angle = pose_error(arm.fk(q), arm.fk(joints))
            assert distance < 1e-6 and angle < 1e-6


def test_a_regular_pose_has_eight_solutions():
    urdf = _urdf('fairino5_v6')
    arm = Arm(urdf, JOINTS)
    q = np.radians([10, -100, 70, -60, 40, 20])
    lots = [-10.0] * 6, [10.0] * 6     # no limits: every solution counts
    solutions, outside = arm.inverse(arm.fk(q), q, *lots)
    assert len(solutions) == 8 and outside == 0
    assert len({tuple(np.round(s, 6)) for s in solutions}) == 8


def test_nearest_solution_follows_the_reference():
    urdf = _urdf('fairino5_v6')
    arm, (lower, upper) = Arm(urdf, JOINTS), _limits(urdf)
    elbow_up = np.radians([0, -100, 80, -70, -90, 0])
    target = arm.fk(elbow_up)
    solutions, _ = arm.inverse(target, elbow_up, lower, upper)
    other = np.array(solutions[1])
    # starting from another solution keeps that configuration
    again, _ = arm.inverse(target, other + 0.05, lower, upper)
    assert np.max(np.abs(np.array(again[0]) - other)) < 1e-6
    # and the same reference always gives the same answer
    assert arm.inverse(target, elbow_up, lower, upper)[0][0] == solutions[0]


def test_wrist_singularity_keeps_j6_of_the_reference():
    urdf = _urdf('fairino5_v6')
    arm, (lower, upper) = Arm(urdf, JOINTS), _limits(urdf)
    q = np.radians([20, -110, 80, -40, 0, 30])     # j5 = 0: j4 and j6 are aligned
    solutions, _ = arm.inverse(arm.fk(q), q, lower, upper)
    # any j4 + j6 = const works here; the solver keeps j6 (to the refinement's few 1e-5 rad)
    assert np.max(np.abs(np.array(solutions[0]) - q)) < 1e-4


def test_unreachable_and_limited_targets():
    urdf = _urdf('fairino5_v6')
    arm, (lower, upper) = Arm(urdf, JOINTS), _limits(urdf)
    home = np.radians([0, -90, 90, -90, -90, 0])
    far = arm.fk(home) @ transform((0.0, 0.0, -2.0))
    assert arm.inverse(far, home, lower, upper) == ([], 0)
    narrow = list(lower), list(upper)
    narrow[0][2], narrow[1][2] = -0.1, 0.1      # j3 nearly locked: the elbow cannot bend
    solutions, outside = arm.inverse(arm.fk(home), home, *narrow)
    assert solutions == [] and outside > 0


def test_fit_limits_shifts_by_whole_turns():
    lower, upper = [-4.6, -3.0], [1.4, 3.0]
    fitted = fit_limits([3.0, 0.5], [-3.0, 0.0], lower, upper)
    assert fitted[0] == pytest.approx(3.0 - 2 * math.pi) and fitted[1] == pytest.approx(0.5)
    assert fit_limits([3.1], [0.0], [-3.0], [3.0]) is None


def test_other_layouts_are_rejected():
    urdf = _urdf('fairino5_v6').replace('<axis xyz="0 0 1"/>', '<axis xyz="0 1 0"/>', 1)
    with pytest.raises(KinematicsError):
        Arm(urdf, JOINTS)
    with pytest.raises(KinematicsError):
        Arm(_urdf('fairino5_v6'), ['j1', 'j2', 'j3', 'j4', 'j6', 'j5'])
