"""
Kinematics of the FAIRINO arms: forward kinematics from the URDF and analytic inverse kinematics.

The FAIRINO cobots have the joint layout of the Universal Robots arms: j2, j3 and j4 are
parallel and the wrist joints j4, j5 and j6 are perpendicular in turn. A TCP pose then has up
to eight joint solutions (shoulder left or right, elbow up or down, wrist flipped or not).
Like the robot controller does for Cartesian targets, the solver takes the solution nearest to
a reference joint position, normally the current one, so a pose target gives the same joints
every time: in a check run and in the run that follows it.
"""

import math
import xml.etree.ElementTree as ET

from fairino_workcell.motion import inverse, transform
import numpy as np

HALF_PI = math.pi / 2
NEAR_SINGULAR = 1e-3   # |sin(j5)| below this: j4 and j6 turn about almost the same axis
LAYOUT_TOLERANCE = 1e-4  # the URDF rounds pi/2, so the closed form is refined afterwards
POSITION_TOLERANCE = 1e-6
ANGLE_TOLERANCE = 1e-6


class KinematicsError(ValueError):
    """The robot description does not fit the kinematics solver."""


def _floats(element, attribute, default):
    text = element.get(attribute) if element is not None else None
    return tuple(float(v) for v in text.split()) if text else default


def _rotation(axis, angle):
    """Return the 4x4 rotation by angle about a unit axis."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    matrix = np.eye(4)
    matrix[:3, :3] = [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                      [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                      [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]
    return matrix


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    matrix = np.eye(4)
    matrix[0, 0], matrix[0, 1], matrix[1, 0], matrix[1, 1] = c, -s, s, c
    return matrix


def _clip(value):
    return max(-1.0, min(1.0, value))


def pose_error(target, current):
    """Return the position distance (m) and the rotation angle (rad) between two poses."""
    rotation = target[:3, :3] @ current[:3, :3].T
    angle = math.acos(_clip((np.trace(rotation) - 1.0) / 2.0))
    return float(np.linalg.norm(target[:3, 3] - current[:3, 3])), angle


def _twist_error(target, current):
    """Return the 6-vector error for Newton steps (valid for small rotations)."""
    rotation = target[:3, :3] @ current[:3, :3].T
    spin = 0.5 * np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                           rotation[1, 0] - rotation[0, 1]])
    return np.concatenate([target[:3, 3] - current[:3, 3], spin])


def fit_limits(joints, reference, lower, upper):
    """
    Shift each angle by whole turns into its limits, nearest to the reference.

    Return None when an angle fits nowhere in its limits.
    """
    result = []
    for value, ref, low, high in zip(joints, reference, lower, upper):
        candidates = [value + 2.0 * math.pi * k for k in range(-2, 3)]
        candidates = [v for v in candidates if low - 1e-9 <= v <= high + 1e-9]
        if not candidates:
            return None
        result.append(min(candidates, key=lambda v: abs(v - ref)))
    return np.array(result)


class Arm:
    """Kinematics of six revolute joints on the chain from the URDF root link to a tip link."""

    def __init__(self, urdf, joints, tip='tcp'):
        root = ET.fromstring(urdf) if isinstance(urdf, str) else urdf
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip
        while link in by_child:
            chain.append(by_child[link])
            link = by_child[link].find('parent').get('link')
        chain.reverse()
        self.root, self.tip, self.joints = link, tip, list(joints)
        names = [j.get('name') for j in chain]
        if [n for n in names if n in self.joints] != self.joints:
            raise KinematicsError(f'joints {", ".join(self.joints)} are not on the chain from '
                                  f'{link} to {tip}, in this order')
        self.origins, self.axes = [], []    # fixed transform before each joint, joint axis
        fixed = np.eye(4)
        for joint in chain:
            origin = joint.find('origin')
            fixed = fixed @ transform(_floats(origin, 'xyz', (0.0, 0.0, 0.0)),
                                      _floats(origin, 'rpy', (0.0, 0.0, 0.0)))
            if joint.get('name') in self.joints:
                if joint.get('type') not in ('revolute', 'continuous'):
                    raise KinematicsError(f"joint {joint.get('name')} is not revolute")
                axis = np.array(_floats(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0)))
                self.axes.append(axis / np.linalg.norm(axis))
                self.origins.append(fixed)
                fixed = np.eye(4)
            elif joint.get('type') != 'fixed':
                raise KinematicsError(f"joint {joint.get('name')} between the arm joints moves")
        self.suffix = fixed                 # last joint -> tip
        self.parameters = self._ur_parameters()
        if self.parameters is None:
            raise KinematicsError('the arm does not have the joint layout of the FAIRINO cobots')

    def _ur_parameters(self):
        """Return (d1, a2, a3, d4, d5) when the joints have the UR layout, else None."""
        if len(self.axes) != 6 or not all(np.allclose(a, (0, 0, 1)) for a in self.axes):
            return None
        o = self.origins
        d1, a2, a3, d4, d5 = o[1][2, 3], o[2][0, 3], o[3][0, 3], o[4][2, 3], o[5][2, 3]
        layout = [transform((0, 0, d1), (HALF_PI, 0, 0)), transform((a2, 0, 0)),
                  transform((a3, 0, 0)), transform((0, 0, d4), (HALF_PI, 0, 0)),
                  transform((0, 0, d5), (-HALF_PI, 0, 0))]
        if not all(np.allclose(actual, ideal, atol=1e-4) for actual, ideal in zip(o[1:], layout)):
            return None
        if abs(a2) < 1e-6 or abs(a3) < 1e-6:
            return None
        return float(d1), float(a2), float(a3), float(d4), float(d5)

    # ---------------------------------------------------------------- forward
    def frames(self, joints):
        """Return each joint frame, before the joint turns, and the tip pose in the root frame."""
        matrix, frames = np.eye(4), []
        for origin, axis, angle in zip(self.origins, self.axes, joints):
            matrix = matrix @ origin
            frames.append(matrix)
            matrix = matrix @ _rotation(axis, angle)
        return frames, matrix @ self.suffix

    def fk(self, joints):
        """Return the 4x4 pose of the tip in the root frame."""
        return self.frames(joints)[1]

    def jacobian(self, joints):
        """Return the 6x6 geometric Jacobian (linear rows first) and the tip pose."""
        frames, tip = self.frames(joints)
        result = np.zeros((6, len(frames)))
        for i, (frame, axis) in enumerate(zip(frames, self.axes)):
            z = frame[:3, :3] @ axis
            result[:3, i] = np.cross(z, tip[:3, 3] - frame[:3, 3])
            result[3:, i] = z
        return result, tip

    # ---------------------------------------------------------------- inverse
    def inverse(self, target, reference, lower, upper):
        """
        Solve a 4x4 tip pose given in the root frame.

        Return the solutions within the joint limits, nearest to the reference first, and
        the number of solutions that only exist outside the limits.
        """
        reference = np.asarray(reference, dtype=float)
        inside, outside, seen = [], 0, []
        for joints in self._analytic(target, reference):
            joints = self._refine(joints, target)
            distance, angle = pose_error(target, self.fk(joints))
            if distance > POSITION_TOLERANCE or angle > ANGLE_TOLERANCE:
                continue
            joints = np.array([math.remainder(v, 2.0 * math.pi) for v in joints])
            if any(np.max(np.abs(joints - other)) < 1e-6 for other in seen):
                continue
            seen.append(joints)
            fitted = fit_limits(joints, reference, lower, upper)
            if fitted is None:
                outside += 1
            else:
                inside.append(fitted)
        inside.sort(key=lambda joints: float(np.sum((joints - reference) ** 2)))
        return [tuple(float(v) for v in joints) for joints in inside], outside

    def _analytic(self, target, reference):
        """Return the closed-form solutions of the UR layout (up to eight)."""
        d4 = self.parameters[3]
        # wrist3 frame (on the j6 axis, at the wrist centre) in the frame of j1
        t = inverse(self.origins[0]) @ target @ inverse(self.suffix)
        p, x6, y6, z6 = t[:3, 3], t[:3, 0], t[:3, 1], t[:3, 2]
        radius = math.hypot(p[0], p[1])
        if radius < abs(d4) - LAYOUT_TOLERANCE:
            return []
        psi, phi = math.atan2(p[1], p[0]), math.asin(_clip(d4 / max(radius, 1e-12)))
        solutions = []
        for q1 in (psi + phi, psi + math.pi - phi):
            s1, c1 = math.sin(q1), math.cos(q1)
            # the j2 axis in the wrist3 frame is (sin q5 cos q6, -sin q5 sin q6, cos q5)
            c5 = _clip(z6[0] * s1 - z6[1] * c1)
            for q5 in (math.acos(c5), -math.acos(c5)):
                s5 = math.sin(q5)
                wrist = []
                if abs(s5) > 1e-12:
                    wrist.append(math.atan2(-(y6[0] * s1 - y6[1] * c1) / s5,
                                            (x6[0] * s1 - x6[1] * c1) / s5))
                if abs(s5) < NEAR_SINGULAR:    # j4 + j6 is what counts: keep j6 if possible
                    wrist.append(float(reference[5]))
                for q6 in wrist:
                    solutions.extend(self._planar(t, q1, q5, q6))
        return solutions

    def _planar(self, t, q1, q5, q6):
        """Return the solutions of j2, j3 and j4, a planar arm in the frame after j1."""
        _, a2, a3, _, _ = self.parameters
        planar = (inverse(_rz(q1) @ self.origins[1]) @ t @ inverse(_rz(q6))
                  @ inverse(_rz(q5) @ self.origins[5]))
        px, py = planar[0, 3], planar[1, 3]
        c3 = (px * px + py * py - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
        if abs(c3) > 1.0 + LAYOUT_TOLERANCE:
            return []
        q234 = math.atan2(planar[1, 0], planar[0, 0])
        result = []
        for q3 in (math.acos(_clip(c3)), -math.acos(_clip(c3))):
            q2 = math.atan2(py, px) - math.atan2(a3 * math.sin(q3), a2 + a3 * math.cos(q3))
            result.append(np.array((q1, q2, q3, q234 - q2 - q3, q5, q6)))
        return result

    def follow(self, target, previous, iterations=20):
        """
        Return the joints for a 4x4 tip pose (root frame) close to previous, or None.

        Newton steps from previous keep the arm configuration, so a Cartesian path followed
        in small steps stays on one branch of the solutions.
        """
        joints = np.array(previous, dtype=float)
        for _ in range(iterations):
            jacobian, tip = self.jacobian(joints)
            error = _twist_error(target, tip)
            if np.max(np.abs(error)) < 1e-12:
                break
            step = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            largest = float(np.max(np.abs(step)))
            if largest > 0.5:                   # near a singularity: do not jump far
                step *= 0.5 / largest
            joints = joints + step
        distance, angle = pose_error(target, self.fk(joints))
        if distance > POSITION_TOLERANCE or angle > ANGLE_TOLERANCE:
            return None
        return joints

    def singularity(self, joints):
        """
        Return how far the joints are from the singularities of the UR layout.

        (wrist, elbow, shoulder): |sin j5| and |sin j3| (0 at the wrist and elbow
        singularities), and the distance in metres of the wrist centre from the cylinder
        around j1 where the two shoulder solutions meet.
        """
        frames, _ = self.frames(joints)
        centre = inverse(self.origins[0]) @ frames[5][:, 3]
        d4 = self.parameters[3]
        shoulder = math.sqrt(max(centre[0] ** 2 + centre[1] ** 2 - d4 ** 2, 0.0))
        return abs(math.sin(joints[4])), abs(math.sin(joints[2])), shoulder

    def _refine(self, joints, target, iterations=10):
        """Newton steps on the URDF kinematics (the layout is only ideal up to rounding)."""
        joints = np.array(joints, dtype=float)
        for _ in range(iterations):
            jacobian, tip = self.jacobian(joints)
            error = _twist_error(target, tip)
            if np.max(np.abs(error)) < 1e-13:
                break
            joints = joints + np.linalg.lstsq(jacobian, error, rcond=None)[0]
        return joints
