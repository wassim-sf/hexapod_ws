"""Validity checks that return readable reasons rather than bare booleans."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import IKSolution, link_frames
from .model import JOINT_ORDER, RobotModel

UNREACHABLE = 'unreachable'
JOINT_LIMIT = 'joint_limit'
LEG_COLLISION = 'leg_collision'
BODY_COLLISION = 'body_collision'
SERVO_SPEED = 'servo_speed'
TOO_FEW_FEET = 'too_few_feet'
UNSTABLE = 'unstable'


@dataclass(frozen=True)
class Issue:
    kind: str
    message: str
    leg: int | None = None        # leg id, 1 to 6
    joint: str | None = None
    other_leg: int | None = None
    value: float | None = None

    def __str__(self) -> str:
        return self.message


def check_reachable(model: RobotModel, solution: IKSolution) -> list:
    issues = []
    for leg in model.legs:
        if not solution.reachable[leg.index]:
            gap = float(solution.distance_from_workspace[leg.index])
            issues.append(Issue(UNREACHABLE, f'{leg.label}: foot is unreachable, {gap * 1000:.1f} mm outside the workspace',
                                leg=leg.id, value=gap))
    return issues


def check_joint_limits(model: RobotModel, q, tolerance: float = 1e-6) -> list:
    """q has shape (6, 3)."""
    issues = []
    q = np.asarray(q, float)
    for leg in model.legs:
        for j, name in enumerate(JOINT_ORDER):
            v, lo, hi = q[leg.index, j], model.lower[j], model.upper[j]
            if v < lo - tolerance or v > hi + tolerance:
                side, bound = ('below', lo) if v < lo else ('above', hi)
                issues.append(Issue(
                    JOINT_LIMIT,
                    f'{leg.label}: {name} {np.degrees(v):.1f}° is {side} the limit {np.degrees(bound):.1f}°',
                    leg=leg.id, joint=name, value=float(v)))
    return issues


def _segment_distance(p1, q1, p2, q2) -> float:
    """Shortest distance between two 3D segments (Ericson, Real-Time Collision Detection)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= 1e-12:
            s, t = np.clip(-c / a, 0.0, 1.0), 0.0
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm(p1 + d1 * s - (p2 + d2 * t)))


def leg_capsules(model: RobotModel, leg_index: int, q) -> list:
    """Capsules (name, start, end, radius) that bound one leg, from the leg_collision YAML section."""
    frames = link_frames(model, leg_index, q)
    c = model.leg_collision
    capsules = []

    def point(frame, xyz):
        return (frames[frame] @ np.array([*xyz, 1.0]))[:3]

    capsules.append(('femur', point('femur', (0.0, 0.0, 0.0)), point('femur', (model.femur, 0.0, 0.0)),
                     float(c['femur_radius'])))
    split = float(c['tibia_split'])
    capsules.append(('tibia_upper', point('tibia', (0.0, 0.0, 0.0)), point('tibia', (0.0, 0.0, -split)),
                     float(c['tibia_upper_radius'])))
    capsules.append(('tibia_lower', point('tibia', (0.0, 0.0, -split)), point('tibia', (0.0, 0.0, -model.tibia)),
                     float(c['tibia_lower_radius'])))
    return capsules


def leg_clearances(model: RobotModel, q) -> dict:
    """Smallest capsule gap in metres for each neighbouring pair (negative means overlap)."""
    q = np.asarray(q, float)
    capsules = {leg.index: leg_capsules(model, leg.index, q[leg.index]) for leg in model.legs}
    out = {}
    for i, j in model.neighbour_pairs:
        best = np.inf
        for _, a0, a1, ra in capsules[i]:
            for _, b0, b1, rb in capsules[j]:
                best = min(best, _segment_distance(a0, a1, b0, b1) - ra - rb)
        out[(i, j)] = best
    return out


def check_leg_collisions(model: RobotModel, q) -> list:
    issues = []
    for (i, j), gap in leg_clearances(model, q).items():
        if gap < 0.0:
            a, b = model.legs[i], model.legs[j]
            issues.append(Issue(LEG_COLLISION, f'{a.label} and {b.label} collide: legs overlap by {-gap * 1000:.1f} mm',
                                leg=a.id, other_leg=b.id, value=float(gap)))
    return issues


def body_capsules(model: RobotModel, leg_index: int, q) -> list:
    """Capsules (name, start, end, radius) for the parts of a leg that can swing under or into the body."""
    frames = link_frames(model, leg_index, q)
    c = model.body_collision
    split = float(c['tibia_split'])

    def point(xyz):
        return (frames['tibia'] @ np.array([*xyz, 1.0]))[:3]

    return [('tibia', point((0.0, 0.0, 0.0)), point((0.0, 0.0, -split)), float(c['tibia_radius'])),
            ('foot', point((0.0, 0.0, -split)), point((0.0, 0.0, -model.tibia)), float(c['foot_radius']))]


def _point_box_distance(points: np.ndarray, box) -> np.ndarray:
    """Distance from points (n, 3) to a box given as (centre, half extents, cos yaw, sin yaw)."""
    centre, half, cy, sy = box
    d = points - centre
    local = np.stack((cy * d[:, 0] + sy * d[:, 1], -sy * d[:, 0] + cy * d[:, 1], d[:, 2]), axis=1)
    return np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)


def body_clearances(model: RobotModel, q) -> dict:
    """Smallest gap in metres between each leg's tibia and foot capsules and the body boxes (negative means overlap)."""
    q = np.asarray(q, float)
    out = {}
    for leg in model.legs:
        best = np.inf
        for _, start, end, radius in body_capsules(model, leg.index, q[leg.index]):
            points = start + np.linspace(0.0, 1.0, 13)[:, None] * (end - start)
            for box in model.body_boxes:
                best = min(best, float(_point_box_distance(points, box).min()) - radius)
        out[leg.id] = best
    return out


def check_body_collisions(model: RobotModel, q) -> list:
    """A tibia or foot swinging into or under the body. Coxa and femur contact is prevented by the joint limits."""
    issues = []
    for leg_id, gap in body_clearances(model, q).items():
        if gap < 0.0:
            leg = model.legs[leg_id - 1]
            issues.append(Issue(BODY_COLLISION, f'{leg.label}: tibia or foot hits the body, overlap {-gap * 1000:.1f} mm', leg=leg.id, value=float(gap)))
    return issues


def check_servo_speed(model: RobotModel, q_from, q_to, dt: float) -> list:
    """Compare the speed needed to move between two poses in dt with the YAML joint velocity."""
    if dt <= 0.0:
        raise ValueError('dt must be positive')
    issues = []
    speed = np.abs(np.asarray(q_to, float) - np.asarray(q_from, float)) / dt
    for leg in model.legs:
        for j, name in enumerate(JOINT_ORDER):
            if speed[leg.index, j] > model.velocity[j]:
                issues.append(Issue(
                    SERVO_SPEED,
                    f'{leg.label}: {name} needs {speed[leg.index, j]:.1f} rad/s, servo limit is {model.velocity[j]:.1f} rad/s',
                    leg=leg.id, joint=name, value=float(speed[leg.index, j])))
    return issues


def check_pose(model: RobotModel, q, solution: IKSolution | None = None, previous_q=None, dt: float | None = None) -> list:
    """All pose checks. Pass the IK solution when q came from IK, and previous_q with dt for speed."""
    issues = []
    if solution is not None:
        issues += check_reachable(model, solution)
    issues += check_joint_limits(model, q)
    issues += check_leg_collisions(model, q)
    issues += check_body_collisions(model, q)
    if previous_q is not None and dt is not None:
        issues += check_servo_speed(model, previous_q, q, dt)
    return issues
