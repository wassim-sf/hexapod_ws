"""Leg forward and inverse kinematics and frame transforms.

Leg frame: origin at the hip (coxa axis, at the coxa-femur plane), x outward at q = 0, z up.
Radial reach and height for a leg, from the step 1 convention:
    radial = c + f cos(q2) + t sin(q2 + q3)
    up     =     f sin(q2) - t cos(q2 + q3)
Functions accept any leading batch shape, so the RL environment can vectorise them.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import RobotModel


def leg_fk(model: RobotModel, q) -> np.ndarray:
    """Foot position in the leg frame. q has shape (..., 3): coxa, femur, tibia."""
    q = np.asarray(q, float)
    q1, q2, q3 = q[..., 0], q[..., 1], q[..., 2]
    radial = model.coxa + model.femur * np.cos(q2) + model.tibia * np.sin(q2 + q3)
    up = model.femur * np.sin(q2) - model.tibia * np.cos(q2 + q3)
    return np.stack((radial * np.cos(q1), radial * np.sin(q1), up), axis=-1)


@dataclass
class IKSolution:
    """q has shape (..., 3). Where reachable is False, q is the closest pose along the same direction."""
    q: np.ndarray
    reachable: np.ndarray
    distance_from_workspace: np.ndarray


def leg_ik(model: RobotModel, p_leg) -> IKSolution:
    """Closed-form IK for a foot position in the leg frame. See docs/kinematics.md."""
    p = np.asarray(p_leg, float)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    # A foot behind the coxa axis is reached with negative radial, keeping |q1| <= 90 deg.
    sign = np.where(x < 0.0, -1.0, 1.0)
    q1 = np.arctan2(sign * y, sign * x)
    radial = sign * np.hypot(x, y)
    a, b = radial - model.coxa, z
    distance = np.hypot(a, b)
    f, t = model.femur, model.tibia
    near, far = abs(f - t), f + t
    outside = np.maximum(distance - far, near - distance)
    reachable = outside <= 1e-9
    # Elbow angle between femur and tibia. Negative branch: tibia hangs below the femur.
    clamped = np.clip(distance, near, far)
    cos_elbow = np.clip((clamped ** 2 - f ** 2 - t ** 2) / (2.0 * f * t), -1.0, 1.0)
    elbow = -np.arccos(cos_elbow)
    q3 = elbow + np.pi / 2.0
    q2 = np.arctan2(b, a) - np.arctan2(t * np.sin(elbow), f + t * np.cos(elbow))
    q2 = (q2 + np.pi) % (2.0 * np.pi) - np.pi
    return IKSolution(np.stack((q1, q2, q3), axis=-1), reachable, np.maximum(outside, 0.0))


def _rz(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def body_to_leg(model: RobotModel, leg_index: int, p_body) -> np.ndarray:
    """Point in base_link to the given leg's frame."""
    leg = model.legs[leg_index]
    return (np.asarray(p_body, float) - leg.hip) @ _rz(leg.yaw)


def leg_to_body(model: RobotModel, leg_index: int, p_leg) -> np.ndarray:
    """Point in the given leg's frame to base_link."""
    leg = model.legs[leg_index]
    return np.asarray(p_leg, float) @ _rz(leg.yaw).T + leg.hip


def feet_fk(model: RobotModel, q) -> np.ndarray:
    """All foot positions in base_link. q (..., 6, 3) -> (..., 6, 3)."""
    p = leg_fk(model, q)
    c, s = np.cos(model.yaws), np.sin(model.yaws)
    x = c * p[..., 0] - s * p[..., 1] + model.hips[:, 0]
    y = s * p[..., 0] + c * p[..., 1] + model.hips[:, 1]
    return np.stack((x, y, p[..., 2] + model.hips[:, 2]), axis=-1)


def feet_ik(model: RobotModel, feet_body) -> IKSolution:
    """All joint angles for foot positions in base_link. feet_body (..., 6, 3)."""
    d = np.asarray(feet_body, float) - model.hips
    c, s = np.cos(model.yaws), np.sin(model.yaws)
    leg_frame = np.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]), axis=-1)
    return leg_ik(model, leg_frame)


def _transform(rotation: np.ndarray, translation) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def _about_minus_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


def link_frames(model: RobotModel, leg_index: int, q) -> dict:
    """URDF link frames in base_link for one leg (4x4): coxa, femur, tibia, foot."""
    leg = model.legs[leg_index]
    q = np.asarray(q, float)
    coxa = _transform(_rz(leg.yaw + q[0]), leg.hip)
    femur = coxa @ _transform(_about_minus_y(q[1]), (model.coxa, 0.0, 0.0))
    tibia = femur @ _transform(_about_minus_y(q[2]), (model.femur, 0.0, 0.0))
    foot = tibia @ _transform(np.eye(3), (0.0, 0.0, -model.tibia))
    return {'coxa': coxa, 'femur': femur, 'tibia': tibia, 'foot': foot}


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return _rz(yaw) @ ry @ rx
