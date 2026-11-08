"""Body pose IK: move the body while the feet stay fixed on the ground."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .checks import TOO_FEW_FEET, UNSTABLE, Issue, check_pose
from .kinematics import feet_fk, feet_ik, rpy_matrix
from .model import RobotModel
from .stability import Stability, center_of_mass, stability_margin


def nominal_stance(model: RobotModel, pose: str = 'standby') -> tuple:
    """Feet on the ground (z = 0) at a named pose, and the body height that gives. Returns (feet_world, height)."""
    feet = feet_fk(model, model.pose_q(pose))
    height = -float(feet[:, 2].mean())
    world = feet.copy()
    world[:, 2] = 0.0
    return world, height


@dataclass
class BodyPoseResult:
    q: np.ndarray               # (6, 3)
    feet_world: np.ndarray      # (6, 3)
    feet_body: np.ndarray       # (6, 3)
    position: np.ndarray        # base_link origin in the world
    rotation: np.ndarray        # base_link to world, 3x3
    stability: Stability
    issues: list


def body_pose_ik(model: RobotModel, translation=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), feet_world=None,
                 stance: str = 'standby') -> BodyPoseResult:
    """Joint angles for a body pose, with feet fixed in the world.

    translation is the body offset from its nominal pose (centred above the stance at the stance height).
    rpy is roll, pitch, yaw about fixed axes. feet_world defaults to the named stance on z = 0.
    """
    nominal_feet, height = nominal_stance(model, stance)
    feet = nominal_feet if feet_world is None else np.asarray(feet_world, float)
    rotation = rpy_matrix(*rpy)
    position = np.array([0.0, 0.0, height]) + np.asarray(translation, float)
    feet_body = (feet - position) @ rotation
    solution = feet_ik(model, feet_body)
    issues = check_pose(model, solution.q, solution=solution)
    com_body = center_of_mass(model, solution.q)
    com_world = rotation @ com_body + position
    stability = stability_margin(feet[:, :2], com_world[:2])
    issues += stability_issues(stability)
    return BodyPoseResult(solution.q, feet.copy(), feet_body, position, rotation, stability, issues)


def stability_issues(stability: Stability) -> list:
    issues = []
    if stability.n_feet < 3:
        issues.append(Issue(TOO_FEW_FEET, f'only {stability.n_feet} feet on the ground', value=float(stability.n_feet)))
    if not stability.stable:
        issues.append(Issue(UNSTABLE, f'statically unstable: centre of mass is {-stability.margin * 1000:.1f} mm outside the support polygon',
                            value=float(stability.margin)))
    return issues
