"""Static stability: centre of mass, support polygon and stability margin."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import link_frames
from .model import LEG_LINKS, RobotModel


def center_of_mass(model: RobotModel, q) -> np.ndarray:
    """Centre of mass in base_link from the estimated YAML masses. Left legs mirror y of each link's com."""
    q = np.asarray(q, float)
    total = model.base_inertial.mass * model.base_inertial.com
    for leg in model.legs:
        frames = link_frames(model, leg.index, q[leg.index])
        for name in LEG_LINKS:
            link = model.link_inertials[name]
            com = link.com * np.array([1.0, leg.mirror, 1.0])
            total = total + link.mass * (frames[name][:3, :3] @ com + frames[name][:3, 3])
    return total / model.total_mass


def convex_hull(points) -> np.ndarray:
    """Counter-clockwise convex hull of 2D points (monotone chain). Collinear points are dropped."""
    pts = sorted({(float(x), float(y)) for x, y in np.asarray(points, float).reshape(-1, 2)})
    if len(pts) <= 2:
        return np.array(pts, float).reshape(-1, 2)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def half(sequence):
        out = []
        for p in sequence:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 1e-15:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(reversed(pts)), float)


def _distance_to_segment(p, a, b) -> float:
    ab = b - a
    denom = ab @ ab
    s = 0.0 if denom <= 0.0 else np.clip((p - a) @ ab / denom, 0.0, 1.0)
    return float(np.linalg.norm(p - (a + s * ab)))


@dataclass
class Stability:
    polygon: np.ndarray    # (n, 2) counter-clockwise
    com_xy: np.ndarray
    margin: float          # metres to the nearest polygon edge; negative if the com is outside
    n_feet: int

    @property
    def stable(self) -> bool:
        return self.margin > 0.0


def stability_margin(feet_xy, com_xy) -> Stability:
    """Signed distance from the com projection to the support polygon boundary.

    With fewer than 3 non-collinear feet the polygon has no area, so the margin is minus the distance
    to the feet (and never positive).
    """
    feet_xy = np.asarray(feet_xy, float).reshape(-1, 2)
    com_xy = np.asarray(com_xy, float)
    hull = convex_hull(feet_xy)
    if len(hull) == 0:
        return Stability(hull, com_xy, -np.inf, 0)
    if len(hull) == 1:
        return Stability(hull, com_xy, -float(np.linalg.norm(com_xy - hull[0])), len(feet_xy))
    if len(hull) == 2:
        return Stability(hull, com_xy, -_distance_to_segment(com_xy, hull[0], hull[1]), len(feet_xy))
    n = len(hull)
    inside = True
    distance = np.inf
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        edge = b - a
        if edge[0] * (com_xy[1] - a[1]) - edge[1] * (com_xy[0] - a[0]) < 0.0:
            inside = False
        distance = min(distance, _distance_to_segment(com_xy, a, b))
    return Stability(hull, com_xy, distance if inside else -distance, len(feet_xy))
