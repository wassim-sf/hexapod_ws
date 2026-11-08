"""Simulator logic with no GUI: turns slider values into a Frame, and a Frame into HTML panels."""
from __future__ import annotations

import html
from dataclasses import dataclass

import numpy as np

from hexapod_kinematics import (JOINT_ORDER, GaitSample, RobotModel, body_pose_ik, center_of_mass, check_pose, check_servo_speed,
                                feet_fk, rpy_matrix, stability_margin)
from hexapod_kinematics.body_ik import stability_issues

SUPPORT_TOLERANCE = 0.003  # m: without a ground model, feet this close to the lowest foot count as supporting


@dataclass
class Frame:
    q: np.ndarray               # (6, 3)
    position: np.ndarray        # base_link origin in the world
    rotation: np.ndarray        # base_link to world
    feet_world: np.ndarray      # (6, 3)
    contact: np.ndarray         # (6,)
    polygon: np.ndarray         # (n, 2)
    com_xy: np.ndarray
    margin: float
    issues: list
    notes: list


def joint_frame(model: RobotModel, q, start_q=None, move_time: float | None = None) -> Frame:
    """A pose given directly as joint angles. The body stays level with the lowest foot on the ground."""
    q = np.asarray(q, float)
    feet = feet_fk(model, q)
    height = -float(feet[:, 2].min())
    position = np.array([0.0, 0.0, height])
    feet_world = feet + position
    contact = feet_world[:, 2] < SUPPORT_TOLERANCE
    com = center_of_mass(model, q) + position
    stability = stability_margin(feet_world[contact, :2], com[:2])
    issues = check_pose(model, q) + stability_issues(stability)
    if start_q is not None and move_time is not None:
        issues += check_servo_speed(model, start_q, q, move_time)
    notes = ['Body held level; feet within 3 mm of the lowest foot count as on the ground.']
    return Frame(q, position, np.eye(3), feet_world, contact, stability.polygon, stability.com_xy, stability.margin, issues, notes)


def pattern_q(model: RobotModel, coxa: float, femur: float, tibia: float) -> np.ndarray:
    """The same angles (radians) on all legs."""
    return np.tile(np.array([coxa, femur, tibia], float), (len(model.legs), 1))


def body_frame(model: RobotModel, translation, rpy, start_q=None, move_time: float | None = None) -> Frame:
    result = body_pose_ik(model, translation, rpy)
    issues = list(result.issues)
    if start_q is not None and move_time is not None:
        issues += check_servo_speed(model, start_q, result.q, move_time)
    s = result.stability
    return Frame(result.q, result.position, result.rotation, result.feet_world, np.ones(len(model.legs), bool), s.polygon, s.com_xy,
                 s.margin, issues, ['Feet stay fixed at the standby stance.'])


def gait_frame(model: RobotModel, sample: GaitSample, max_speed: float, requested_speed: bool = False) -> Frame:
    c, s = np.cos(sample.body_yaw), np.sin(sample.body_yaw)
    notes = [f'Time {sample.t:.2f} s. Feet down: {int(sample.contact.sum())}. This gait allows at most {max_speed * 1000:.1f} mm/s in a straight line.']
    if sample.saturated:
        notes.append(f'Command limited by step length to vx {sample.command.vx * 1000:.1f}, vy {sample.command.vy * 1000:.1f} mm/s, '
                     f'yaw {sample.command.yaw_rate:.3f} rad/s.')
    st = sample.stability
    return Frame(sample.q, sample.body_position, rpy_matrix(0.0, 0.0, sample.body_yaw), sample.feet_world, sample.contact, st.polygon, st.com_xy,
                 st.margin, sample.issues, notes)


def nearest_point_on_polygon(polygon, point) -> np.ndarray:
    """Closest point on the boundary of a closed 2D polygon (or on a segment or a single point)."""
    polygon = np.asarray(polygon, float)
    point = np.asarray(point, float)
    best, best_distance = polygon[0], np.inf
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = b - a
        length = edge @ edge
        s = 0.0 if length <= 0.0 else float(np.clip((point - a) @ edge / length, 0.0, 1.0))
        candidate = a + s * edge
        distance = np.linalg.norm(point - candidate)
        if distance < best_distance:
            best, best_distance = candidate, distance
    return best


def table_html(model: RobotModel, q) -> str:
    """A table of joint angles in degrees. Out-of-limit values are red."""
    rows = []
    for leg in model.legs:
        cells = []
        for j, _ in enumerate(JOINT_ORDER):
            value = q[leg.index, j]
            bad = value < model.lower[j] - 1e-6 or value > model.upper[j] + 1e-6
            style = 'color:#c0141c;font-weight:700' if bad else ''
            cells.append(f'<td style="text-align:right;padding:1px 8px;{style}">{np.degrees(value):.1f}</td>')
        rows.append(f'<tr><td style="padding:1px 8px 1px 0">{leg.id} {html.escape(leg.name)}</td>{"".join(cells)}</tr>')
    head = ''.join(f'<th style="text-align:right;padding:1px 8px">{n}</th>' for n in JOINT_ORDER)
    return ('<div style="padding:4px 10px;font-size:0.8em" id="joint-table"><b>Joint angles (°)</b>'
            f'<table style="border-collapse:collapse;width:100%"><tr><th style="text-align:left">leg</th>{head}</tr>{"".join(rows)}</table></div>')


def status_html(frame: Frame, limit_issues: int = 8) -> str:
    """Stability readout, notes and red warnings."""
    colour = '#1a7f37' if frame.margin > 0 else '#c0141c'
    margin = '—' if not np.isfinite(frame.margin) else f'{frame.margin * 1000:.1f} mm'
    parts = [f'<div style="padding:4px 10px;font-size:0.85em" id="status"><b>Stability margin: <span style="color:{colour}">{margin}</span></b> '
             f'({len(frame.polygon)}-sided support polygon)']
    parts += [f'<div style="color:#555;margin-top:3px">{html.escape(n)}</div>' for n in frame.notes]
    if frame.issues:
        shown = frame.issues[:limit_issues]
        parts += [f'<div class="warning" style="color:#fff;background:#c0141c;border-radius:4px;padding:3px 7px;margin-top:4px">⚠ {html.escape(str(i))}</div>' for i in shown]
        if len(frame.issues) > len(shown):
            parts.append(f'<div style="color:#c0141c;margin-top:3px">and {len(frame.issues) - len(shown)} more</div>')
    else:
        parts.append('<div id="all-clear" style="color:#1a7f37;margin-top:4px">✓ No warnings</div>')
    parts.append('</div>')
    return ''.join(parts)
