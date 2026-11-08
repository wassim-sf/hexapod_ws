"""Write a gait trajectory as CSV. SI units, radians, world-frame foot positions."""
from __future__ import annotations

import csv
import os

from .gait import GaitTrajectory
from .model import JOINT_ORDER, RobotModel


def csv_header(model: RobotModel) -> list:
    header = ['time']
    header += [f'leg_{leg.id}_{joint}' for leg in model.legs for joint in JOINT_ORDER]
    header += [f'leg_{leg.id}_foot_{axis}' for leg in model.legs for axis in 'xyz']
    header += [f'leg_{leg.id}_contact' for leg in model.legs]
    header += ['body_x', 'body_y', 'body_z', 'body_yaw', 'stability_margin']
    return header


def write_csv(model: RobotModel, trajectory: GaitTrajectory, path: str | os.PathLike) -> None:
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(csv_header(model))
        for k, t in enumerate(trajectory.t):
            row = [f'{t:.4f}']
            row += [f'{v:.6f}' for v in trajectory.q[k].ravel()]
            row += [f'{v:.6f}' for v in trajectory.feet_world[k].ravel()]
            row += [str(int(v)) for v in trajectory.contact[k]]
            row += [f'{v:.6f}' for v in trajectory.body_position[k]]
            row += [f'{trajectory.body_yaw[k]:.6f}', f'{trajectory.margin[k]:.6f}']
            writer.writerow(row)
