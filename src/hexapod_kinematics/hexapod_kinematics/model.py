"""Robot model. Every number comes from hexapod_description/config/robot_parameters.yaml."""
from __future__ import annotations

import os
import pathlib
from dataclasses import dataclass, field

import numpy as np
import yaml

ENV_VAR = 'HEXAPOD_PARAMS'
_RELATIVE = pathlib.Path('share/hexapod_description/config/robot_parameters.yaml')


def find_parameters_file() -> pathlib.Path:
    """Locate robot_parameters.yaml without any ROS import.

    Search order: $HEXAPOD_PARAMS, the source tree next to this package, then $AMENT_PREFIX_PATH.
    """
    candidates = []
    if os.environ.get(ENV_VAR):
        candidates.append(pathlib.Path(os.environ[ENV_VAR]))
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / 'hexapod_description/config/robot_parameters.yaml')
        candidates.append(parent / 'src/hexapod_description/config/robot_parameters.yaml')
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(pathlib.Path(prefix) / _RELATIVE)
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        f'robot_parameters.yaml not found. Pass its path to RobotModel.load() or set ${ENV_VAR}.')


@dataclass(frozen=True)
class Joint:
    name: str
    axis: tuple
    lower: float
    upper: float
    effort: float
    velocity: float


@dataclass(frozen=True)
class Leg:
    index: int
    id: int
    name: str
    side: str
    mirror: int
    yaw: float
    hip: np.ndarray = field(repr=False)

    @property
    def label(self) -> str:
        return f'leg_{self.id} ({self.name})'


@dataclass(frozen=True)
class LinkInertial:
    mass: float
    com: np.ndarray


JOINT_ORDER = ('coxa', 'femur', 'tibia')
LEG_LINKS = ('coxa', 'femur', 'tibia', 'foot')


class RobotModel:
    """Geometry, limits, poses and mass properties. SI units, radians."""

    def __init__(self, data: dict, path: pathlib.Path | None = None):
        self.params_path = path
        self.raw = data
        geo = data['leg_geometry']
        self.coxa = float(geo['coxa_length'])
        self.femur = float(geo['femur_length'])
        self.tibia = float(geo['tibia_length'])
        self.hip_z = float(data['body']['hip_z'])
        self.legs = tuple(
            Leg(i, int(leg['id']), leg['name'], leg['side'], int(leg['mirror']), float(leg['yaw']),
                np.array([leg['x'], leg['y'], self.hip_z], float))
            for i, leg in enumerate(data['legs']))
        self.joints = {
            name: Joint(name, tuple(data['joints'][name]['axis']), float(data['joints'][name]['lower']),
                        float(data['joints'][name]['upper']), float(data['joints'][name]['effort']),
                        float(data['joints'][name]['velocity']))
            for name in JOINT_ORDER}
        self.lower = np.array([self.joints[n].lower for n in JOINT_ORDER])
        self.upper = np.array([self.joints[n].upper for n in JOINT_ORDER])
        self.velocity = np.array([self.joints[n].velocity for n in JOINT_ORDER])
        self.effort = np.array([self.joints[n].effort for n in JOINT_ORDER])
        self.poses = {k: np.array(v, float) for k, v in data['poses'].items()}
        self.servo = dict(data['servo'])
        links = data['links']
        self.base_inertial = LinkInertial(float(links['base']['mass']), np.array(links['base']['com'], float))
        self.link_inertials = {
            n: LinkInertial(float(links[n]['mass']), np.array(links[n]['com'], float)) for n in LEG_LINKS}
        self.total_mass = self.base_inertial.mass + len(self.legs) * sum(i.mass for i in self.link_inertials.values())
        self.leg_collision = dict(data['leg_collision'])
        self.hips = np.array([leg.hip for leg in self.legs])
        self.yaws = np.array([leg.yaw for leg in self.legs])
        # Neighbours follow the ring of leg yaws, so pairs are never listed by hand.
        order = sorted(self.legs, key=lambda leg: leg.yaw)
        self.neighbour_pairs = tuple(
            tuple(sorted((order[i].index, order[(i + 1) % len(order)].index))) for i in range(len(order)))

    @classmethod
    def load(cls, path: str | os.PathLike | None = None) -> 'RobotModel':
        resolved = pathlib.Path(path) if path else find_parameters_file()
        with open(resolved, encoding='utf-8') as stream:
            return cls(yaml.safe_load(stream), resolved)

    def leg_by_name(self, name: str) -> Leg:
        for leg in self.legs:
            if leg.name == name:
                return leg
        raise KeyError(name)

    def pose_q(self, pose: str) -> np.ndarray:
        """The named pose as an array (6, 3): the same joint angles on every leg."""
        return np.tile(self.poses[pose], (len(self.legs), 1))

    @property
    def mesh_dir(self) -> pathlib.Path:
        if self.params_path is None:
            raise ValueError('model was not loaded from a file')
        subdir = self.raw.get('visual', {}).get('mesh_subdir', '')
        return pathlib.Path(self.params_path).resolve().parent.parent / 'meshes' / subdir
