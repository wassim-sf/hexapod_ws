"""Tripod, ripple and wave gaits driven by a body velocity command.

GaitGenerator is stateful and advances with update(dt, command), so the command may change at any time
(as with /cmd_vel). generate_gait() runs it to make a complete checked trajectory.

Stance feet are fixed in the world: in the body frame they follow the exact inverse of the body motion.
Swing feet travel from lift-off to a touchdown point chosen so that the foot passes under its home
position halfway through the next stance.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .body_ik import nominal_stance, stability_issues
from .checks import Issue, check_pose, check_servo_speed
from .kinematics import feet_ik
from .model import JOINT_ORDER, RobotModel
from .stability import Stability, center_of_mass, stability_margin

# Phase offset by leg name: the fraction of a cycle at which each leg begins its stance.
# Swing begins when the phase reaches duty_factor.
PHASES = {
    'tripod': {'front_right': 0.0, 'rear_right': 0.0, 'center_left': 0.0,
               'center_right': 0.5, 'front_left': 0.5, 'rear_left': 0.5},
    # Each side steps rear to front, with the two sides half a cycle apart.
    'ripple': {'rear_right': 0.0, 'center_right': 1 / 3, 'front_right': 2 / 3,
               'rear_left': 1 / 2, 'center_left': 5 / 6, 'front_left': 1 / 6},
    # One leg at a time: right side rear to front, then left side rear to front.
    'wave': {'rear_right': 0.0, 'center_right': 1 / 6, 'front_right': 2 / 6,
             'rear_left': 3 / 6, 'center_left': 4 / 6, 'front_left': 5 / 6},
}
# Slow, low defaults: holding a stance already uses about 54% of femur stall torque.
DEFAULTS = {
    'tripod': {'cycle_period': 2.0, 'duty_factor': 0.55},
    'ripple': {'cycle_period': 3.0, 'duty_factor': 0.70},
    'wave': {'cycle_period': 6.0, 'duty_factor': 0.85},
}
DEFAULT_STEP_LENGTH = 0.030
DEFAULT_STEP_HEIGHT = 0.015
_IDLE = 1e-6


@dataclass(frozen=True)
class GaitParams:
    gait: str = 'tripod'
    step_length: float = DEFAULT_STEP_LENGTH    # largest foot travel during one stance, metres
    step_height: float = DEFAULT_STEP_HEIGHT    # swing foot clearance, metres
    cycle_period: float | None = None           # seconds. None uses the gait's default
    duty_factor: float | None = None            # fraction of a cycle in stance. None uses the gait's default
    body_offset: float = 0.0                    # body height relative to the stance pose, metres

    def __post_init__(self):
        if self.gait not in PHASES:
            raise ValueError(f'unknown gait {self.gait!r}; choose from {sorted(PHASES)}')
        if self.cycle_period is None:
            object.__setattr__(self, 'cycle_period', DEFAULTS[self.gait]['cycle_period'])
        if self.duty_factor is None:
            object.__setattr__(self, 'duty_factor', DEFAULTS[self.gait]['duty_factor'])
        if not 0.0 < self.duty_factor < 1.0:
            raise ValueError('duty_factor must be between 0 and 1')
        if self.cycle_period <= 0.0:
            raise ValueError('cycle_period must be positive')

    @property
    def stance_time(self) -> float:
        return self.cycle_period * self.duty_factor


@dataclass(frozen=True)
class VelocityCommand:
    """Body twist, like /cmd_vel: vx, vy in m/s in the body frame, yaw_rate in rad/s."""
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    @property
    def is_zero(self) -> bool:
        return abs(self.vx) < _IDLE and abs(self.vy) < _IDLE and abs(self.yaw_rate) < _IDLE

    def scaled(self, factor: float) -> 'VelocityCommand':
        return VelocityCommand(self.vx * factor, self.vy * factor, self.yaw_rate * factor)


def _displacement(cmd: VelocityCommand, dt: float) -> tuple:
    """Body displacement (dx, dy) in the starting body frame, and the turn angle, for a constant twist."""
    w = cmd.yaw_rate
    theta = w * dt
    if abs(theta) < 1e-9:
        return cmd.vx * dt, cmd.vy * dt, theta
    s, c = np.sin(theta), np.cos(theta)
    return (cmd.vx * s - cmd.vy * (1.0 - c)) / w, (cmd.vx * (1.0 - c) + cmd.vy * s) / w, theta


def _world_fixed_step(p: np.ndarray, cmd: VelocityCommand, dt: float) -> np.ndarray:
    """A world-fixed point seen from the body after the body moves for dt (dt may be negative)."""
    dx, dy, theta = _displacement(cmd, dt)
    s, c = np.sin(theta), np.cos(theta)
    x, y = p[..., 0] - dx, p[..., 1] - dy
    return np.stack((c * x + s * y, -s * x + c * y, p[..., 2]), axis=-1)


def _smooth(tau: float) -> float:
    """Minimum-jerk blend from 0 to 1, with zero velocity and acceleration at both ends."""
    return tau ** 3 * (10.0 + tau * (-15.0 + 6.0 * tau))


@dataclass
class GaitSample:
    t: float
    q: np.ndarray
    feet_body: np.ndarray
    feet_world: np.ndarray
    contact: np.ndarray
    body_position: np.ndarray
    body_yaw: float
    command: VelocityCommand    # the command actually applied, after limiting
    saturated: bool
    stability: Stability
    issues: list
    reachable: np.ndarray


class GaitGenerator:
    def __init__(self, model: RobotModel, params: GaitParams | None = None, stance: str = 'standby'):
        self.model = model
        self.params = params or GaitParams()
        self.home_world, self.height = nominal_stance(model, stance)
        self.height += self.params.body_offset
        self.home = self.home_world + np.array([0.0, 0.0, -self.height])
        offsets = PHASES[self.params.gait]
        self.offsets = np.array([offsets[leg.name] for leg in model.legs])
        self.reset()

    def reset(self, command: VelocityCommand | None = None) -> None:
        """Restart at t = 0. With a command, feet start on the periodic orbit for it; otherwise at home."""
        self.t = 0.0
        self.cycles = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.q_previous = None
        self.feet = self.home.copy()
        self.swinging = np.zeros(len(self.model.legs), bool)
        self.lift = self.home.copy()
        if command is not None and not command.is_zero:
            applied, _ = self.limit_command(command)
            for i in range(len(self.model.legs)):
                phase = self._phase(i)
                if phase < self.params.duty_factor:
                    s = (phase / self.params.duty_factor - 0.5) * self.params.stance_time
                    self.feet[i] = _world_fixed_step(self.home[i], applied, s)
                else:
                    self.swinging[i] = True
                    self.lift[i] = _world_fixed_step(self.home[i], applied, self.params.stance_time / 2.0)
                    self.feet[i] = self._swing_point(i, phase, applied)

    def set_params(self, params: GaitParams) -> None:
        """Change step length, step height or period while running; the phase stays continuous.

        A new gait or duty factor changes which feet should be down, so stand still and call reset() after it.
        """
        self.params = params
        offsets = PHASES[params.gait]
        self.offsets = np.array([offsets[leg.name] for leg in self.model.legs])

    def limit_command(self, command: VelocityCommand) -> tuple:
        """Scale a twist so that no stance foot travels further than step_length. Returns (applied, saturated)."""
        if command.is_zero:
            return VelocityCommand(), False
        vx = command.vx - command.yaw_rate * self.home[:, 1]
        vy = command.vy + command.yaw_rate * self.home[:, 0]
        stride = float(np.max(np.hypot(vx, vy))) * self.params.stance_time
        if stride <= self.params.step_length:
            return command, False
        return command.scaled(self.params.step_length / stride), True

    def max_speed(self) -> float:
        """Largest straight-line body speed the current parameters allow, in m/s."""
        return self.params.step_length / self.params.stance_time

    def _phase(self, i: int) -> float:
        return round((self.offsets[i] + self.cycles) % 1.0, 9) % 1.0

    def _swing_point(self, i: int, phase: float, cmd: VelocityCommand) -> np.ndarray:
        duty = self.params.duty_factor
        tau = min(max((phase - duty) / (1.0 - duty), 0.0), 1.0)
        target = _world_fixed_step(self.home[i], cmd, -self.params.stance_time / 2.0)
        blend = _smooth(tau)
        point = self.lift[i] + (target - self.lift[i]) * blend
        point[2] = self.home[i, 2] + self.params.step_height * np.sin(np.pi * tau) ** 2
        return point

    def update(self, dt: float, command: VelocityCommand = VelocityCommand(), checks: bool = True) -> GaitSample:
        """Advance by dt and return the new sample. checks=False skips collision and stability for speed."""
        cmd, saturated = self.limit_command(command)
        self.t += dt
        self.cycles += dt / self.params.cycle_period
        dx, dy, theta = _displacement(cmd, dt)
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        self.yaw += theta
        duty = self.params.duty_factor
        for i in range(len(self.model.legs)):
            phase = self._phase(i)
            wants_swing = phase >= duty
            if wants_swing and not self.swinging[i]:
                # Idle feet already at home stay down instead of stepping on the spot.
                if not (cmd.is_zero and np.linalg.norm(self.feet[i] - self.home[i]) < 1e-3):
                    self.swinging[i] = True
                    self.lift[i] = self.feet[i].copy()
            elif not wants_swing and self.swinging[i]:
                self.swinging[i] = False
                self.feet[i] = _world_fixed_step(self.home[i], cmd, -self.params.stance_time / 2.0 + phase * self.params.cycle_period)
                continue
            if self.swinging[i]:
                self.feet[i] = self._swing_point(i, phase, cmd)
            else:
                self.feet[i] = _world_fixed_step(self.feet[i], cmd, dt)
        return self._sample(cmd, saturated, dt, checks)

    def sample(self, command: VelocityCommand = VelocityCommand()) -> GaitSample:
        """The current state, without advancing time."""
        cmd, saturated = self.limit_command(command)
        return self._sample(cmd, saturated, None, True)

    def _sample(self, cmd, saturated, dt, checks) -> GaitSample:
        solution = feet_ik(self.model, self.feet)
        q = solution.q
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        position = np.array([self.x, self.y, self.height])
        feet_world = self.feet @ rotation.T + position
        contact = ~self.swinging
        issues = []
        if checks:
            issues = check_pose(self.model, q, solution=solution)
            if dt is not None and self.q_previous is not None:
                issues += check_servo_speed(self.model, self.q_previous, q, dt)
            com = rotation @ center_of_mass(self.model, q) + position
            stability = stability_margin(feet_world[contact, :2], com[:2])
            issues += stability_issues(stability)
        else:
            stability = Stability(np.empty((0, 2)), np.zeros(2), float('nan'), int(contact.sum()))
        self.q_previous = q.copy()
        return GaitSample(self.t, q, self.feet.copy(), feet_world, contact, position, self.yaw, cmd, saturated,
                          stability, issues, solution.reachable)


@dataclass
class IssueSummary:
    kind: str
    example: str
    count: int
    first_time: float


@dataclass
class GaitTrajectory:
    params: GaitParams
    requested: VelocityCommand
    applied: VelocityCommand
    t: np.ndarray               # (N,)
    q: np.ndarray               # (N, 6, 3)
    feet_body: np.ndarray       # (N, 6, 3)
    feet_world: np.ndarray      # (N, 6, 3)
    contact: np.ndarray         # (N, 6)
    body_position: np.ndarray   # (N, 3)
    body_yaw: np.ndarray        # (N,)
    margin: np.ndarray          # (N,)
    issues: list = field(default_factory=list)

    @property
    def min_feet(self) -> int:
        return int(self.contact.sum(axis=1).min())

    @property
    def min_margin(self) -> float:
        return float(self.margin.min())

    @property
    def peak_joint_speed(self) -> np.ndarray:
        """Peak speed in rad/s for coxa, femur and tibia, over all legs."""
        speed = np.abs(np.diff(self.q, axis=0)) / np.diff(self.t)[:, None, None]
        return speed.max(axis=(0, 1))

    def summary(self) -> str:
        speed = ', '.join(f'{n} {v:.2f}' for n, v in zip(JOINT_ORDER, self.peak_joint_speed))
        lines = [f'{self.params.gait}: period {self.params.cycle_period:.2f} s, duty {self.params.duty_factor:.2f}, '
                 f'stride limit {self.params.step_length * 1000:.0f} mm, lift {self.params.step_height * 1000:.0f} mm',
                 f'  command vx {self.applied.vx * 1000:.1f} mm/s, vy {self.applied.vy * 1000:.1f} mm/s, yaw {self.applied.yaw_rate:.3f} rad/s',
                 f'  min feet down {self.min_feet}, min stability margin {self.min_margin * 1000:.1f} mm',
                 f'  peak joint speed (rad/s): {speed}']
        lines += [f'  ISSUE x{i.count} from t={i.first_time:.2f} s: {i.example}' for i in self.issues] or ['  no issues']
        return '\n'.join(lines)


def generate_gait(model: RobotModel, params: GaitParams | None = None, command: VelocityCommand = VelocityCommand(0.015, 0.0, 0.0),
                  cycles: float = 3.0, dt: float = 0.02, stance: str = 'standby') -> GaitTrajectory:
    """Run a gait at a constant command, starting on its periodic orbit, and check every sample."""
    generator = GaitGenerator(model, params, stance)
    generator.reset(command)
    steps = int(round(cycles * generator.params.cycle_period / dt))
    samples = [generator.sample(command)]
    samples += [generator.update(dt, command) for _ in range(steps)]
    summary: dict = {}
    for sample in samples:
        for issue in sample.issues:
            key = (issue.kind, issue.leg, issue.joint, issue.other_leg)
            if key in summary:
                summary[key].count += 1
            else:
                summary[key] = IssueSummary(issue.kind, issue.message, 1, sample.t)
    return GaitTrajectory(
        generator.params, command, samples[0].command,
        np.array([s.t for s in samples]), np.array([s.q for s in samples]),
        np.array([s.feet_body for s in samples]), np.array([s.feet_world for s in samples]),
        np.array([s.contact for s in samples]), np.array([s.body_position for s in samples]),
        np.array([s.body_yaw for s in samples]), np.array([s.stability.margin for s in samples]),
        list(summary.values()))


def min_feet_for(params: GaitParams, samples: int = 2000) -> int:
    """Fewest stance feet at any phase, from the phase table alone."""
    offsets = np.array(list(PHASES[params.gait].values()))
    phases = (offsets[None, :] + np.linspace(0.0, 1.0, samples, endpoint=False)[:, None]) % 1.0
    return int((phases < params.duty_factor).sum(axis=1).min())


__all__ = ['DEFAULTS', 'PHASES', 'GaitGenerator', 'GaitParams', 'GaitSample', 'GaitTrajectory', 'Issue',
           'VelocityCommand', 'generate_gait', 'min_feet_for']
