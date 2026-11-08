"""The controller without ROS: state machine, command ramps and safety. All kinematics come from hexapod_kinematics."""
from __future__ import annotations

import enum
from dataclasses import dataclass

import numpy as np

from hexapod_kinematics import (BODY_COLLISION, JOINT_LIMIT, LEG_COLLISION, UNREACHABLE, GaitGenerator, GaitParams, RobotModel, VelocityCommand,
                                check_body_collisions, check_joint_limits, check_leg_collisions, check_reachable, feet_fk, feet_ik)


class State(enum.Enum):
    LAYDOWN = 'laydown'
    STANDING_UP = 'standing_up'
    STAND = 'stand'
    WALK = 'walk'
    STOPPING = 'stopping'
    SITTING_DOWN = 'sitting_down'


@dataclass
class Config:
    rate: float = 50.0
    gait: str = 'tripod'
    step_length: float = 0.05
    step_height: float = 0.02
    tripod_cycle_period: float = 1.6
    tripod_duty_factor: float = 0.55
    ripple_cycle_period: float = 2.4
    ripple_duty_factor: float = 0.70
    wave_cycle_period: float = 4.8
    wave_duty_factor: float = 0.85
    body_offset: float = 0.0
    stand_duration: float = 3.0
    sit_duration: float = 3.0
    cmd_timeout: float = 0.5
    max_linear_accel: float = 0.1
    max_yaw_accel: float = 0.5
    violation_limit: int = 5
    violation_window: float = 1.0


@dataclass
class Output:
    q: np.ndarray                # (6, 3), what is published
    state: State
    command: VelocityCommand     # applied after ramp and cap
    issues: list
    fault: bool


def _ease(s: float) -> float:
    """Minimum-jerk blend from 0 to 1."""
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10.0 + s * (-15.0 + 6.0 * s))


class Safety:
    """Checks every outgoing command. Clamps what it can, holds the last safe pose otherwise, and counts repeats."""

    def __init__(self, model: RobotModel, config: Config, start_q: np.ndarray):
        self.model = model
        self.config = config
        self.last_safe = start_q.copy()
        self.times = []
        self.total = 0

    def filter(self, t: float, q: np.ndarray, solution=None) -> tuple:
        issues = []
        if solution is not None:
            issues += check_reachable(self.model, solution)
        issues += check_joint_limits(self.model, q)
        out = np.clip(q, self.model.lower, self.model.upper)
        hard = check_leg_collisions(self.model, out) + check_body_collisions(self.model, out)
        issues += hard
        if hard:
            out = self.last_safe.copy()  # clamping can't undo a collision, so stay at the last safe pose
        else:
            self.last_safe = out.copy()
        if issues:
            self.total += 1
            self.times.append(t)
        self.times = [v for v in self.times if t - v <= self.config.violation_window]
        return out, issues

    @property
    def repeating(self) -> bool:
        return len(self.times) >= self.config.violation_limit


class Controller:
    def __init__(self, model: RobotModel, config: Config | None = None):
        self.model = model
        self.config = config or Config()
        c = self.config
        self.dt = 1.0 / c.rate
        self.params = GaitParams(gait=c.gait, step_length=c.step_length, step_height=c.step_height, cycle_period=getattr(c, f'{c.gait}_cycle_period'),
                                 duty_factor=getattr(c, f'{c.gait}_duty_factor'), body_offset=c.body_offset)
        self.gait = GaitGenerator(model, self.params)
        self.home = self.gait.home.copy()
        self.laydown_q = model.pose_q('laydown')
        self.stand_q = model.pose_q('standby')
        self.laydown_feet = feet_fk(model, self.laydown_q)
        self.safety = Safety(model, c, self.laydown_q)
        self.state = State.LAYDOWN
        self.time = 0.0
        self.phase_start = 0.0
        self.requested = VelocityCommand()
        self.requested_at = -1e9
        self.ramped = VelocityCommand()
        self.fault = False
        self.q = self.laydown_q.copy()
        self._sit_after_stop = False

    # ---- requests ----
    def set_velocity(self, command: VelocityCommand) -> None:
        self.requested = command
        self.requested_at = self.time

    def request_stand(self) -> bool:
        if self.state == State.LAYDOWN:
            self.state, self.phase_start = State.STANDING_UP, self.time
            self.fault = False
            return True
        return self.state in (State.STANDING_UP, State.STAND, State.WALK)

    def request_sit(self) -> bool:
        if self.state == State.STAND:
            self.state, self.phase_start = State.SITTING_DOWN, self.time
            return True
        if self.state in (State.WALK, State.STOPPING):
            self.state = State.STOPPING
            self._sit_after_stop = True
            return True
        return self.state in (State.SITTING_DOWN, State.LAYDOWN)

    def clear_fault(self) -> None:
        self.fault = False
        self.safety.times.clear()

    # ---- one 50 Hz tick ----
    def _ramp(self, target: VelocityCommand) -> VelocityCommand:
        c = self.config
        linear = c.max_linear_accel * self.dt
        yaw = c.max_yaw_accel * self.dt
        r = self.ramped

        def move(v, goal, step):
            return v + float(np.clip(goal - v, -step, step))

        self.ramped = VelocityCommand(move(r.vx, target.vx, linear), move(r.vy, target.vy, linear), move(r.yaw_rate, target.yaw_rate, yaw))
        return self.ramped

    def _target(self) -> VelocityCommand:
        fresh = self.time - self.requested_at <= self.config.cmd_timeout
        if not fresh or self.fault:
            return VelocityCommand()
        capped, _ = self.gait.limit_command(self.requested)
        return capped

    def step(self) -> Output:
        self.time += self.dt
        t = self.time
        solution = None
        if self.state == State.LAYDOWN:
            q = self.laydown_q
        elif self.state == State.STANDING_UP:
            s = (t - self.phase_start) / self.config.stand_duration
            q, solution = self._between_feet(_ease(s))
            if s >= 1.0:
                self.state = State.STAND
                self.gait.reset()
        elif self.state == State.SITTING_DOWN:
            s = (t - self.phase_start) / self.config.sit_duration
            q, solution = self._between_feet(1.0 - _ease(s))
            if s >= 1.0:
                self.state = State.LAYDOWN
        else:
            target = self._target()
            if self.state == State.STAND and not target.is_zero:
                self.state = State.WALK
            if self.state == State.WALK and target.is_zero:
                self.state = State.STOPPING
            if self.state == State.STOPPING and not target.is_zero and not self._sit_after_stop:
                self.state = State.WALK
            if self.state == State.STOPPING:
                target = VelocityCommand()
            command = self._ramp(target)
            sample = self.gait.update(self.dt, command, checks=False)
            solution = feet_ik(self.model, sample.feet_body)
            q = solution.q
            if self.state == State.STOPPING and self._settled(sample):
                self.state = State.STAND
                self.gait.reset()
                if self._sit_after_stop:
                    self._sit_after_stop = False
                    self.state, self.phase_start = State.SITTING_DOWN, self.time
            return self._finish(t, q, solution, sample.command)
        return self._finish(t, q, solution, VelocityCommand())

    def _finish(self, t, q, solution, command) -> Output:
        out, issues = self.safety.filter(t, q, solution)
        if self.safety.repeating and not self.fault:
            self.fault = True  # stop walking; _target now returns zero until clear_fault()
        self.q = out
        return Output(out, self.state, command, issues, self.fault)

    def _settled(self, sample) -> bool:
        ramp_done = self.ramped.is_zero
        return ramp_done and sample.contact.all() and np.allclose(sample.feet_body, self.home, atol=1.5e-3)

    def _between_feet(self, s: float) -> tuple:
        """Feet stay where they are on the ground; only the body rises or falls."""
        feet = self.laydown_feet + (self.home - self.laydown_feet) * s
        solution = feet_ik(self.model, feet)
        return solution.q, solution


__all__ = ['BODY_COLLISION', 'JOINT_LIMIT', 'LEG_COLLISION', 'UNREACHABLE', 'Config', 'Controller', 'Output', 'Safety', 'State']
