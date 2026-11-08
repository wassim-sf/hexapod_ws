"""Gaits at the default parameters, plus the generator's behaviour."""
import csv

import numpy as np
import pytest

from hexapod_kinematics import (DEFAULTS, SERVO_SPEED, GaitGenerator, GaitParams, VelocityCommand, csv_header, feet_fk, generate_gait,
                                min_feet_for, write_csv)

GAITS = sorted(DEFAULTS)
COMMANDS = {
    'forward': VelocityCommand(0.015, 0.0, 0.0), 'backward': VelocityCommand(-0.015, 0.0, 0.0),
    'sideways': VelocityCommand(0.0, 0.015, 0.0), 'turn': VelocityCommand(0.0, 0.0, 0.1),
    'combined': VelocityCommand(0.01, -0.005, 0.05),
}
MIN_FEET = {'tripod': 3, 'ripple': 4, 'wave': 5}


@pytest.fixture(scope='module')
def trajectories(model):
    return {(g, c): generate_gait(model, GaitParams(gait=g), cmd) for g in GAITS for c, cmd in COMMANDS.items()}


@pytest.mark.parametrize('gait', GAITS)
@pytest.mark.parametrize('command', COMMANDS)
def test_default_gaits_keep_three_feet_and_positive_margin_with_no_issues(trajectories, gait, command):
    trajectory = trajectories[(gait, command)]
    assert trajectory.min_feet >= 3, trajectory.summary()
    assert trajectory.min_feet == MIN_FEET[gait]
    assert trajectory.min_margin > 0.0, trajectory.summary()
    assert trajectory.issues == [], trajectory.summary()


@pytest.mark.parametrize('gait', GAITS)
def test_phase_table_gives_the_expected_feet_on_the_ground(gait):
    assert min_feet_for(GaitParams(gait=gait)) == MIN_FEET[gait]


@pytest.mark.parametrize('gait', GAITS)
def test_every_leg_swings_once_per_cycle(trajectories, gait):
    contact = trajectories[(gait, 'forward')].contact
    lifts = ((contact[:-1]) & (~contact[1:])).sum(axis=0)
    assert (lifts == 3).all()  # three cycles


@pytest.mark.parametrize('gait', GAITS)
@pytest.mark.parametrize('command', COMMANDS)
def test_stance_feet_do_not_slip_in_the_world(trajectories, gait, command):
    trajectory = trajectories[(gait, command)]
    both = trajectory.contact[:-1] & trajectory.contact[1:]
    slip = np.linalg.norm(np.diff(trajectory.feet_world, axis=0), axis=-1)
    assert slip[both].max() < 1e-9


@pytest.mark.parametrize('gait', GAITS)
def test_feet_stay_on_the_ground_in_stance_and_reach_step_height_in_swing(trajectories, gait):
    trajectory = trajectories[(gait, 'forward')]
    z = trajectory.feet_world[..., 2]
    assert np.abs(z[trajectory.contact]).max() < 1e-9
    assert z.max() == pytest.approx(trajectory.params.step_height, abs=2e-4)
    assert z.min() > -1e-9


@pytest.mark.parametrize('gait', GAITS)
def test_joint_angles_reproduce_the_foot_trajectories(model, trajectories, gait):
    trajectory = trajectories[(gait, 'combined')]
    assert np.abs(feet_fk(model, trajectory.q) - trajectory.feet_body).max() < 1e-8


@pytest.mark.parametrize('gait', GAITS)
def test_motion_is_periodic_in_the_body_frame(trajectories, gait):
    trajectory = trajectories[(gait, 'forward')]
    per_cycle = (len(trajectory.t) - 1) // 3
    assert np.abs(trajectory.feet_body[per_cycle] - trajectory.feet_body[2 * per_cycle]).max() < 1e-3


@pytest.mark.parametrize('gait', GAITS)
def test_body_travels_at_the_commanded_speed(trajectories, gait):
    trajectory = trajectories[(gait, 'forward')]
    speed = trajectory.body_position[-1, 0] / trajectory.t[-1]
    assert speed == pytest.approx(trajectory.applied.vx, rel=1e-6)


def test_yaw_rate_is_followed(trajectories):
    trajectory = trajectories[('tripod', 'turn')]
    assert trajectory.body_yaw[-1] / trajectory.t[-1] == pytest.approx(0.1, rel=1e-6)


def test_over_fast_command_is_limited_to_step_length(model):
    generator = GaitGenerator(model, GaitParams(gait='wave'))
    applied, saturated = generator.limit_command(VelocityCommand(1.0, 0.0, 0.0))
    assert saturated
    assert applied.vx == pytest.approx(generator.max_speed())


def test_limit_keeps_the_command_direction(model):
    generator = GaitGenerator(model)
    applied, _ = generator.limit_command(VelocityCommand(1.0, 0.5, 0.0))
    assert applied.vy / applied.vx == pytest.approx(0.5)


def test_slow_command_is_not_limited(model):
    applied, saturated = GaitGenerator(model).limit_command(VelocityCommand(0.005, 0.0, 0.0))
    assert not saturated and applied.vx == 0.005


def test_zero_command_keeps_all_feet_down(model):
    generator = GaitGenerator(model)
    for _ in range(400):
        sample = generator.update(0.02, VelocityCommand())
        assert sample.contact.all()
    assert np.allclose(sample.feet_body, generator.home)
    assert np.allclose(sample.body_position[:2], 0.0)


def test_starts_from_standing_without_a_jump(model):
    generator = GaitGenerator(model)
    previous = generator.sample().q
    for _ in range(400):
        sample = generator.update(0.02, VelocityCommand(0.015))
        assert np.abs(sample.q - previous).max() < 0.05
        assert not [i for i in sample.issues if i.kind == SERVO_SPEED]
        previous = sample.q


def test_command_changes_stay_valid(model):
    generator = GaitGenerator(model)
    rng = np.random.default_rng(8)
    previous = generator.sample().feet_body
    command = VelocityCommand()
    for k in range(1500):
        if k % 75 == 0:
            command = VelocityCommand(rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(-0.15, 0.15))
        sample = generator.update(0.02, command)
        assert sample.contact.sum() >= 3
        assert np.linalg.norm(sample.feet_body - previous, axis=-1).max() < 0.012
        assert sample.stability.margin > 0.0
        previous = sample.feet_body


def test_low_duty_tripod_reports_too_few_feet(model):
    trajectory = generate_gait(model, GaitParams(gait='tripod', duty_factor=0.4))
    assert trajectory.min_feet < 3
    assert any(i.kind == 'too_few_feet' for i in trajectory.issues)


def test_very_fast_gait_reports_servo_speed(model):
    params = GaitParams(gait='tripod', cycle_period=0.2, step_height=0.03)
    trajectory = generate_gait(model, params, VelocityCommand(0.1), cycles=5, dt=0.005)
    assert any(i.kind == SERVO_SPEED for i in trajectory.issues)


def test_unknown_gait_and_bad_parameters_are_rejected():
    with pytest.raises(ValueError):
        GaitParams(gait='gallop')
    with pytest.raises(ValueError):
        GaitParams(duty_factor=1.0)
    with pytest.raises(ValueError):
        GaitParams(cycle_period=0.0)


def test_csv_export(model, tmp_path):
    trajectory = generate_gait(model, GaitParams(), cycles=1)
    path = tmp_path / 'gait.csv'
    write_csv(model, trajectory, path)
    rows = list(csv.reader(open(path)))
    assert rows[0] == csv_header(model)
    assert len(rows) == len(trajectory.t) + 1
    assert all(len(r) == len(rows[0]) for r in rows)
    header = rows[0]
    assert header[0] == 'time' and 'leg_6_tibia' in header and 'leg_1_foot_z' in header
    assert float(rows[5][header.index('leg_3_femur')]) == pytest.approx(trajectory.q[4, 2, 1], abs=1e-6)
    assert float(rows[5][header.index('leg_3_foot_x')]) == pytest.approx(trajectory.feet_world[4, 2, 0], abs=1e-6)
