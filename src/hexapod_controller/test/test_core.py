"""Controller logic without ROS or Gazebo."""
import numpy as np
import pytest

from hexapod_controller.core import Config, Controller, State
from hexapod_kinematics import RobotModel, VelocityCommand, feet_fk, nominal_stance


@pytest.fixture(scope='module')
def model():
    return RobotModel.load()


def run(c, seconds, command=None):
    out = None
    for _ in range(int(seconds * c.config.rate)):
        if command is not None:
            c.set_velocity(command)
        out = c.step()
    return out


def standing(model, **kwargs):
    c = Controller(model, Config(**kwargs))
    c.request_stand()
    run(c, c.config.stand_duration + 0.1)
    assert c.state == State.STAND
    return c


def test_starts_lying_down_and_waits(model):
    c = Controller(model)
    assert run(c, 1.0).state == State.LAYDOWN


def test_stand_up_ends_at_standby_with_feet_fixed_in_xy(model):
    c = Controller(model)
    start = feet_fk(model, c.laydown_q)
    c.request_stand()
    for _ in range(int(c.config.stand_duration * c.config.rate)):
        out = c.step()
        assert np.abs(feet_fk(model, out.q) - start)[:, :2].max() < 1e-3
        assert out.issues == []
    run(c, 0.1)
    assert np.allclose(c.q, model.pose_q('standby'), atol=1e-6)


def test_stand_up_is_smooth(model):
    c = Controller(model)
    c.request_stand()
    previous = c.q.copy()
    for _ in range(int(c.config.stand_duration * c.config.rate)):
        out = c.step()
        assert np.abs(out.q - previous).max() < 0.03
        previous = out.q.copy()


def test_walks_and_returns_to_stand(model):
    c = standing(model)
    assert run(c, 2.0, VelocityCommand(0.03)).state == State.WALK
    out = run(c, 6.0)
    assert out.state == State.STAND
    assert np.allclose(out.q, model.pose_q('standby'), atol=1e-3)


def test_velocity_ramps_and_is_capped(model):
    c = standing(model)
    assert run(c, 0.1, VelocityCommand(10.0)).command.vx < 0.02
    out = run(c, 3.0, VelocityCommand(10.0))
    assert out.command.vx == pytest.approx(c.gait.max_speed(), rel=1e-3)


@pytest.mark.parametrize('gait', ['tripod', 'ripple', 'wave'])
@pytest.mark.parametrize('idle', [0.0, 0.3, 0.77, 1.1, 1.45])
def test_published_targets_never_outrun_the_servo(model, gait, idle):
    """From lying down, through standing and walking, back to lying down, at the tuned parameters."""
    c = Controller(model, Config(gait=gait))
    limit = model.velocity.max() * c.dt
    previous = c.q.copy()

    def advance(seconds, command=None):
        nonlocal previous
        for _ in range(int(seconds * c.config.rate)):
            if command is not None:
                c.set_velocity(command)
            out = c.step()
            assert np.abs(out.q - previous).max() <= limit
            previous = out.q.copy()

    c.request_stand()
    advance(c.config.stand_duration + idle)
    advance(6.0, VelocityCommand(0.05))
    advance(0.5 + 8.0)
    assert c.state == State.STAND
    c.request_sit()
    advance(c.config.sit_duration + 0.1)
    assert c.state == State.LAYDOWN


def test_published_order_is_the_documented_joint_order(model):
    """/joint_commands is q.ravel(): leg 1 coxa, femur, tibia, then leg 2, up to leg 6. Every consumer relies on this."""
    assert [leg.id for leg in model.legs] == [1, 2, 3, 4, 5, 6]
    assert model.joints.keys() >= {'coxa', 'femur', 'tibia'}
    from hexapod_kinematics import JOINT_ORDER
    assert JOINT_ORDER == ('coxa', 'femur', 'tibia')


def test_stale_command_stops_walking(model):
    c = standing(model)
    run(c, 1.0, VelocityCommand(0.03))
    assert run(c, 6.0).state == State.STAND


def test_sit_from_walk_stops_first_then_lies_down(model):
    c = standing(model)
    run(c, 1.0, VelocityCommand(0.03))
    assert c.request_sit()
    assert c.state == State.STOPPING
    for _ in range(int(12 * c.config.rate)):
        out = c.step()
    assert out.state == State.LAYDOWN
    assert np.allclose(out.q, model.pose_q('laydown'), atol=1e-3)


def test_joint_limit_is_clamped_and_reported(model):
    c = standing(model)
    q = model.pose_q('standby')
    q[1, 2] = model.lower[2] - 0.2
    out, issues = c.safety.filter(c.time, q)
    assert out[1, 2] == pytest.approx(model.lower[2])
    assert any(i.kind == 'joint_limit' for i in issues)


def test_leg_collision_holds_last_safe_pose(model):
    c = standing(model)
    safe = c.safety.last_safe.copy()
    q = model.pose_q('standby')
    q[0, 0], q[1, 0] = -0.8, 0.8  # legs 1 and 2 swing together, within coxa limits
    out, issues = c.safety.filter(c.time, q)
    assert np.allclose(out, safe)
    assert any(i.kind == 'leg_collision' for i in issues)


def test_foot_under_body_holds_last_safe_pose(model):
    c = standing(model)
    safe = c.safety.last_safe.copy()
    q = model.pose_q('standby')
    q[1, 1], q[1, 2] = np.radians(-40), np.radians(-70)  # tibia folded under the body
    out, issues = c.safety.filter(c.time, q)
    assert np.allclose(out, safe)
    assert any(i.kind == 'body_collision' for i in issues)


def test_repeated_violations_stop_walking(model):
    c = standing(model)
    run(c, 1.0, VelocityCommand(0.03))
    bad = model.pose_q('standby')
    bad[0, 0], bad[1, 0] = -0.8, 0.8
    for _ in range(c.config.violation_limit):
        c.safety.filter(c.time, bad)
    assert c.safety.repeating
    out = c._finish(c.time, model.pose_q('standby'), None, VelocityCommand())
    assert out.fault
    assert run(c, 6.0, VelocityCommand(0.03)).state == State.STAND  # commands ignored until cleared
    c.clear_fault()
    assert run(c, 1.0, VelocityCommand(0.03)).state == State.WALK


def test_default_gait_walks_without_violations(model):
    c = standing(model)
    total = c.safety.total
    run(c, 30.0, VelocityCommand(0.05))
    assert c.safety.total == total


@pytest.mark.parametrize('command', [VelocityCommand(1.0), VelocityCommand(0.0, 1.0), VelocityCommand(0.0, -1.0), VelocityCommand(1.0, 1.0, 3.0)], ids=['forward', 'left', 'right', 'diagonal'])
@pytest.mark.parametrize('idle', [0.37, 1.23])
def test_tuned_gait_is_clean_from_any_start(model, command, idle):
    """The first swing starts from home, not the rear of the stride, so it is the closest to the femur limit. It depends on the start phase."""
    c = standing(model)
    run(c, idle)
    total = c.safety.total
    run(c, 6.0, command)
    run(c, 4.0)
    assert c.safety.total == total
    assert not c.fault


@pytest.mark.parametrize('gait', ['ripple', 'wave'])
def test_other_gaits_are_selectable(model, gait):
    c = standing(model, **{'gait': gait, f'{gait}_cycle_period': 3.0, f'{gait}_duty_factor': 0.8})
    assert c.gait.params.gait == gait
    assert run(c, 2.0, VelocityCommand(0.01)).state == State.WALK


def test_laydown_feet_are_on_the_ground_at_rest(model):
    _, height = nominal_stance(model)
    assert height > 0
    feet = feet_fk(model, model.pose_q('laydown'))
    assert np.allclose(feet[:, 2], -model.raw['body']['body_bottom_z'] * -1, atol=1e-3)


def test_sit_request_while_stopping_is_honoured(model):
    c = standing(model)
    run(c, 2.0, VelocityCommand(0.03))
    c.set_velocity(VelocityCommand())
    for _ in range(40):  # past cmd_timeout is not needed: a zero command starts the stop straight away
        c.step()
        if c.state == State.STOPPING:
            break
    assert c.state == State.STOPPING
    assert c.request_sit()
    for _ in range(int(20 * c.config.rate)):
        c.step()
    assert c.state == State.LAYDOWN
