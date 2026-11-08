"""Limits, unreachable points, collisions and servo speed are detected with clear reasons."""
import numpy as np
import pytest

from hexapod_kinematics import (JOINT_LIMIT, JOINT_ORDER, LEG_COLLISION, SERVO_SPEED, UNREACHABLE, check_joint_limits, check_leg_collisions,
                                check_pose, check_reachable, check_servo_speed, feet_ik, leg_ik)


def toward(model, i, j, angle):
    """(6, 3) pose with legs i and j each swung toward the other by angle, from standby."""
    q = model.pose_q('standby')
    wrap = (model.legs[j].yaw - model.legs[i].yaw + np.pi) % (2 * np.pi) - np.pi
    q[i, 0] = np.sign(wrap) * angle
    q[j, 0] = -np.sign(wrap) * angle
    return q


def test_limits_come_from_the_yaml(model):
    for name in JOINT_ORDER:
        assert np.degrees(model.joints[name].lower) == pytest.approx(model.raw['joints'][name]['lower_deg'], abs=1e-3)
        assert np.degrees(model.joints[name].upper) == pytest.approx(model.raw['joints'][name]['upper_deg'], abs=1e-3)


@pytest.mark.parametrize('leg, joint, side, word', [(0, 2, 'lower', 'below'), (5, 2, 'upper', 'above'), (2, 0, 'upper', 'above'), (3, 1, 'lower', 'below')])
def test_joint_limit_detected_with_reason(model, leg, joint, side, word):
    q = model.pose_q('standby')
    q[leg, joint] = model.lower[joint] - 0.02 if side == 'lower' else model.upper[joint] + 0.02
    issues = check_joint_limits(model, q)
    assert len(issues) == 1
    assert issues[0].kind == JOINT_LIMIT and issues[0].leg == leg + 1
    assert word in issues[0].message and f'leg_{leg + 1}' in issues[0].message


def test_limits_themselves_are_allowed(model):
    q = np.tile(model.upper, (6, 1))
    assert check_joint_limits(model, q) == []
    q = np.tile(model.lower, (6, 1))
    assert check_joint_limits(model, q) == []


def test_all_named_poses_are_within_limits(model):
    for name in model.poses:
        assert check_joint_limits(model, model.pose_q(name)) == []


@pytest.mark.parametrize('point', [(0.3, 0.0, 0.0), (0.0, 0.0, -0.4), (0.036 + 0.02, 0.0, 0.0)])
def test_unreachable_points_are_detected(model, point):
    assert not leg_ik(model, point).reachable


def test_unreachable_foot_reason_names_the_leg_and_distance(model):
    feet = model.hips + np.array([0.0, 0.0, -0.08])
    feet[4] = model.hips[4] + np.array([0.0, 0.0, -0.5])
    solution = feet_ik(model, feet)
    issues = [i for i in check_reachable(model, solution) if i.leg == 5]
    assert issues and issues[0].kind == UNREACHABLE
    assert 'leg_5' in issues[0].message and 'mm outside' in issues[0].message


def test_workspace_edge_is_reachable_just_inside(model):
    far = model.coxa + model.femur + model.tibia
    assert leg_ik(model, (far - 1e-6, 0.0, 0.0)).reachable
    assert not leg_ik(model, (far + 1e-6, 0.0, 0.0)).reachable


def test_standby_has_no_collision(model):
    assert check_leg_collisions(model, model.pose_q('standby')) == []


def test_every_neighbour_pair_is_checked(model):
    assert len(model.neighbour_pairs) == 6
    ids = {tuple(sorted((model.legs[a].id, model.legs[b].id))) for a, b in model.neighbour_pairs}
    assert ids == {(1, 2), (2, 3), (3, 6), (5, 6), (4, 5), (1, 4)}


# First mesh contact, measured by scripts/calibrate_leg_collision.py: side pairs 30-33 deg, front and rear pairs 60-66 deg.
@pytest.mark.parametrize('pair, clear, hit', [((0, 1), 25, 35), ((1, 2), 25, 35), ((3, 4), 25, 35), ((4, 5), 25, 35), ((2, 5), 55, 72), ((0, 3), 55, 72)])
def test_collision_detected_near_the_measured_contact(model, pair, clear, hit):
    i, j = pair
    assert check_leg_collisions(model, toward(model, i, j, np.radians(clear))) == []
    issues = check_leg_collisions(model, toward(model, i, j, np.radians(hit)))
    assert issues and issues[0].kind == LEG_COLLISION
    assert {issues[0].leg, issues[0].other_leg} == {i + 1, j + 1}
    assert 'collide' in issues[0].message


def test_one_leg_swinging_alone_does_not_collide_at_full_coxa_limit(model):
    q = model.pose_q('standby')
    q[0, 0] = model.joints['coxa'].lower
    assert check_leg_collisions(model, q) == []


def test_speed_limit_uses_the_yaml_velocity(model):
    a = model.pose_q('standby')
    b = a.copy()
    b[1, 2] += 0.2
    limit = model.joints['tibia'].velocity
    assert check_servo_speed(model, a, b, dt=0.2 / (limit * 0.9)) == []
    issues = check_servo_speed(model, a, b, dt=0.2 / (limit * 1.1))
    assert len(issues) == 1 and issues[0].kind == SERVO_SPEED
    assert 'leg_2' in issues[0].message and 'tibia' in issues[0].message and 'rad/s' in issues[0].message


def test_speed_check_rejects_non_positive_dt(model):
    q = model.pose_q('standby')
    with pytest.raises(ValueError):
        check_servo_speed(model, q, q, 0.0)


def test_check_pose_combines_everything(model):
    a = model.pose_q('standby')
    b = toward(model, 0, 1, np.radians(44))
    b[3, 2] = model.lower[2] - 0.1
    kinds = {i.kind for i in check_pose(model, b, previous_q=a, dt=0.01)}
    assert {LEG_COLLISION, JOINT_LIMIT, SERVO_SPEED} <= kinds
