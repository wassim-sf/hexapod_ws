"""Body pose IK and static stability."""
import numpy as np
import pytest

from hexapod_kinematics import (UNREACHABLE, UNSTABLE, body_pose_ik, center_of_mass, convex_hull, feet_fk, nominal_stance, rpy_matrix,
                                stability_margin)


def test_zero_pose_returns_standby(model):
    assert np.allclose(body_pose_ik(model).q, model.pose_q('standby'), atol=1e-6)


@pytest.mark.parametrize('translation, rpy', [
    ((0.01, 0.0, 0.0), (0, 0, 0)), ((0.0, -0.015, 0.01), (0, 0, 0)), ((0, 0, -0.02), (0, 0, 0)),
    ((0, 0, 0), (0.1, 0, 0)), ((0, 0, 0), (0, -0.12, 0)), ((0, 0, 0), (0, 0, 0.2)),
    ((0.01, 0.01, 0.01), (0.05, -0.08, 0.1))])
def test_feet_stay_fixed_on_the_ground(model, translation, rpy):
    result = body_pose_ik(model, translation, rpy)
    reconstructed = feet_fk(model, result.q) @ rpy_matrix(*rpy).T + result.position
    assert np.linalg.norm(reconstructed - result.feet_world, axis=-1).max() < 1e-8
    assert np.allclose(result.feet_world[:, 2], 0.0)
    assert result.q.shape == (6, 3)


def test_moderate_body_pose_has_no_issues(model):
    assert body_pose_ik(model, (0.01, 0.0, 0.005), (0.05, 0.05, 0.1)).issues == []


def test_body_too_high_is_reported_as_unreachable(model):
    issues = body_pose_ik(model, (0, 0, 0.2)).issues
    assert any(i.kind == UNREACHABLE for i in issues)


def test_body_moved_past_the_feet_is_unstable(model):
    issues = body_pose_ik(model, (0.25, 0, 0)).issues  # the front feet reach about x = 153 mm
    assert any(i.kind == UNSTABLE for i in issues)
    assert any('outside the support polygon' in i.message for i in issues if i.kind == UNSTABLE)


def test_custom_feet_are_kept(model):
    feet, _ = nominal_stance(model)
    feet[:, 0] += 0.005
    assert np.allclose(body_pose_ik(model, feet_world=feet).feet_world, feet)


def test_total_mass_is_the_sum_of_yaml_masses(model):
    links = model.raw['links']
    expected = links['base']['mass'] + 6 * sum(links[n]['mass'] for n in ('coxa', 'femur', 'tibia', 'foot'))
    assert model.total_mass == pytest.approx(expected)
    assert 0.9 < model.total_mass < 1.0


def test_com_is_near_the_centre_by_symmetry(model):
    com = center_of_mass(model, model.pose_q('standby'))
    assert abs(com[0]) < 1e-3 and abs(com[1]) < 2e-3


def test_com_follows_a_moved_leg(model):
    base = center_of_mass(model, model.pose_q('standby'))
    q = model.pose_q('standby')
    q[1, 0] = 0.6  # leg 2 swings forward
    assert center_of_mass(model, q)[0] > base[0]


def test_convex_hull_drops_inner_and_collinear_points():
    square = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.5, 0)]
    hull = convex_hull(square)
    assert len(hull) == 4
    x, y = hull[:, 0], hull[:, 1]
    assert 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) == pytest.approx(1.0)  # positive: counter-clockwise


def test_margin_inside_and_outside_a_square():
    square = np.array([(0, 0), (2, 0), (2, 2), (0, 2)], float)
    assert stability_margin(square, (1.0, 1.0)).margin == pytest.approx(1.0)
    assert stability_margin(square, (0.5, 1.0)).margin == pytest.approx(0.5)
    outside = stability_margin(square, (3.0, 1.0))
    assert outside.margin == pytest.approx(-1.0) and not outside.stable


def test_margin_with_two_feet_is_never_positive():
    result = stability_margin(np.array([(0, 0), (1, 0)], float), (0.5, 0.0))
    assert result.margin == 0.0 and not result.stable


def test_margin_with_no_feet():
    assert not stability_margin(np.empty((0, 2)), (0, 0)).stable
