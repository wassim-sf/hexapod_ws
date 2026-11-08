"""FK, IK and frame transforms."""
import ast
import math
import pathlib

import numpy as np
import pytest
import yaml

from hexapod_kinematics import (RobotModel, body_to_leg, feet_fk, feet_ik, find_parameters_file, leg_fk, leg_ik, leg_to_body,
                                link_frames)

VERIFY_JOINTS = pathlib.Path(__file__).resolve().parents[3] / 'scripts/verify_joints.py'


def random_q(model, rng, shape=()):
    return rng.uniform(model.lower, model.upper, size=(*shape, 3))


def test_ik_fk_round_trip_on_random_reachable_points(model):
    rng = np.random.default_rng(1)
    points = leg_fk(model, random_q(model, rng, (20000,)))
    solution = leg_ik(model, points)
    assert solution.reachable.all()
    assert np.linalg.norm(leg_fk(model, solution.q) - points, axis=-1).max() < 1e-5  # 0.01 mm


def test_ik_recovers_joint_angles_within_limits(model):
    rng = np.random.default_rng(2)
    q = random_q(model, rng, (20000,))
    solved = leg_ik(model, leg_fk(model, q))
    radial = model.coxa + model.femur * np.cos(q[:, 1]) + model.tibia * np.sin(q[:, 1] + q[:, 2])
    # A foot exactly on the coxa axis has no defined coxa angle, so leave out that tiny neighbourhood.
    keep = np.abs(radial) > 1e-3
    assert np.abs(solved.q - q)[keep].max() < 1e-6


def test_leg_ik_scalar_and_batch_agree(model):
    q = model.poses['standby']
    p = leg_fk(model, q)
    assert np.allclose(leg_ik(model, p).q, q)
    assert np.allclose(leg_ik(model, np.array([p, p])).q[1], q)


def test_neutral_foot_is_out_along_the_femur_and_down_the_tibia(model):
    assert np.allclose(leg_fk(model, [0, 0, 0]), [model.coxa + model.femur, 0.0, -model.tibia])


def load_foot_fk(model):
    """Run the real foot_fk from verify_joints.py without importing ROS: extract just that function."""
    tree = ast.parse(VERIFY_JOINTS.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'foot_fk')
    namespace = {'math': math, 'np': np, 'P': yaml.safe_load(open(model.params_path))}
    exec(compile(ast.Module([function], []), str(VERIFY_JOINTS), 'exec'), namespace)
    return namespace['foot_fk'], namespace['P']


def test_agrees_with_foot_fk_in_verify_joints(model):
    foot_fk, params = load_foot_fk(model)
    rng = np.random.default_rng(3)
    for _ in range(200):
        q = random_q(model, rng, (6,))
        ours = feet_fk(model, q)
        for i, leg in enumerate(params['legs']):
            assert np.allclose(ours[i], foot_fk(leg, q[i]), atol=1e-12)


def test_body_and_leg_frames_are_inverse(model):
    point = np.array([0.123, -0.045, 0.067])
    for i in range(6):
        assert np.allclose(leg_to_body(model, i, body_to_leg(model, i, point)), point)


def test_body_to_leg_matches_feet_fk(model):
    rng = np.random.default_rng(4)
    q = random_q(model, rng, (6,))
    feet = feet_fk(model, q)
    for i in range(6):
        assert np.allclose(body_to_leg(model, i, feet[i]), leg_fk(model, q[i]))


def test_feet_ik_round_trip(model):
    rng = np.random.default_rng(5)
    feet = feet_fk(model, random_q(model, rng, (100, 6)))
    solution = feet_ik(model, feet)
    assert np.linalg.norm(feet_fk(model, solution.q) - feet, axis=-1).max() < 1e-5


def test_link_frames_end_at_the_foot(model):
    rng = np.random.default_rng(6)
    q = random_q(model, rng, (6,))
    feet = feet_fk(model, q)
    for i in range(6):
        assert np.allclose(link_frames(model, i, q[i])['foot'][:3, 3], feet[i])


def test_positive_joint_directions(model):
    base = leg_fk(model, [0, 0, 0])
    assert leg_fk(model, [0.4, 0, 0])[1] > base[1] + 0.01      # coxa: counter-clockwise from above
    assert leg_fk(model, [0, 0.4, 0])[2] > base[2] + 0.01      # femur: foot rises
    assert leg_fk(model, [0, 0, 0.4])[0] > base[0] + 0.01      # tibia: foot swings outward


def test_geometry_comes_from_the_yaml(tmp_path, model):
    data = yaml.safe_load(open(model.params_path))
    data['leg_geometry']['tibia_length'] = 0.100
    changed = tmp_path / 'params.yaml'
    changed.write_text(yaml.safe_dump(data))
    other = RobotModel.load(changed)
    assert other.tibia == 0.100
    assert leg_fk(other, [0, 0, 0])[2] == pytest.approx(-0.100)


def test_parameters_file_is_found_without_ros(monkeypatch):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    assert find_parameters_file().name == 'robot_parameters.yaml'


def test_missing_parameters_gives_a_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        RobotModel.load(tmp_path / 'missing.yaml')
