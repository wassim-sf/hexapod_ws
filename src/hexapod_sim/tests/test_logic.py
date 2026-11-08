"""Simulator logic and a headless run of the app, with no browser."""
import numpy as np
import pytest

from hexapod_kinematics import GaitGenerator, RobotModel, VelocityCommand, feet_fk
from hexapod_sim.logic import body_frame, gait_frame, joint_frame, pattern_q, status_html, table_html


@pytest.fixture(scope='module')
def model():
    return RobotModel.load()


def kinds(frame):
    return {i.kind for i in frame.issues}


def test_standby_is_clean_with_all_feet_on_the_ground(model):
    frame = joint_frame(model, model.pose_q('standby'))
    assert frame.issues == [] and frame.contact.all() and frame.margin > 0
    assert np.allclose(frame.feet_world[:, 2], 0.0)
    assert 'No warnings' in status_html(frame)


def test_raising_a_leg_drops_it_from_the_support_polygon(model):
    q = model.pose_q('standby')
    q[0, 1] += 0.4
    frame = joint_frame(model, q)
    assert not frame.contact[0] and frame.contact[1:].all()


def test_limit_warning_is_shown_in_red_with_the_reason(model):
    q = model.pose_q('standby')
    q[1, 2] = model.lower[2] - np.radians(10)
    frame = joint_frame(model, q)
    assert 'joint_limit' in kinds(frame)
    assert f'leg_2 (center_right): tibia {np.degrees(q[1, 2]):.1f}°' in status_html(frame)
    assert 'class="warning"' in status_html(frame)
    assert '#c0141c' in table_html(model, frame.q)


def test_table_lists_all_18_angles(model):
    assert table_html(model, model.pose_q('neutral')).count('<td style="text-align:right') == 18


def test_collision_warning(model):
    q = model.pose_q('standby')
    q[0, 0], q[1, 0] = -np.radians(44), np.radians(44)
    assert 'leg_collision' in kinds(joint_frame(model, q))


def test_servo_speed_warning_uses_the_move_time(model):
    start = model.pose_q('standby')
    q = start.copy()
    q[3, 1] += 1.0
    assert 'servo_speed' not in kinds(joint_frame(model, q, start, 1.0))
    assert 'servo_speed' in kinds(joint_frame(model, q, start, 0.02))


def test_body_mode_reports_unreachable_feet(model):
    frame = body_frame(model, (0.0, 0.0, 0.3), (0.0, 0.0, 0.0))
    assert 'unreachable' in kinds(frame)
    assert 'unreachable' in status_html(frame)


def test_html_is_escaped(model):
    frame = joint_frame(model, model.pose_q('standby'))
    frame.notes.append('<script>')
    assert '<script>' not in status_html(frame)


def test_pattern_applies_to_every_leg(model):
    q = pattern_q(model, 0.1, 0.2, 0.3)
    assert q.shape == (6, 3) and np.allclose(q, [0.1, 0.2, 0.3])


def test_gait_frame_notes_a_limited_command(model):
    generator = GaitGenerator(model)
    sample = generator.update(0.02, VelocityCommand(1.0))
    frame = gait_frame(model, sample, generator.max_speed())
    assert any('limited' in n for n in frame.notes)


def test_app_runs_every_mode_without_a_browser(model):
    from hexapod_sim.app import MODES, HexapodSim
    sim = HexapodSim(model, port=8765)
    try:
        for mode in MODES:
            sim.mode.value = mode
            sim._on_mode()
            sim.tick()
            assert sim.frame is not None and 'Stability margin' in sim.status.content
        assert sim.mode.value == MODES[3]
        before = sim.generator.t
        sim.playing = True
        for _ in range(10):
            sim.tick()
        assert sim.generator.t > before
        q = sim.frame.q
        assert np.allclose(feet_fk(model, q), sim.last_sample.feet_body, atol=1e-8)
    finally:
        sim.stop()


def test_nearest_point_distance_equals_the_margin(model):
    from hexapod_sim.logic import nearest_point_on_polygon
    frame = joint_frame(model, model.pose_q('standby'))
    nearest = nearest_point_on_polygon(frame.polygon, frame.com_xy)
    assert np.linalg.norm(nearest - frame.com_xy) == pytest.approx(frame.margin)
