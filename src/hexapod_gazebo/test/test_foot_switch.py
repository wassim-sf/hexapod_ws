import pytest

from hexapod_gazebo.foot_switch import FootSwitch


def test_presses_at_the_threshold_and_not_below():
    s = FootSwitch(2.0, 1.0, 0.0)
    assert not s.update(0.0, 1.9)
    assert s.update(0.1, 2.0)


def test_release_force_gives_hysteresis():
    s = FootSwitch(2.0, 1.0, 0.0)
    s.update(0.0, 3.0)
    assert s.update(0.1, 1.5), 'between release and threshold it stays pressed'
    assert not s.update(0.2, 0.9)
    assert not s.update(0.3, 1.5), 'and stays open until the threshold again'


def test_report_is_delayed_both_ways():
    s = FootSwitch(2.0, 1.0, 0.05)
    assert not s.update(0.00, 3.0)
    assert not s.update(0.04, 3.0)
    assert s.update(0.05, 3.0)
    assert s.update(0.06, 0.0)
    assert not s.update(0.11, 0.0)


def test_release_above_threshold_is_refused():
    with pytest.raises(ValueError):
        FootSwitch(1.0, 2.0, 0.0)
