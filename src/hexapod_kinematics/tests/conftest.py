import pytest

from hexapod_kinematics import RobotModel


@pytest.fixture(scope='session')
def model():
    return RobotModel.load()
