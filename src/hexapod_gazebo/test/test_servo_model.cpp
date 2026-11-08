#include <gtest/gtest.h>

#include "hexapod_gazebo/servo_model.hpp"

using hexapod_gazebo::ServoParams;
using hexapod_gazebo::servo_torque;

TEST(ServoModel, TorqueNeverExceedsStall)
{
  const ServoParams s;
  EXPECT_DOUBLE_EQ(servo_torque(s, 3.0, 0.0, 0.0), s.stall_torque);
  EXPECT_DOUBLE_EQ(servo_torque(s, -3.0, 0.0, 0.0), -s.stall_torque);
}

TEST(ServoModel, NoDriveAtOrPastNoLoadSpeed)
{
  const ServoParams s;
  EXPECT_LE(servo_torque(s, 3.0, 0.0, s.no_load_speed), 0.0);
  EXPECT_GE(servo_torque(s, -3.0, 0.0, -s.no_load_speed), 0.0);
}

TEST(ServoModel, TorqueFallsAlongTheSpeedLine)
{
  const ServoParams s;
  EXPECT_NEAR(servo_torque(s, 3.0, 0.0, s.no_load_speed / 2.0), s.stall_torque / 2.0, 1e-9);
}

TEST(ServoModel, BrakingIsAvailableUpToStall)
{
  const ServoParams s;
  EXPECT_NEAR(servo_torque(s, -3.0, 0.0, s.no_load_speed / 2.0), -s.stall_torque, 1e-9);
}

TEST(ServoModel, DeadbandIgnoresSmallErrors)
{
  ServoParams s;
  s.deadband = 0.01;
  EXPECT_DOUBLE_EQ(servo_torque(s, 0.005, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(servo_torque(s, 0.02, 0.0, 0.0), s.kp * 0.02);
}
