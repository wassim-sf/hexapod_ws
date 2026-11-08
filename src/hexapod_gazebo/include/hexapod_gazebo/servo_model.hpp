#pragma once
#include <algorithm>
#include <cmath>

namespace hexapod_gazebo
{
// Hobby servo: proportional position loop whose torque is bounded by the motor's torque-speed line.
struct ServoParams
{
  double kp{12.0};
  double kd{0.1};
  double stall_torque{1.08};
  double no_load_speed{7.48};
  double deadband{0.0};
};

inline double servo_torque(const ServoParams & s, double target, double position, double velocity)
{
  double error = target - position;
  if (std::abs(error) < s.deadband) {
    error = 0.0;
  }
  const double demand = s.kp * error - s.kd * velocity;
  // Driving with the motion, torque falls to zero at the no-load speed. Braking is available up to stall.
  const double ratio = velocity / s.no_load_speed;
  const double upper = s.stall_torque * std::clamp(1.0 - ratio, 0.0, 1.0);
  const double lower = -s.stall_torque * std::clamp(1.0 + ratio, 0.0, 1.0);
  return std::clamp(demand, lower, upper);
}
}  // namespace hexapod_gazebo
