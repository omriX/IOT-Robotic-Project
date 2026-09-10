// diff_drive.h
//
// Pure math, header-only, no hardware -- unit-testable on the PC
// (test/test_diff_drive.cpp) before it ever runs on the robot.
//
// The PID keeps its own native unit: signed encoder ticks per control
// interval. TICKS_PER_METER bridges SI units and ticks in both directions.
#pragma once

#include <cmath>

constexpr double PI_D = 3.14159265358979323846;

constexpr double ticksPerMeter(double counts_per_wheel_rev, double wheel_radius_m)
{
  return counts_per_wheel_rev / (2.0 * PI_D * wheel_radius_m);
}

inline double wrapToPi(double angle)
{
  while (angle > PI_D)
    angle -= 2.0 * PI_D;
  while (angle < -PI_D)
    angle += 2.0 * PI_D;
  return angle;
}

struct WheelVelocities
{
  double left_mps;
  double right_mps;
};

struct WheelSetpoints
{
  double left_ticks_per_interval;
  double right_ticks_per_interval;
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct OdometryDelta
{
  double linear_v;  // m/s
  double angular_w; // rad/s
};

// Standard diff-drive forward kinematics: positive angular_w = CCW yaw
// (REP-103) means the right wheel speeds up and the left wheel slows down
// (or reverses) -- viewed from above, that spins the robot CCW.
inline WheelVelocities wheelLinearVelocities(double linear_v, double angular_w, double wheel_base_m)
{
  return {
      linear_v - angular_w * wheel_base_m / 2.0,
      linear_v + angular_w * wheel_base_m / 2.0};
}

// cmd_vel (v, w) -> per-wheel raw PID setpoints (signed ticks/interval).
// This is the ONLY place cmd_vel becomes a wheel setpoint and the ONLY place
// saturation happens.
inline WheelSetpoints cmdVelToWheelSetpoints(
    double linear_v, double angular_w,
    double wheel_base_m, double ticks_per_meter, double control_period_s,
    double left_dir_sign, double right_dir_sign,
    double max_ticks_per_interval)
{
  WheelVelocities wv = wheelLinearVelocities(linear_v, angular_w, wheel_base_m);

  double left = wv.left_mps * ticks_per_meter * control_period_s * left_dir_sign;
  double right = wv.right_mps * ticks_per_meter * control_period_s * right_dir_sign;

  double largest = std::fmax(std::fabs(left), std::fabs(right));
  if (largest > max_ticks_per_interval)
  {
    double scale = max_ticks_per_interval / largest;
    left *= scale;
    right *= scale;
  }

  return {left, right};
}

// Encoder deltas (already in the PID's forward-positive sign convention) ->
// odometry, integrated in place into `pose`. Returns the instantaneous
// linear/angular velocity for the same interval
inline OdometryDelta integrateOdometry(
    Pose2D &pose,
    double delta_ticks_left, double delta_ticks_right,
    double ticks_per_meter, double wheel_base_m, double control_period_s)
{
  double ds_l = delta_ticks_left / ticks_per_meter;
  double ds_r = delta_ticks_right / ticks_per_meter;
  double ds = (ds_r + ds_l) / 2.0;
  double dtheta = (ds_r - ds_l) / wheel_base_m;

  pose.x += ds * std::cos(pose.theta + dtheta / 2.0);
  pose.y += ds * std::sin(pose.theta + dtheta / 2.0);
  pose.theta = wrapToPi(pose.theta + dtheta);

  return {ds / control_period_s, dtheta / control_period_s};
}
