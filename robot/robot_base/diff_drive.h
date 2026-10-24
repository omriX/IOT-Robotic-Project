// Drive math with no hardware dependencies, unit-tested on the PC (test/test_diff_drive.cpp).
// Wheel speeds are signed encoder ticks per control interval, the PID's unit.
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

struct Quaternion2D
{
  double z;
  double w;
};

// Planar yaw -> quaternion (qx = qy = 0).
inline Quaternion2D yawToQuaternion(double theta)
{
  return {std::sin(theta / 2.0), std::cos(theta / 2.0)};
}

// Positive angular_w (CCW) speeds up the right wheel and slows the left.
inline WheelVelocities wheelLinearVelocities(double linear_v, double angular_w, double wheel_base_m)
{
  return {
      linear_v - angular_w * wheel_base_m / 2.0,
      linear_v + angular_w * wheel_base_m / 2.0};
}

// cmd_vel (v, w) -> raw per-wheel setpoints, both scaled down if either exceeds the limit.
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

// Holds the heading of a straight segment by trimming the wheel targets
// against the accumulated left/right encoder difference (forward-positive ticks).
class StraightDriveCorrection
{
public:
  double travel_error_ticks = 0;
  double trim_ticks = 0;

  WheelSetpoints update(double left_target, double right_target,
                        double left_delta, double right_delta,
                        double period_s, double gain_per_s,
                        double max_trim_fraction, double max_ticks)
  {
    const bool straight = std::fabs(left_target) > 1e-6 &&
                          std::fabs(left_target - right_target) < 1e-6;
    const int direction = left_target > 0 ? 1 : -1;
    if (!straight)
    {
      reset();
      return {left_target, right_target};
    }
    if (direction_ != direction)
    {
      // This sample belongs to the interval before this command took effect.
      reset();
      direction_ = direction;
    }
    else
    {
      travel_error_ticks += left_delta - right_delta;
    }

    const double limit = std::fmax(0.0, std::fmin(
        max_trim_fraction * std::fabs(left_target),
        max_ticks - std::fabs(left_target)));
    trim_ticks = std::fmax(-limit, std::fmin(limit,
        0.5 * gain_per_s * period_s * travel_error_ticks));
    return {left_target - trim_ticks, right_target + trim_ticks};
  }

private:
  int direction_ = 0;
  void reset()
  {
    direction_ = 0;
    travel_error_ticks = 0;
    trim_ticks = 0;
  }
};

// Friction feedforward PWM: a static term in the setpoint's direction plus a term proportional to speed.
inline double frictionFeedforward(double setpoint_ticks, double static_pwm, double pwm_per_tick)
{
  if (setpoint_ticks == 0)
    return 0;
  const double direction = setpoint_ticks > 0 ? 1.0 : -1.0;
  return direction * static_pwm + pwm_per_tick * setpoint_ticks;
}

// Short PWM burst for a wheel starting from standstill or stalled for stall_cycles.
// Ends on encoder movement or after kick_cycles; gives up after max_kicks.
class StartupKick
{
public:
  bool kicking = false;

  bool update(double setpoint_ticks, double measured_ticks, int kick_cycles,
              int stall_cycles, double stall_ticks, int max_kicks)
  {
    const int direction = setpoint_ticks > 0 ? 1 : (setpoint_ticks < 0 ? -1 : 0);
    if (direction == 0 || kick_cycles <= 0)
    {
      reset();
      return false;
    }

    const bool still = std::fabs(measured_ticks) < stall_ticks;
    if (direction != direction_)
    {
      // A reversal at speed is left to the PID; only a standing wheel is kicked.
      reset();
      direction_ = direction;
      if (still)
        start(kick_cycles);
    }
    else if (remaining_ > 0 && !still)
    {
      remaining_ = 0;
      kicks_ = 0;
    }
    else if (remaining_ == 0)
    {
      if (!still)
      {
        stalled_ = 0;
        kicks_ = 0;
      }
      else if (++stalled_ >= stall_cycles && kicks_ < max_kicks)
      {
        start(kick_cycles);
      }
    }

    kicking = remaining_ > 0;
    if (kicking)
      remaining_--;
    return kicking;
  }

  void reset()
  {
    direction_ = 0;
    remaining_ = 0;
    stalled_ = 0;
    kicks_ = 0;
    kicking = false;
  }

private:
  int direction_ = 0;
  int remaining_ = 0;
  int stalled_ = 0;
  int kicks_ = 0;

  void start(int kick_cycles)
  {
    remaining_ = kick_cycles;
    stalled_ = 0;
    kicks_++;
  }
};

// Integrates forward-positive encoder deltas into `pose`; returns the interval's velocities.
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
