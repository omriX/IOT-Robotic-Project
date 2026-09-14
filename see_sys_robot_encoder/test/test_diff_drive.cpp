// test/test_diff_drive.cpp
//
// PC-only unit tests for diff_drive.h
// Run with:
//   g++ -std=c++17 -I robot_base test/test_diff_drive.cpp -o /tmp/t && /tmp/t
//
// Covers: straight/spin/arc kinematics, saturation scaling preserving curvature,
// meters<->ticks round-trip, odometry closing a square loop, and angle wrapping.

#include "diff_drive.h"
#include <cstdio>
#include <cmath>

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(name, cond)              \
  do                                   \
  {                                    \
    tests_run++;                       \
    if (!(cond))                       \
    {                                  \
      tests_failed++;                  \
      std::printf("FAIL: %s\n", name); \
    }                                  \
    else                               \
    {                                  \
      std::printf("PASS: %s\n", name); \
    }                                  \
  } while (0)

static bool near(double a, double b, double eps = 1e-6)
{
  return std::fabs(a - b) < eps;
}

// Shared constants, matching docs/calibration_log.md's final values.
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_RADIUS_M = 0.0220;
constexpr double WHEEL_BASE_M = 0.116;
constexpr double CONTROL_PERIOD_S = 0.050;
constexpr double MAX_TICKS = 350.0;
constexpr double TICKS_PER_M = ticksPerMeter(COUNTS_PER_WHEEL_REV, WHEEL_RADIUS_M);
constexpr double PWM_FLOOR = 40.0;
constexpr double TICKS_AT_FLOOR = 61.0;
constexpr double TICKS_PER_PWM = 1.419;
constexpr double PWM_MAX = 255.0;

static double pwmToTicks(double pwm)
{
  return TICKS_AT_FLOOR + (pwm - PWM_FLOOR) * TICKS_PER_PWM;
}

void test_straight()
{
  // v>0, w=0 -> both wheels equal.
  auto sp = cmdVelToWheelSetpoints(0.1, 0.0, WHEEL_BASE_M, TICKS_PER_M, CONTROL_PERIOD_S, 1.0, 1.0, MAX_TICKS);
  CHECK("straight: left == right", near(sp.left_ticks_per_interval, sp.right_ticks_per_interval));
  CHECK("straight: positive (forward)", sp.left_ticks_per_interval > 0);
}

void test_spin()
{
  // v=0, w>0 (CCW) -> wheels opposite, equal magnitude (second validation case).
  auto sp = cmdVelToWheelSetpoints(0.0, 1.0, WHEEL_BASE_M, TICKS_PER_M, CONTROL_PERIOD_S, 1.0, 1.0, MAX_TICKS);
  CHECK("spin: opposite signs", (sp.left_ticks_per_interval < 0) != (sp.right_ticks_per_interval < 0));
  CHECK("spin: equal magnitude", near(std::fabs(sp.left_ticks_per_interval), std::fabs(sp.right_ticks_per_interval)));
  CHECK("spin CCW: right wheel forward", sp.right_ticks_per_interval > 0);
  CHECK("spin CCW: left wheel backward", sp.left_ticks_per_interval < 0);
}

void test_arc()
{
  // v>0, w!=0 -> wheels unequal, same sign (third validation case).
  auto sp = cmdVelToWheelSetpoints(0.1, 0.5, WHEEL_BASE_M, TICKS_PER_M, CONTROL_PERIOD_S, 1.0, 1.0, MAX_TICKS);
  CHECK("arc: unequal magnitudes", !near(sp.left_ticks_per_interval, sp.right_ticks_per_interval));
  CHECK("arc: both forward (same sign)", sp.left_ticks_per_interval > 0 && sp.right_ticks_per_interval > 0);
  CHECK("arc CCW: right faster than left", sp.right_ticks_per_interval > sp.left_ticks_per_interval);
}

void test_saturation_preserves_curvature()
{
  // Large v -> one wheel exceeds MAX_TICKS and triggers scaling.
  auto sp = cmdVelToWheelSetpoints(2.0, 1.0, WHEEL_BASE_M, TICKS_PER_M, CONTROL_PERIOD_S, 1.0, 1.0, MAX_TICKS);
  double larger = std::fmax(std::fabs(sp.left_ticks_per_interval), std::fabs(sp.right_ticks_per_interval));
  CHECK("saturation: larger wheel clamped to MAX_TICKS", near(larger, MAX_TICKS, 1e-3));

  double v_left_raw = 2.0 - 1.0 * WHEEL_BASE_M / 2.0;
  double v_right_raw = 2.0 + 1.0 * WHEEL_BASE_M / 2.0;
  double expected_ratio = v_left_raw / v_right_raw;
  double actual_ratio = sp.left_ticks_per_interval / sp.right_ticks_per_interval;
  CHECK("saturation: curvature (L/R ratio) preserved", near(expected_ratio, actual_ratio, 1e-6));
}

void test_sign_convention()
{
  // Matches docs/calibration_log.md:
  auto sp = cmdVelToWheelSetpoints(0.1, 0.0, WHEEL_BASE_M, TICKS_PER_M, CONTROL_PERIOD_S, -1.0, -1.0, MAX_TICKS);
  CHECK("sign convention: forward cmd_vel -> negative raw setpoint",
        sp.left_ticks_per_interval < 0 && sp.right_ticks_per_interval < 0);
}

void test_meters_ticks_roundtrip()
{
  double distance_m = 1.234;
  double ticks = distance_m * TICKS_PER_M;
  double back_to_m = ticks / TICKS_PER_M;
  CHECK("meters->ticks->meters round-trip", near(distance_m, back_to_m, 1e-9));
}

void test_angle_wrap()
{
  CHECK("wrapToPi: already in range", near(wrapToPi(0.5), 0.5));
  CHECK("wrapToPi: just over +PI wraps negative", near(wrapToPi(PI_D + 0.1), -PI_D + 0.1, 1e-6));
  CHECK("wrapToPi: just under -PI wraps positive", near(wrapToPi(-PI_D - 0.1), PI_D - 0.1, 1e-6));
  CHECK("wrapToPi: 3*PI/2 wraps to -PI/2", near(wrapToPi(3.0 * PI_D / 2.0), -PI_D / 2.0, 1e-6));
}

void test_odometry_square_loop()
{
  Pose2D pose;
  double side_ticks = 2.0 * TICKS_PER_M; // 2 m straight, one big exact step
  double quarter_turn = PI_D / 2.0;
  // dtheta = (ds_r - ds_l) / WHEEL_BASE_M, with ds_r = -ds_l (in-place turn):
  // ds_r - ds_l = 2*ds_r = dtheta * WHEEL_BASE_M  =>  ds_r = dtheta*WHEEL_BASE_M/2
  double turn_ds = quarter_turn * WHEEL_BASE_M / 2.0;
  double turn_ticks = turn_ds * TICKS_PER_M;

  for (int side = 0; side < 4; side++)
  {
    integrateOdometry(pose, side_ticks, side_ticks, TICKS_PER_M, WHEEL_BASE_M, CONTROL_PERIOD_S);
    integrateOdometry(pose, -turn_ticks, turn_ticks, TICKS_PER_M, WHEEL_BASE_M, CONTROL_PERIOD_S);
  }

  CHECK("square loop: returns to x=0", near(pose.x, 0.0, 1e-6));
  CHECK("square loop: returns to y=0", near(pose.y, 0.0, 1e-6));
  CHECK("square loop: returns to theta=0 (mod 2pi)", near(wrapToPi(pose.theta), 0.0, 1e-6));
}

void test_feedforward()
{
  CHECK("ff: zero setpoint -> zero pwm", near(ticksToPwm(0.0, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM), 0.0));
  CHECK("ff: speed at the floor -> the floor pwm",
        near(ticksToPwm(TICKS_AT_FLOOR, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM), PWM_FLOOR, 1e-9));
  CHECK("ff: speed below the floor -> still the floor pwm (motor cannot go slower)",
        near(ticksToPwm(10.0, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM), PWM_FLOOR, 1e-9));
  CHECK("ff: negative setpoint -> negative pwm of equal magnitude",
        near(ticksToPwm(-152.5, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM),
             -ticksToPwm(152.5, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM), 1e-9));

  // ticks -> pwm -> ticks, against the ramp the constants were fitted to.
  double ticks = 152.5; // 0.1 m/s
  double pwm = ticksToPwm(ticks, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM);
  CHECK("ff: round-trips through the measured ramp", near(pwmToTicks(pwm), ticks, 1e-6));
  CHECK("ff: 0.1 m/s needs well over half the floor pwm", pwm > 2.0 * PWM_FLOOR);
}

void test_motor_command()
{
  double ff = ticksToPwm(-152.5, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM);

  CHECK("cmd: zero setpoint stops the wheel", motorCommand(0.0, 99.0, PWM_FLOOR, PWM_MAX) == 0);

  // The old failure: a small controller output fell inside the dead zone and was
  // chopped to a full stop, so the loop oscillated between stalled and kicking.
  CHECK("cmd: small output is lifted to the floor, not chopped to zero",
        motorCommand(-152.5, -5.0, PWM_FLOOR, PWM_MAX) == -(int)PWM_FLOOR);
  CHECK("cmd: output opposing the setpoint is lifted to the floor too",
        motorCommand(-152.5, 30.0, PWM_FLOOR, PWM_MAX) == -(int)PWM_FLOOR);

  CHECK("cmd: normal output passes through", motorCommand(-152.5, ff, PWM_FLOOR, PWM_MAX) == (int)ff);
  CHECK("cmd: clamped to +pwm_max", motorCommand(152.5, 900.0, PWM_FLOOR, PWM_MAX) == (int)PWM_MAX);
  CHECK("cmd: clamped to -pwm_max", motorCommand(-152.5, -900.0, PWM_FLOOR, PWM_MAX) == -(int)PWM_MAX);
}

void test_setpoint_ceiling_is_reachable()
{
  // MAX_TICKS_PER_INTERVAL used to allow ~0.23 m/s while the output limit
  // capped the motors near 0.11 m/s, so fast commands saturated forever.
  CHECK("ceiling: the fastest allowed setpoint is within the pwm range",
        ticksToPwm(MAX_TICKS, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM) <= PWM_MAX);
}

void test_wheel_control()
{
  constexpr double KP = 0.4, KI = 2.5, DT = 0.05, I_LIMIT = 60.0;
  double ff = ticksToPwm(-152.5, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM);

  WheelController c;
  CHECK("control: zero setpoint outputs zero",
        near(wheelControl(c, 0.0, -50.0, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX), 0.0));

  c.integral = 33.0;
  wheelControl(c, 0.0, 0.0, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
  CHECK("control: stopping clears the integral", near(c.integral, 0.0));

  // At the setpoint with no history, the output is exactly the feed-forward.
  WheelController d;
  CHECK("control: on target -> feed-forward only",
        near(wheelControl(d, -152.5, -152.5, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX), ff));

  // dt is explicit: the same error for the same time gives the same integral,
  // however the caller is scheduled. This is what ArduPID's timer got wrong.
  WheelController e, f;
  for (int i = 0; i < 20; i++)
    wheelControl(e, -152.5, -132.5, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
  for (int i = 0; i < 20; i++)
    wheelControl(f, -152.5, -132.5, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
  CHECK("control: integral depends only on error and dt", near(e.integral, f.integral));
  CHECK("control: one second of -20 tick error integrates to -Ki*20", near(e.integral, -KI * 20.0 * 1.0, 1e-9));

  // A wheel stuck at zero must not wind the integral up while saturated.
  WheelController g;
  for (int i = 0; i < 400; i++)
    wheelControl(g, -152.5, 0.0, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
  CHECK("control: integral stays within its limit", std::fabs(g.integral) <= I_LIMIT + 1e-9);
  double out = wheelControl(g, -152.5, 0.0, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
  CHECK("control: output stays within the pwm range", std::fabs(out) <= PWM_MAX + 1e-9);

  // Closing the loop against the measured motor ramp must settle, not oscillate.
  WheelController h;
  double ticks = 0.0, last = 0.0;
  for (int i = 0; i < 200; i++)
  {
    double pwm = wheelControl(h, -152.5, ticks, KP, KI, DT, PWM_FLOOR, TICKS_AT_FLOOR, TICKS_PER_PWM, I_LIMIT, PWM_MAX);
    int command = motorCommand(-152.5, pwm, PWM_FLOOR, PWM_MAX);
    double target = -pwmToTicks(std::fabs((double)command));
    ticks += 0.6 * (target - ticks); // first-order motor response
    last = ticks;
  }
  CHECK("control: settles on the setpoint against the measured ramp", near(last, -152.5, 2.0));
}

int main()
{
  test_straight();
  test_spin();
  test_arc();
  test_saturation_preserves_curvature();
  test_sign_convention();
  test_meters_ticks_roundtrip();
  test_angle_wrap();
  test_odometry_square_loop();
  test_feedforward();
  test_motor_command();
  test_setpoint_ceiling_is_reachable();
  test_wheel_control();

  std::printf("\n%d/%d tests passed\n", tests_run - tests_failed, tests_run);
  return tests_failed == 0 ? 0 : 1;
}
