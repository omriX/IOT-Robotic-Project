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

// Shared constants, matching robot_config.h's calibrated defaults.
constexpr double COUNTS_PER_WHEEL_REV = 4216.0;
constexpr double WHEEL_RADIUS_M = 0.021961;
constexpr double WHEEL_BASE_M = 0.116;
constexpr double CONTROL_PERIOD_S = 0.050;
constexpr double MAX_TICKS = 350.0;
constexpr double TICKS_PER_M = ticksPerMeter(COUNTS_PER_WHEEL_REV, WHEEL_RADIUS_M);

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

void test_yaw_to_quaternion()
{
  Quaternion2D q0 = yawToQuaternion(0.0);
  CHECK("yaw 0 -> identity quaternion", near(q0.z, 0.0) && near(q0.w, 1.0));

  Quaternion2D qpi = yawToQuaternion(PI_D);
  CHECK("yaw pi -> qz=1, qw=0", near(qpi.z, 1.0, 1e-6) && near(qpi.w, 0.0, 1e-6));
}

void test_straight_correction()
{
  StraightDriveCorrection control;
  auto step = [&](double left, double right, double dl, double dr) {
    return control.update(left, right, dl, dr, 0.05, 2.0, 0.20, 350.0);
  };
  step(100, 100, 30, -30);
  CHECK("straight correction: excludes motion before segment", near(control.travel_error_ticks, 0));
  auto sp = step(100, 100, 66, 55);
  CHECK("startup right lag: slow left and speed right", sp.left_ticks_per_interval < 100 && sp.right_ticks_per_interval > 100);
  CHECK("straight correction: preserves average speed", near(sp.left_ticks_per_interval + sp.right_ticks_per_interval, 200));
  step(100, 100, 100, 100);
  CHECK("matched speeds: retains correction for accumulated error", near(control.travel_error_ticks, 11) && control.trim_ticks > 0);
  for (int i = 0; i < 100; ++i)
    sp = step(100, 100, sp.left_ticks_per_interval, sp.right_ticks_per_interval);
  CHECK("ideal wheel response: recovers startup heading", std::fabs(control.travel_error_ticks) < 0.001);

  sp = step(100, 100, 10000, 0);
  CHECK("large disturbance: correction limited to 20 percent", near(sp.left_ticks_per_interval, 80) && near(sp.right_ticks_per_interval, 120));
  sp = step(340, 340, 340, 340);
  CHECK("straight correction: respects maximum wheel speed", near(sp.right_ticks_per_interval, 350));
  sp = step(60, 140, 60, 140);
  CHECK("turn command: unchanged and clears heading reference", near(sp.left_ticks_per_interval, 60) && near(sp.right_ticks_per_interval, 140) && near(control.travel_error_ticks, 0));

  step(-100, -100, 0, 0);
  sp = step(-100, -100, -55, -66);
  CHECK("reverse: corrects measured heading with proper signs", sp.left_ticks_per_interval < -100 && sp.right_ticks_per_interval > -100);
  sp = step(100, 100, -55, -66);
  CHECK("direction reversal: clears previous correction", near(sp.left_ticks_per_interval, 100) && near(control.travel_error_ticks, 0));
  step(100, 100, 80, 60);
  sp = step(0, 0, 5, 0);
  CHECK("stop: zero targets and cleared correction", near(sp.left_ticks_per_interval, 0) && near(sp.right_ticks_per_interval, 0) && near(control.travel_error_ticks, 0));
  sp = step(100, 100, 5, 0);
  CHECK("restart: captures new heading", near(sp.left_ticks_per_interval, 100) && near(control.travel_error_ticks, 0));
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
  test_yaw_to_quaternion();
  test_straight_correction();

  std::printf("\n%d/%d tests passed\n", tests_run - tests_failed, tests_run);
  return tests_failed == 0 ? 0 : 1;
}
