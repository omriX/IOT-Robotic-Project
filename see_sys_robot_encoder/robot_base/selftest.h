// selftest.h
//
// Drives one motor forward then backward and checks the encoder actually
// moved the right amount in the right direction.
// Run before the PID control task starts, since this writes
// the motor pins directly.
#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>

constexpr int SELFTEST_PWM = 60;
constexpr unsigned long SELFTEST_PHASE_MS = 200;
// Based on the ramp test measurement.
constexpr long MIN_SELFTEST_TICKS = 40;

struct SelftestResult
{
  bool passed;
  long forward_delta;
  long backward_delta;
};

inline void selftestSetRawSpeed(int dirPin, int pwmPin, int speed)
{
  if (speed >= 0)
  {
    analogWrite(pwmPin, speed);
    digitalWrite(dirPin, LOW);
  }
  else
  {
    analogWrite(pwmPin, 256 + speed);
    digitalWrite(dirPin, HIGH);
  }
}

inline SelftestResult runMotorSelftest(int dirPin, int pwmPin, ESP32Encoder &encoder)
{
  encoder.clearCount();
  selftestSetRawSpeed(dirPin, pwmPin, SELFTEST_PWM);
  delay(SELFTEST_PHASE_MS);
  long forward = (long)encoder.getCount();

  encoder.clearCount();
  selftestSetRawSpeed(dirPin, pwmPin, -SELFTEST_PWM);
  delay(SELFTEST_PHASE_MS);
  long backward = (long)encoder.getCount();

  selftestSetRawSpeed(dirPin, pwmPin, 0);

  bool passed = forward >= MIN_SELFTEST_TICKS && backward <= -MIN_SELFTEST_TICKS;
  return {passed, forward, backward};
}
