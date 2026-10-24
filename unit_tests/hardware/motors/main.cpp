// motors -- checks the motor driver wiring, one motor at a time.
//
// Lift the robot so both wheels are off the ground before sending a command.
// Commands (type a letter, then Enter):
//   a   Motor A: 1 s forward (raw +PWM), pause, 1 s backward. Prints encoder deltas.
//   b   Motor B: same.
//   s   Run the firmware's boot self-test (selftest.h) 20 times per motor, print pass rate.
//   x   Stop both motors.
//
// Expected with the current wiring (robot_hardware.h, docs/hardware_calibration.md):
//   Motor A = right wheel, Motor B = left wheel.
//   Raw +PWM spins each wheel BACKWARD and the encoder count goes UP.

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "robot_hardware.h"
#include "selftest.h"

constexpr int TEST_PWM = 80;
constexpr unsigned long SPIN_MS = 1000;
constexpr int SELFTEST_REPEATS = 20;

ESP32Encoder encoderA;
ESP32Encoder encoderB;

void stopMotors()
{
  selftestSetRawSpeed(MOTOR_A_DIRECTION_PIN, MOTOR_A_PWM_PIN, 0);
  selftestSetRawSpeed(MOTOR_B_DIRECTION_PIN, MOTOR_B_PWM_PIN, 0);
}

long spinFor(int dirPin, int pwmPin, ESP32Encoder &encoder, int pwm)
{
  encoder.clearCount();
  selftestSetRawSpeed(dirPin, pwmPin, pwm);
  delay(SPIN_MS);
  selftestSetRawSpeed(dirPin, pwmPin, 0);
  delay(300);
  return (long)encoder.getCount();
}

void testMotor(char name, int dirPin, int pwmPin, ESP32Encoder &encoder, bool isLeft)
{
  Serial.printf("\nMotor %c (expected: %s wheel)\n", name, isLeft ? "LEFT" : "RIGHT");
  Serial.printf("  forward, raw PWM +%d for %lu ms... ", TEST_PWM, SPIN_MS);
  long forward = spinFor(dirPin, pwmPin, encoder, TEST_PWM);
  Serial.printf("encoder delta %ld\n", forward);
  Serial.printf("  backward, raw PWM -%d for %lu ms... ", TEST_PWM, SPIN_MS);
  long backward = spinFor(dirPin, pwmPin, encoder, -TEST_PWM);
  Serial.printf("encoder delta %ld\n", backward);

  bool moved = labs(forward) >= MIN_SELFTEST_TICKS && labs(backward) >= MIN_SELFTEST_TICKS;
  bool signsOk = forward > 0 && backward < 0;
  Serial.printf("  %s: %s\n", moved && signsOk ? "PASS" : "FAIL",
                !moved     ? "encoder barely moved -- check motor power, driver wiring, encoder wiring"
                : !signsOk ? "encoder counts the wrong way -- swap encoder A/B wires or fix DIR_SIGN"
                           : "moved both ways, encoder sign matches");
  Serial.println("  Check by eye: did the expected wheel turn, and did +PWM roll it BACKWARD?");
}

void repeatSelftest(char name, int dirPin, int pwmPin, ESP32Encoder &encoder)
{
  int passed = 0;
  long minForward = LONG_MAX, maxBackward = LONG_MIN;
  for (int i = 0; i < SELFTEST_REPEATS; i++)
  {
    SelftestResult r = runMotorSelftest(dirPin, pwmPin, encoder);
    if (r.passed)
      passed++;
    minForward = min(minForward, r.forward_delta);
    maxBackward = max(maxBackward, r.backward_delta);
    delay(300);
  }
  Serial.printf("Motor %c self-test: %d/%d passed | weakest fwd:%ld bwd:%ld (need >= %ld / <= -%ld)\n",
                name, passed, SELFTEST_REPEATS, minForward, maxBackward,
                MIN_SELFTEST_TICKS, MIN_SELFTEST_TICKS);
}

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  stopMotors();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(MOT_A_ENCODER_A, MOT_A_ENCODER_B);
  encoderB.attachFullQuad(MOT_B_ENCODER_A, MOT_B_ENCODER_B);

  delay(500);
  Serial.println("\n=== motors test ===");
  Serial.println("Lift the robot: both wheels off the ground.");
  Serial.println("Commands: a | b | s (self-test x20) | x (stop)");
}

void loop()
{
  if (!Serial.available())
    return;
  char c = Serial.read();
  switch (c)
  {
  case 'a':
    testMotor('A', MOTOR_A_DIRECTION_PIN, MOTOR_A_PWM_PIN, encoderA, MOTOR_A_IS_LEFT);
    break;
  case 'b':
    testMotor('B', MOTOR_B_DIRECTION_PIN, MOTOR_B_PWM_PIN, encoderB, !MOTOR_A_IS_LEFT);
    break;
  case 's':
    repeatSelftest('A', MOTOR_A_DIRECTION_PIN, MOTOR_A_PWM_PIN, encoderA);
    repeatSelftest('B', MOTOR_B_DIRECTION_PIN, MOTOR_B_PWM_PIN, encoderB);
    break;
  case 'x':
    stopMotors();
    Serial.println("stopped");
    break;
  }
}
