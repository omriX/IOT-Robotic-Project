// encoders -- checks both wheel encoders with the motors off.
//
// Turn the wheels by hand and watch the counts. Commands (letter, then Enter):
//   z   Zero both counts.
//
// Expected (robot_hardware.h, docs/calibration_log_1.md):
//   One full wheel turn = about COUNTS_PER_WHEEL_REV ticks (4216).
//   Encoder A = right wheel, encoder B = left wheel.
//   Rolling the robot FORWARD by hand makes both counts go DOWN
//   (raw +PWM drives backward with count going up, so DIR_SIGN = -1).

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "robot_hardware.h"

ESP32Encoder encoderA;
ESP32Encoder encoderB;

void setup()
{
  Serial.begin(115200);
  // Hold the motors off so the wheels turn freely.
  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_A_PWM_PIN, 0);
  analogWrite(MOTOR_B_PWM_PIN, 0);
  digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(MOT_A_ENCODER_A, MOT_A_ENCODER_B);
  encoderB.attachFullQuad(MOT_B_ENCODER_A, MOT_B_ENCODER_B);
  encoderA.clearCount();
  encoderB.clearCount();

  delay(500);
  Serial.println("\n=== encoders test ===");
  Serial.printf("Turn each wheel one full turn by hand; expect about %.0f ticks.\n", COUNTS_PER_WHEEL_REV);
  Serial.printf("Encoder A = %s wheel, encoder B = %s wheel.\n",
                MOTOR_A_IS_LEFT ? "LEFT" : "RIGHT", MOTOR_A_IS_LEFT ? "RIGHT" : "LEFT");
  Serial.println("Command: z (zero counts)");
}

void loop()
{
  if (Serial.available() && Serial.read() == 'z')
  {
    encoderA.clearCount();
    encoderB.clearCount();
    Serial.println("zeroed");
  }

  static long lastA = 0, lastB = 0;
  static unsigned long lastPrintMs = 0;
  long a = (long)encoderA.getCount();
  long b = (long)encoderB.getCount();
  if ((a != lastA || b != lastB) && millis() - lastPrintMs >= 200)
  {
    Serial.printf("A:%8ld (%+.2f turns)   B:%8ld (%+.2f turns)\n",
                  a, a / COUNTS_PER_WHEEL_REV, b, b / COUNTS_PER_WHEEL_REV);
    lastA = a;
    lastB = b;
    lastPrintMs = millis();
  }
}
