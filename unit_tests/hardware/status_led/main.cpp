// status_led -- shows every status LED pattern from status_led.h, 6 s each.
//
// Watch the LED on LOW_BATTERY_LED_PIN and compare with the name printed on Serial.

#include <Arduino.h>
#include "robot_hardware.h"
#include "status_led.h"

constexpr unsigned long PATTERN_MS = 6000;

struct Step
{
  const char *name;
  bool faulted, criticalBattery, agentConnected;
};

constexpr Step STEPS[] = {
    {"solid ON          -- all OK, agent connected", false, false, true},
    {"slow blink        -- waiting for micro-ROS agent", false, false, false},
    {"double blink      -- battery below threshold", false, true, true},
    {"triple blink      -- fault (self-test failed)", true, false, true},
};
constexpr int STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

StatusLedState led;

void setup()
{
  Serial.begin(115200);
  // Hold the motors off; an unconfigured GPIO14 spins motor A at boot.
  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_A_PWM_PIN, 0);
  analogWrite(MOTOR_B_PWM_PIN, 0);
  digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);
  statusLedBegin(led, LOW_BATTERY_LED_PIN);
  delay(500);
  Serial.println("\n=== status LED test ===");
}

void loop()
{
  static int step = -1;
  static unsigned long stepStartMs = 0;
  if (step < 0 || millis() - stepStartMs >= PATTERN_MS)
  {
    step = (step + 1) % STEP_COUNT;
    stepStartMs = millis();
    Serial.printf("[%d/%d] %s\n", step + 1, STEP_COUNT, STEPS[step].name);
  }
  const Step &s = STEPS[step];
  statusLedUpdate(led, s.faulted, s.criticalBattery, s.agentConnected);
}
