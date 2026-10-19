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
  bool faulted, criticalBattery, selftestRunning, agentConnected;
};

constexpr Step STEPS[] = {
    {"solid ON          -- all OK, agent connected", false, false, false, true},
    {"slow blink        -- waiting for micro-ROS agent", false, false, false, false},
    {"fast blink        -- self-test running", false, false, true, true},
    {"double blink      -- battery below threshold", false, true, false, true},
    {"triple blink      -- fault (self-test failed)", true, false, false, true},
};
constexpr int STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

StatusLedState led;

void setup()
{
  Serial.begin(115200);
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
  statusLedUpdate(led, s.faulted, s.criticalBattery, s.selftestRunning, s.agentConnected);
}
