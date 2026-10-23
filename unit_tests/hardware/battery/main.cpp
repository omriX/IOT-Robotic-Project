// battery -- checks the battery voltage divider on BATTERY_PIN.
//
// Prints the averaged ADC reading, the pin voltage, and the battery voltage the
// firmware would report, twice a second. Measure the battery with a multimeter
// at the same time:
//   measured divider factor = V_multimeter / V_pin
// It should be close to BATTERY_VOLTAGE_DIVIDER_FACTOR (2.8). If it isn't,
// update the factor in robot/robot_base/robot_hardware.h.

#include <Arduino.h>
#include "robot_hardware.h"

constexpr int SAMPLES = 32;

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
  delay(500);
  Serial.println("\n=== battery test ===");
  Serial.printf("factor %.2f, low-battery threshold %.2f V\n",
                BATTERY_VOLTAGE_DIVIDER_FACTOR, LOW_BATTERY_THRESHOLD_V);
}

void loop()
{
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++)
    sum += analogRead(BATTERY_PIN);
  float raw = (float)sum / SAMPLES;
  float pinV = raw / ADC_MAX_VALUE * ADC_LOGIC_LEVEL_V;
  float batteryV = pinV * BATTERY_VOLTAGE_DIVIDER_FACTOR;

  Serial.printf("adc:%6.1f  pin:%.3f V  battery:%.2f V  %s\n", raw, pinV, batteryV,
                raw < 1        ? "<- reads 0: check the divider wiring"
                : batteryV < LOW_BATTERY_THRESHOLD_V ? "LOW"
                                                     : "ok");
  delay(500);
}
