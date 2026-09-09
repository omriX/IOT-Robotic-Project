// robot_calibration.ino
//
// Pure open-loop motor control + raw encoder access. It exists
// to measure, before robot_base is written:
//   - COUNTS_PER_WHEEL_REV  (zero, hand-rotate one turn, read)
//   - WHEEL_RADIUS_M        (back-solve from a measured straight drive)
//   - MOTOR_A_IS_LEFT / LEFT_DIR_SIGN / RIGHT_DIR_SIGN (watch pwm vs. rotation)
//   - MAX_TICKS_PER_INTERVAL and the deadband floor (ramp test)
//
// Commands, newline-terminated, accepted over both Serial and WebSerial:
//   zero                   Clear both encoder counts.
//   read                   Print cumulative counts, last 50 ms tick delta, battery V.
//   pwm <A|B|BOTH> <val>   Open-loop PWM (-255..255) to one or both motors.
//                          Deliberately has NO deadband snap -- that floor is
//                          exactly what this tool is used to measure.
//   ramp <A|B>             Auto-ramp one motor 0 -> PWM_MAX -> 0, logging
//                          pwm + ticks/50ms at each step.
//   stop                   Zero PWM on both motors, cancel any ramp in progress.
//   help                   List commands.
//
// Record results into docs/calibration_log.md.

#include <ESP32Encoder.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

const char *ssid = "...";
const char *password = "...";

AsyncWebServer server(80);

ESP32Encoder encoderA;
ESP32Encoder encoderB;

// Pins - match the POC code.
const int MOTOR_A_DIRECTION_PIN = 27;
const int MOTOR_A_PWM_PIN = 14;
const int MOTOR_B_DIRECTION_PIN = 13;
const int MOTOR_B_PWM_PIN = 12;

const int MOT_A_ENCODER_A = 23;
const int MOT_A_ENCODER_B = 22;
const int MOT_B_ENCODER_A = 19;
const int MOT_B_ENCODER_B = 21;

const int BATTERY_PIN = 34;
const int LOW_BATTERY_LED_PIN = 2;
const float BATTERY_VOLTAGE_DIVIDER_FACTOR = 2.8;
const float LOW_BATTERY_THRESHOLD_V = 6.6; // 2S battery, 3.3 V/cell
const float ADC_MAX_VALUE = 4095.0;
const float ADC_LOGIC_LEVEL_V = 3.3;

const int PWM_MAX = 250; // stay under the 255 hardware ceiling
const unsigned long CONTROL_PERIOD_MS = 50;
const unsigned long RAMP_STEP_MS = 200; // how often the ramp advances
const int RAMP_STEP_SIZE = 10;          // PWM increment per ramp step

// Open-loop PWM currently applied to each motor. Plain volatile int/char/enum
// reads and writes are single-word on the ESP32, which is adequate for this
// non-safety-critical bench tool; robot_base's shared-state needs the real
// spinlock discipline described in the plan, this sketch does not.
volatile int pwmA = 0;
volatile int pwmB = 0;

enum RampState
{
  RAMP_IDLE,
  RAMP_UP,
  RAMP_DOWN
};
volatile RampState rampState = RAMP_IDLE;
volatile char rampMotor = 'A';
unsigned long lastRampStepMs = 0;

volatile long lastDeltaA = 0, lastDeltaB = 0; // ticks in the most recent 50 ms interval

void logBoth(const String &line)
{
  Serial.println(line);
  WebSerial.println(line);
}

// Raw open-loop drive
void motors(int speedA, int speedB)
{
  if (speedA >= 0)
  {
    analogWrite(MOTOR_A_PWM_PIN, speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  }
  else
  {
    analogWrite(MOTOR_A_PWM_PIN, 256 + speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, HIGH);
  }

  if (speedB >= 0)
  {
    analogWrite(MOTOR_B_PWM_PIN, speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);
  }
  else
  {
    analogWrite(MOTOR_B_PWM_PIN, 256 + speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, HIGH);
  }
}

float measure_battery()
{
  float adc_voltage = (analogRead(BATTERY_PIN) / ADC_MAX_VALUE) * ADC_LOGIC_LEVEL_V;
  float battery_voltage = adc_voltage * BATTERY_VOLTAGE_DIVIDER_FACTOR;
  digitalWrite(LOW_BATTERY_LED_PIN, battery_voltage < LOW_BATTERY_THRESHOLD_V ? HIGH : LOW);
  return battery_voltage;
}

void cmdZero()
{
  encoderA.clearCount();
  encoderB.clearCount();
  lastDeltaA = 0;
  lastDeltaB = 0;
  logBoth("OK zero");
}

void cmdRead()
{
  String line = "count_A:" + String((long)encoderA.getCount());
  line += " count_B:" + String((long)encoderB.getCount());
  line += " dtick_A:" + String(lastDeltaA);
  line += " dtick_B:" + String(lastDeltaB);
  line += " battery_V:" + String(measure_battery(), 2);
  logBoth(line);
}

void cmdStop()
{
  rampState = RAMP_IDLE;
  pwmA = 0;
  pwmB = 0;
  logBoth("OK stop");
}

void cmdPwm(String motor, int value)
{
  value = constrain(value, -255, 255);
  motor.toUpperCase();
  if (motor == "A")
  {
    pwmA = value;
  }
  else if (motor == "B")
  {
    pwmB = value;
  }
  else if (motor == "BOTH")
  {
    pwmA = value;
    pwmB = value;
  }
  else
  {
    logBoth("ERR pwm: expected A|B|BOTH");
    return;
  }
  rampState = RAMP_IDLE; // manual pwm cancels any in-progress ramp
  logBoth("OK pwm " + motor + " " + String(value));
}

void cmdRamp(String motor)
{
  motor.toUpperCase();
  if (motor != "A" && motor != "B")
  {
    logBoth("ERR ramp: expected A or B");
    return;
  }
  rampMotor = motor.charAt(0);
  rampState = RAMP_UP;
  pwmA = 0;
  pwmB = 0;
  lastRampStepMs = millis();
  logBoth("OK ramp " + motor + " started");
}

void cmdHelp()
{
  logBoth("Commands: zero | read | pwm <A|B|BOTH> <val> | ramp <A|B> | stop | help");
}

void handleCommand(String cmd)
{
  cmd.trim();
  if (cmd.length() == 0)
    return;

  int sp1 = cmd.indexOf(' ');
  String verb = (sp1 == -1 ? cmd : cmd.substring(0, sp1));
  verb.toUpperCase();

  if (verb == "ZERO")
  {
    cmdZero();
  }
  else if (verb == "READ")
  {
    cmdRead();
  }
  else if (verb == "STOP")
  {
    cmdStop();
  }
  else if (verb == "HELP")
  {
    cmdHelp();
  }
  else if (verb == "PWM")
  {
    if (sp1 == -1)
    {
      logBoth("ERR pwm: usage 'pwm <A|B|BOTH> <val>'");
      return;
    }
    String rest = cmd.substring(sp1 + 1);
    rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 == -1)
    {
      logBoth("ERR pwm: usage 'pwm <A|B|BOTH> <val>'");
      return;
    }
    String motor = rest.substring(0, sp2);
    int value = rest.substring(sp2 + 1).toInt();
    cmdPwm(motor, value);
  }
  else if (verb == "RAMP")
  {
    if (sp1 == -1)
    {
      logBoth("ERR ramp: usage 'ramp <A|B>'");
      return;
    }
    cmdRamp(cmd.substring(sp1 + 1));
  }
  else
  {
    logBoth("ERR unknown command: " + cmd);
  }
}

// Advances the ramp state machine by one step, if RAMP_STEP_MS has elapsed.
// Called from controlTask, so it only ever touches pwmA/pwmB from that one task.
void stepRamp()
{
  if (millis() - lastRampStepMs < RAMP_STEP_MS)
    return;
  lastRampStepMs = millis();

  int value = (rampMotor == 'A') ? pwmA : pwmB;

  if (rampState == RAMP_UP)
  {
    value += RAMP_STEP_SIZE;
    if (value >= PWM_MAX)
    {
      value = PWM_MAX;
      rampState = RAMP_DOWN;
    }
  }
  else
  { // RAMP_DOWN
    value -= RAMP_STEP_SIZE;
    if (value <= 0)
    {
      value = 0;
      rampState = RAMP_IDLE;
    }
  }

  if (rampMotor == 'A')
    pwmA = value;
  else
    pwmB = value;

  if (rampState == RAMP_IDLE)
  {
    logBoth("OK ramp " + String(rampMotor) + " complete");
  }
}

void controlTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

  long prevCountA = 0;
  long prevCountB = 0;

  for (;;)
  {
    long currCountA = (long)encoderA.getCount();
    lastDeltaA = currCountA - prevCountA;
    prevCountA = currCountA;

    long currCountB = (long)encoderB.getCount();
    lastDeltaB = currCountB - prevCountB;
    prevCountB = currCountB;

    if (rampState != RAMP_IDLE)
    {
      stepRamp();
    }

    motors(pwmA, pwmB);

    if (rampState != RAMP_IDLE)
    {
      long activeDelta = (rampMotor == 'A') ? lastDeltaA : lastDeltaB;
      int activePwm = (rampMotor == 'A') ? pwmA : pwmB;
      logBoth("ramp_" + String(rampMotor) + " pwm:" + String(activePwm) + " dtick:" + String(activeDelta));
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  motors(0, 0);

  pinMode(LOW_BATTERY_LED_PIN, OUTPUT);
  digitalWrite(LOW_BATTERY_LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed! (WebSerial unavailable; Serial commands still work)");
  }
  else
  {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Calibration tool. Open http://" + WiFi.localIP().toString() + "/webserial"); });

  // WebSerial terminal available at http://<IPAddress>/webserial
  WebSerial.begin(&server);
  WebSerial.onMessage([&](uint8_t *data, size_t len)
                      {
    String cmd = "";
    for (size_t idx = 0; idx < len; idx++) cmd += char(data[idx]);
    handleCommand(cmd); });
  server.begin();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(MOT_A_ENCODER_A, MOT_A_ENCODER_B);
  encoderA.clearCount();
  encoderB.attachFullQuad(MOT_B_ENCODER_A, MOT_B_ENCODER_B);
  encoderB.clearCount();

  xTaskCreatePinnedToCore(
      controlTask,  // Task function
      "Calib_Task", // Name
      4096,         // Stack size in words
      NULL,         // Parameter
      1,            // Priority
      NULL,         // Task handle
      1             // Run on core 1
  );

  cmdHelp();
  Serial.println("Calibration tool ready. Encoders zeroed.");
}

void loop()
{
  if (Serial.available())
  {
    handleCommand(Serial.readStringUntil('\n'));
  }
  WebSerial.loop();
  delay(20);
}
