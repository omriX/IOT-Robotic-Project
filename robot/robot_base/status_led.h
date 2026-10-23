// status_led.h
//
// LED states: fault > critical battery > self-test running >
// no agent > solid (all OK).
// Non-blocking, millis()-based.
#pragma once

#include <Arduino.h>

struct LedPattern
{
  const unsigned long *segments; // alternating on/off durations (ms), starts ON
  int count;
};

constexpr unsigned long LED_TRIPLE_BLINK[] = {100, 100, 100, 100, 100, 500};
constexpr unsigned long LED_DOUBLE_BLINK[] = {150, 150, 150, 450};
constexpr unsigned long LED_FAST_BLINK[] = {100, 100};
constexpr unsigned long LED_SLOW_BLINK[] = {500, 500};

constexpr LedPattern PATTERN_FAULT = {LED_TRIPLE_BLINK, 6};
constexpr LedPattern PATTERN_CRITICAL_BATTERY = {LED_DOUBLE_BLINK, 4};
constexpr LedPattern PATTERN_SELFTEST = {LED_FAST_BLINK, 2};
constexpr LedPattern PATTERN_NO_AGENT = {LED_SLOW_BLINK, 2};
constexpr LedPattern PATTERN_SOLID = {nullptr, 0};

struct StatusLedState
{
  int pin = -1;
  LedPattern current = PATTERN_SOLID;
  unsigned long patternStartMs = 0;
};

inline void statusLedBegin(StatusLedState &led, int pin)
{
  led.pin = pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

inline void statusLedUpdate(StatusLedState &led, bool faulted, bool criticalBattery, bool selftestRunning, bool agentConnected)
{
  LedPattern next = PATTERN_SOLID;
  if (!agentConnected)
    next = PATTERN_NO_AGENT;
  if (selftestRunning)
    next = PATTERN_SELFTEST;
  if (criticalBattery)
    next = PATTERN_CRITICAL_BATTERY;
  if (faulted)
    next = PATTERN_FAULT;

  if (next.segments != led.current.segments)
  {
    led.current = next;
    led.patternStartMs = millis();
  }

  if (led.current.segments == nullptr)
  {
    digitalWrite(led.pin, HIGH);
    return;
  }

  unsigned long total = 0;
  for (int i = 0; i < led.current.count; i++)
    total += led.current.segments[i];
  unsigned long elapsed = (millis() - led.patternStartMs) % total;

  unsigned long acc = 0;
  for (int i = 0; i < led.current.count; i++)
  {
    acc += led.current.segments[i];
    if (elapsed < acc)
    {
      digitalWrite(led.pin, (i % 2 == 0) ? HIGH : LOW);
      return;
    }
  }
}
