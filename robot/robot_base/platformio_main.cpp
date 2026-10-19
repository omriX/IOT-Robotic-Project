// PlatformIO entry point: compiles the Arduino sketch without duplicating it.
// The Arduino IDE also compiles every .cpp in the sketch folder, so this file
// must be empty there or setup()/loop() get defined twice.
#ifdef PLATFORMIO
#include <Arduino.h>
#include "robot_base.ino"
#endif
