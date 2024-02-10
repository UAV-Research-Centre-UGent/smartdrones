#include "HardwareConfig.h"

#include <Arduino.h>

void InitHardwarePins() {
  // Pin initialization
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void blinkLED(int t) {
  // this is the pin number for the led
  static int x = 1;
  static unsigned long t1;
  static unsigned long t2;
  t2 = millis();
  if (t2 - t1 >= t) {
    x = 1 - x;
    t1 = millis();
    digitalWrite(LED, x);
  }
}
