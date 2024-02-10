#pragma once

#include <stdint.h>

/// These are all the available targets
#define ESP32DOIT_DEVKIT_V1 1
#define ESP32_DEVKITC_V4 2

/// Define the board used
#define BOARD ESP32DOIT_DEVKIT_V1  
#define BAUDRATE 115200

/// Pins
#define SBUS_TXPIN 17 //TX2
#define SBUS_RXPIN 16 //RX2
#define NC_PIN 32 //Not connected pin

/// Outputs all messages on the serial port. Used to use Livetime via USB
#define USE_SERIAL_OUTPUT

#include "targets/target.h" // Needs to be at the bottom

void InitHardwarePins();
void blinkLED(int);
