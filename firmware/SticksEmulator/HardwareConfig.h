#pragma once

#include <stdint.h>

/// These are all the available targets
#define ESP32DOIT_DEVKIT_V1 1

/// Define the board used
#define BOARD ESP32DOIT_DEVKIT_V1
#define BAUDRATE 115200

/// Pins
#define SBUS_TXPIN 17 //TX2
#define SBUS_RXPIN 16 //RX2
#define NC_PIN 32 //Not connected pin

#define MSPTxPin 13
#define MSPRxPin 13
#define LED 9 


// Define the aircraft Bluetooth name
// This will be used for the bluetooth name and ssid wifi connection
#define AIRCRAFT_NAME "Drone Droid 01"

// Define the wifi password.
#define WIFI_PASSWORD "testpassword"

/// Outputs all messages on the serial port. Used to use Livetime via USB
#define USE_SERIAL_OUTPUT

// Enables the ArduinoOTA service. It allows flashing over WiFi and enters an emergency mode if a crashloop is detected.
// Comming soon features
//#define USE_ARDUINO_OTA
// defines the time after which the crash loop detection assumes the operation is stable
//#define CRASH_COUNT_RESET_TIME_MS 300000


#include "targets/target.h" // Needs to be at the bottom

void InitHardwarePins();
void blinkLED(int);
