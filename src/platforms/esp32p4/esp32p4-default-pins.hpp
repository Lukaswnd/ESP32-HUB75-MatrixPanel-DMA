#pragma once

// Default pins for ESP32-P4 with Raspberry Pi pinout + Pastberry Pi Shield (DEBO MATRIXCTRL)
// These pins match the Pastberry Pi Shield mapping to Raspberry Pi header

#include "sdkconfig.h"
#if defined (CONFIG_IDF_TARGET_ESP32P4)

#define R1_PIN_DEFAULT 17   // RPi Pin 11 (GPIO 17)
#define G1_PIN_DEFAULT 18   // RPi Pin 12 (GPIO 18)
#define B1_PIN_DEFAULT 22   // RPi Pin 15 (GPIO 22)
#define R2_PIN_DEFAULT 23   // RPi Pin 16 (GPIO 23)
#define G2_PIN_DEFAULT 24   // RPi Pin 18 (GPIO 24)
#define B2_PIN_DEFAULT 25   // RPi Pin 22 (GPIO 25)

#define A_PIN_DEFAULT  4    // RPi Pin 7 (GPIO 4)
#define B_PIN_DEFAULT  27   // RPi Pin 13 (GPIO 27)
#define C_PIN_DEFAULT  5    // RPi Pin 29 (GPIO 5)
#define D_PIN_DEFAULT  6    // RPi Pin 31 (GPIO 6)
#define E_PIN_DEFAULT  12   // RPi Pin 32 (GPIO 12) - for 1/32 scan panels (64x64)

#define LAT_PIN_DEFAULT 26  // RPi Pin 37 (GPIO 26)
#define OE_PIN_DEFAULT  19  // RPi Pin 35 (GPIO 19)
#define CLK_PIN_DEFAULT 13  // RPi Pin 33 (GPIO 13)

#endif