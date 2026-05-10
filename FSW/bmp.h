// bmp.h
// BMP280 barometric pressure + temperature + altitude
// I2C @ 0x76 (SDO=GND) — auto-falls back to 0x77

#ifndef BMP_H
#define BMP_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

struct BMPData {
    float temp;       // °C
    float pressure;   // hPa
    float altitude;   // m AGL  (relative to ground baseline)
};

extern BMPData bmp_data;
extern bool    is_BMP;
extern float   sea_level_hpa;   // baseline used by altitude calc

bool initBMP();
void readBMP();

// Sample current pressure as ground level (call after sensor stabilises).
// After this, bmp_data.altitude is height above launch point.
void calibrateBMP(uint16_t samples = 50);

#endif
