// amg.h
// AMG8833 — 8x8 IR thermal array. I2C @ 0x69 (AD_SELECT=VCC) or 0x68.
// Conflicts with MPU6050 (0x68) — leave AMG at 0x69.

#ifndef AMG_H
#define AMG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

struct AMGData {
    float pixels[64];   // full frame, °C  (row-major 8x8)
    float min_t;
    float max_t;
    float avg_t;
    uint8_t hot_idx;    // 0..63 — index of hottest pixel
};

extern AMGData amg_data;
extern bool    is_AMG;

bool initAMG();
void readAMG();

#endif
