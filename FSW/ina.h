// ina.h
// INA219 — pack voltage + current monitor (2S Li-Po 7.4V)
// I2C @ 0x40.  Wire INA219 IN+/IN- in series with the battery + line.

#ifndef INA_H
#define INA_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

struct INAData {
    float bus_v;        // V
    float shunt_mv;     // mV
    float current_ma;   // mA  (negative = charging if you wired it that way)
    float power_mw;     // mW
    uint8_t batt_pct;   // 0..100, rough estimate for 2S Li-Po
};

extern INAData ina_data;
extern bool    is_INA;

bool initINA();
void readINA();

#endif
