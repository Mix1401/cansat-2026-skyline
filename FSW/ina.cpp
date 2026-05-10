// ina.cpp
#include "ina.h"

static Adafruit_INA219 ina(0x40);
INAData ina_data;
bool    is_INA = false;

static uint8_t estPct2S(float v) {
    // 2S Li-Po: 8.4V full, 6.0V empty
    if (v >= 8.4f) return 100;
    if (v <= 6.0f) return 0;
    return (uint8_t)(((v - 6.0f) / 2.4f) * 100.0f);
}

bool initINA() {
    if (!ina.begin()) return false;
    // 32V bus, 2A current — fine for 2S pack drawing under ~1.5 A
    ina.setCalibration_32V_2A();
    return true;
}

void readINA() {
    float bus    = ina.getBusVoltage_V();
    float shunt  = ina.getShuntVoltage_mV();
    ina_data.bus_v      = bus + shunt / 1000.0f;     // total source V
    ina_data.shunt_mv   = shunt;
    ina_data.current_ma = ina.getCurrent_mA();
    ina_data.power_mw   = ina.getPower_mW();
    ina_data.batt_pct   = estPct2S(ina_data.bus_v);
}
