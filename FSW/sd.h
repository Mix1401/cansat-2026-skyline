// sd.h
// CanSat Skyline — SD card data logging
// SPI: MOSI=27, MISO=19, SCK=5, CS=13  (shares SPI bus with LoRa — OK)

#ifndef SD_H
#define SD_H

#include <Arduino.h>

#define SD_CS_PIN  13

bool sdInit();

// Log one complete sensor row to CSV
void sdLog(unsigned long time_ms,
           int   phase,
           float ax,    float ay,    float az,
           float avg_az, float a_xandy, float total_a,
           float temp,  float pressure, float altitude,
           float thermal_max, float thermal_min,
           float bus_v, float current_ma,
           double lat,  double lon,
           float gps_alt, int sats);

#endif