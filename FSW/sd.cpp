// sd.cpp
// CanSat Skyline — SD card data logging

#include "sd.h"
#include <SPI.h>
#include <SD.h>

static const char* LOG_FILE = "/cansat_log.csv";
static bool _ready = false;

bool sdInit() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] Mount failed — logging disabled");
        return false;
    }
    _ready = true;

    if (!SD.exists(LOG_FILE)) {
        File f = SD.open(LOG_FILE, FILE_WRITE);
        if (f) {
            f.println("time_ms,phase,ax,ay,az,avg_az,a_xandy,total_a,"
                      "temp_c,pressure_pa,altitude_m,"
                      "thermal_max,thermal_min,"
                      "bus_v,current_ma,"
                      "gps_lat,gps_lon,gps_alt,gps_sats");
            f.close();
        }
    }
    Serial.println("[SD] Ready");
    return true;
}

void sdLog(unsigned long time_ms,
           int   phase,
           float ax,    float ay,    float az,
           float avg_az, float a_xandy, float total_a,
           float temp,  float pressure, float altitude,
           float thermal_max, float thermal_min,
           float bus_v, float current_ma,
           double lat,  double lon,
           float gps_alt, int sats) {
    if (!_ready) return;
    File f = SD.open(LOG_FILE, FILE_APPEND);
    if (!f) return;
    f.printf("%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
             "%.2f,%.2f,%.2f,"
             "%.2f,%.2f,"
             "%.3f,%.2f,"
             "%.6f,%.6f,%.2f,%d\n",
             time_ms, phase,
             ax, ay, az, avg_az, a_xandy, total_a,
             temp, pressure, altitude,
             thermal_max, thermal_min,
             bus_v, current_ma,
             lat, lon, gps_alt, sats);
    f.close();
}