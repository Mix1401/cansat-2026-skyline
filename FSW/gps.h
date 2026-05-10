// gps.h
// u-blox NEO-M8N on UART2
// RX=GPIO16 (ESP32 RX ← GPS TX)
// TX=GPIO17 (ESP32 TX → GPS RX)
//
// NOTE: Original FSW had GPS_TX_PIN=13 which conflicts with SD CS (also 13).
// Fixed to 17 — matches Deployment esp.cpp wiring.

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_BAUD     9600

struct GPSData {
    double  lat;
    double  lng;
    float   alt;
    float   speed_kmh;
    float   hdop;
    uint8_t sats;
    uint8_t hour, minute, second;
    bool    fix;
};

extern GPSData gps_data;
extern bool    is_GPS;

void initGPS();
void readGPS();

#endif