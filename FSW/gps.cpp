// gps.cpp
#include "gps.h"

static HardwareSerial GPSSerial(2);   // UART2
static TinyGPSPlus    gps;

GPSData gps_data;
bool    is_GPS = false;

void initGPS() {
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    is_GPS = true;
}

void readGPS() {
    while (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }

    if (gps.location.isValid()) {
        gps_data.lat = gps.location.lat();
        gps_data.lng = gps.location.lng();
        gps_data.fix = (gps.satellites.isValid() && gps.satellites.value() >= 3);
    } else {
        gps_data.fix = false;
    }

    gps_data.alt       = gps.altitude.isValid()   ? gps.altitude.meters()  : 0.0f;
    gps_data.speed_kmh = gps.speed.isValid()      ? gps.speed.kmph()       : 0.0f;
    gps_data.hdop      = gps.hdop.isValid()       ? gps.hdop.hdop()        : 99.9f;
    gps_data.sats      = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (gps.time.isValid()) {
        gps_data.hour   = gps.time.hour();
        gps_data.minute = gps.time.minute();
        gps_data.second = gps.time.second();
    }
}
