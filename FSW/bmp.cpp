// bmp.cpp
#include "bmp.h"

static Adafruit_BMP280 bmp;
BMPData bmp_data;
bool    is_BMP = false;
float   sea_level_hpa = 1013.25f;

bool initBMP() {
    // Try 0x76 then 0x77
    if (!bmp.begin(0x76) && !bmp.begin(0x77)) return false;

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                    Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                    Adafruit_BMP280::FILTER_X16,     // IIR filter
                    Adafruit_BMP280::STANDBY_MS_1);  // ~150 Hz output rate
    return true;
}

void readBMP() {
    bmp_data.temp     = bmp.readTemperature();
    bmp_data.pressure = bmp.readPressure() / 100.0f;          // hPa
    bmp_data.altitude = bmp.readAltitude(sea_level_hpa);      // m
}

void calibrateBMP(uint16_t samples) {
    if (samples == 0) samples = 1;
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        sum += bmp.readPressure() / 100.0f;
        delay(10);
    }
    sea_level_hpa = sum / samples;   // ground pressure = "sea level" baseline
}
