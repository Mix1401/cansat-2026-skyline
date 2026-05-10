// mpu.h
// GY-521 / MPU6050 accelerometer + gyroscope + temperature
// I2C @ 0x68 (AD0=GND) on Wire, SDA=21 SCL=22

#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>     // Electronic Cats / Jeff Rowberg

struct MPUData {
    float ax, ay, az;    // g
    float gx, gy, gz;    // dps
    float temp;          // °C
};

extern MPUData mpu_data;
extern bool    is_MPU;

// accel_range : 2 / 4 / 8 / 16   (g)
// gyro_range  : 250 / 500 / 1000 / 2000   (dps)
// bw_hz       : 5 / 10 / 21 / 44 / 94 / 184 / 260   (DLPF)
bool initMPU(uint8_t accel_range, uint16_t gyro_range, uint8_t bw_hz);

void readMPU();

#endif
