// mpu.cpp
#include "mpu.h"

static MPU6050 mpu;
MPUData mpu_data;
bool    is_MPU = false;

static float accel_lsb = 16384.0f;   // ±2g  default
static float gyro_lsb  = 131.0f;     // ±250 dps default

bool initMPU(uint8_t accel_range, uint16_t gyro_range, uint8_t bw_hz) {
    mpu.initialize();
    if (!mpu.testConnection()) return false;

    // ---- accel range ----
    switch (accel_range) {
        case 2:  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  accel_lsb = 16384.0f; break;
        case 4:  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  accel_lsb = 8192.0f;  break;
        case 8:  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  accel_lsb = 4096.0f;  break;
        case 16: mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); accel_lsb = 2048.0f;  break;
        default: return false;
    }

    // ---- gyro range ----
    switch (gyro_range) {
        case 250:  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  gyro_lsb = 131.0f; break;
        case 500:  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  gyro_lsb = 65.5f;  break;
        case 1000: mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); gyro_lsb = 32.8f;  break;
        case 2000: mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); gyro_lsb = 16.4f;  break;
        default: return false;
    }

    // ---- DLPF bandwidth ----
    // DLPF_CFG : 0=260Hz, 1=184, 2=94, 3=44, 4=21, 5=10, 6=5
    uint8_t cfg;
    if      (bw_hz >= 260) cfg = 0;
    else if (bw_hz >= 184) cfg = 1;
    else if (bw_hz >= 94)  cfg = 2;
    else if (bw_hz >= 44)  cfg = 3;
    else if (bw_hz >= 21)  cfg = 4;
    else if (bw_hz >= 10)  cfg = 5;
    else                   cfg = 6;
    mpu.setDLPFMode(cfg);

    return true;
}

void readMPU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    mpu_data.ax = ax / accel_lsb;
    mpu_data.ay = ay / accel_lsb;
    mpu_data.az = az / accel_lsb;
    mpu_data.gx = gx / gyro_lsb;
    mpu_data.gy = gy / gyro_lsb;
    mpu_data.gz = gz / gyro_lsb;

    // MPU6050 die-temp: T (°C) = raw / 340 + 36.53
    mpu_data.temp = mpu.getTemperature() / 340.0f + 36.53f;
}
