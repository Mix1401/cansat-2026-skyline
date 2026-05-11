#include <heltec_unofficial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050 on Wire1 (SDA=41, SCL=42)
Adafruit_MPU6050 mpu;

void setup() {
  heltec_setup();         // init OLED + LED + power rail on Wire (17/18)
  Serial.begin(115200);
  delay(500);

  // Wire1 for MPU6050
  Wire1.begin(41, 42);
  delay(500);

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "MPU6050 Init...");
  display.display();

  // Wake MPU6050 from sleep before begin()
  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B);  // PWR_MGMT_1
  Wire1.write(0x00);  // clear sleep bit
  Wire1.endTransmission();
  delay(100);

  if (!mpu.begin(0x68, &Wire1)) {
    Serial.println("[ERROR] MPU6050 not found!");
    display.clear();
    display.drawString(0, 0, "MPU6050 NOT FOUND");
    display.drawString(0, 16, "Check SDA=41 SCL=42");
    display.display();
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("[OK] MPU6050 ready");
  display.clear();
  display.drawString(0, 0, "MPU6050 OK");
  display.display();
  delay(1000);
}

void loop() {
  heltec_loop();

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  // Serial output
  Serial.printf("Accel  X:%.2f  Y:%.2f  Z:%.2f m/s2\n", ax, ay, az);
  Serial.printf("Gyro   X:%.2f  Y:%.2f  Z:%.2f rad/s\n", gx, gy, gz);

  // OLED output — 3 lines
  char line1[32], line2[32], line3[32];
  snprintf(line1, sizeof(line1), "AX:%.1f AY:%.1f AZ:%.1f", ax, ay, az);
  snprintf(line2, sizeof(line2), "GX:%.1f GY:%.1f GZ:%.1f", gx, gy, gz);
  snprintf(line3, sizeof(line3), "Temp: %.1f C", temp.temperature);

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  line1);
  display.drawString(0, 16, line2);
  display.drawString(0, 32, line3);
  display.display();

  delay(100);
}
