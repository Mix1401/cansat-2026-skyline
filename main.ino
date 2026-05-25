// main.ino — CanSat Skyline FSW  [v4 — no deployment]
// Board  : Heltec ESP32 WiFi LoRa 32 V3 (ESP32-S3 + SX1262, 923 MHz)
// Sensors: MPU6050 · BMP280 · INA219 · AMG8833 · u-blox NEO-M8N
// Radio  : SX1262 onboard — raw LoRa P2P
// Storage: SD อยู่ที่ ESP32-CAM — Heltec ส่งข้อมูลผ่าน UART1
// Power  : MOSFET (pin 4) — sensor rail cut on landing
//
// I2C  : SDA=41, SCL=42
// UART1: ESP32-CAM TX=1, RX=2
// UART2: GPS RX=47, TX=48
// SPI  : SCK=9, MISO=11, MOSI=10  (LoRa SS=8)
// Vext : GPIO36 LOW = sensors ON   (Heltec V3)

// ═══════════════════════════════════════════════════════════════
//  INCLUDES
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <FS.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <MPU6050.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>

// ═══════════════════════════════════════════════════════════════
//  DEFINES
// ═══════════════════════════════════════════════════════════════
#define TEAM_ID          "2026_TH_TEAM"
#define TX_INTERVAL_MS   1000
#define BEACON_INTERVAL  5000

#define VEXT_PIN         36

#define LORA_SCK          9
#define LORA_MISO        11
#define LORA_MOSI        10
#define LORA_SS           8
#define LORA_RST         12
#define LORA_DIO1        14
#define LORA_BUSY        13
#define LORA_FREQ        923.25f
#define LORA_BW          125.0f
#define LORA_SF           11
#define LORA_CR            5
#define LORA_SYNCWORD    0x12
#define LORA_TX_POWER     17
#define LORA_PREAMBLE      8

#define GPS_RX_PIN       47
#define GPS_TX_PIN       48
#define GPS_BAUD         9600

#define I2C_SDA          41
#define I2C_SCL          42

#define CAM_TX_PIN        1   // Heltec TX → ESP32-CAM GPIO3 (RX) !!
#define CAM_RX_PIN        2   // Heltec RX ← ESP32-CAM GPIO1 (TX) !!
#define CAM_BAUD      115200

#define POWER_CUT_PIN          4
#define LAUNCH_AZ_THRESHOLD  -9.0f
#define FREEFALL_THRESHOLD     2.0f
#define EJECT_ANGLE_THRESHOLD  8.84f
#define NORMAL_EJECT_DELAY_MS  2000
#define EMERGENCY_EJECT_MS    10000
#define LAND_ALT_DELTA_M       0.05f
#define LAND_ACCEL_LO          8.0f
#define LAND_ACCEL_HI         12.0f
#define LAND_CONFIRM_MS        3000
#define LAND_FAILSAFE_MS     120000
#define WINDOW_SIZE           10

// ═══════════════════════════════════════════════════════════════
//  STRUCTS & ENUMS
// ═══════════════════════════════════════════════════════════════
struct AMGData { float pixels[64]; float min_t, max_t, avg_t; uint8_t hot_idx; };
struct BMPData { float temp, pressure, altitude; };
struct GPSData { double lat, lng; float alt, speed_kmh, hdop; uint8_t sats, hour, minute, second; bool fix; };
struct INAData { float bus_v, shunt_mv, current_ma, power_mw; uint8_t batt_pct; };
struct MPUData { float ax, ay, az, gx, gy, gz, temp; };

enum FlightPhase { PHASE_WAITING=0, PHASE_ASCENT=1, PHASE_DESCENT=2, PHASE_LANDED=3 };

// ═══════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════
AMGData     amg_data;
BMPData     bmp_data;
GPSData     gps_data;
INAData     ina_data;
MPUData     mpu_data;
FlightPhase flight_phase = PHASE_WAITING;

bool is_AMG=false, is_BMP=false, is_GPS=false;
bool is_INA=false, is_LoRa=false, is_MPU=false;

float sea_level_hpa = 1013.25f;
float avg_ax=0, avg_ay=0, avg_az=0;
float a_xandy=0, total_a=0;

static SPIClass       _sharedSPI;
static HardwareSerial _camSerial(1);  // UART1 → ESP32-CAM

// ═══════════════════════════════════════════════════════════════
//  I2C BUS RECOVERY
// ═══════════════════════════════════════════════════════════════
static void i2cRecover() {
    Wire.end();
    pinMode(I2C_SDA, OUTPUT);
    pinMode(I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
        digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
    }
    digitalWrite(I2C_SDA, HIGH);
    digitalWrite(I2C_SCL, HIGH);
    delay(5);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
}

// ═══════════════════════════════════════════════════════════════
//  I2C SCANNER  (แสดงผลตอน boot เพื่อ debug)
// ═══════════════════════════════════════════════════════════════
static void i2cScan() {
    Serial.println("[I2C] Scanning...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X", addr);
            if (addr==0x68||addr==0x69) Serial.print(" → MPU6050 / AMG8833");
            if (addr==0x76||addr==0x77) Serial.print(" → BMP280");
            if (addr==0x40)             Serial.print(" → INA219");
            Serial.println();
            found++;
        }
    }
    if (found == 0) Serial.println("  No devices found — check Vext & wiring");
    Serial.printf("[I2C] %d device(s) found\n", found);
}

// ═══════════════════════════════════════════════════════════════
//  AMG8833
// ═══════════════════════════════════════════════════════════════
static Adafruit_AMG88xx _amg;

bool initAMG() {
    if (!_amg.begin(0x69)) return false;
    delay(100);
    return true;
}

void readAMG() {
    _amg.readPixels(amg_data.pixels);
    float mn=1e6f, mx=-1e6f, sum=0.0f;
    uint8_t hot=0;
    for (uint8_t i=0; i<64; i++) {
        float v = amg_data.pixels[i];
        if (v < mn) mn = v;
        if (v > mx) { mx = v; hot = i; }
        sum += v;
    }
    amg_data.min_t=mn; amg_data.max_t=mx;
    amg_data.avg_t=sum/64.0f; amg_data.hot_idx=hot;
}

// ═══════════════════════════════════════════════════════════════
//  BMP280
// ═══════════════════════════════════════════════════════════════
static Adafruit_BMP280 _bmp;

bool initBMP() {
    if (!_bmp.begin(0x76) && !_bmp.begin(0x77)) return false;
    _bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);
    return true;
}

void readBMP() {
    bmp_data.temp     = _bmp.readTemperature();
    bmp_data.pressure = _bmp.readPressure() / 100.0f;
    bmp_data.altitude = _bmp.readAltitude(sea_level_hpa);
}

void calibrateBMP(uint16_t samples=50) {
    if (!samples) samples=1;
    float sum=0.0f;
    for (uint16_t i=0; i<samples; i++) { sum += _bmp.readPressure()/100.0f; delay(10); }
    sea_level_hpa = sum/samples;
}

// ═══════════════════════════════════════════════════════════════
//  GPS
// ═══════════════════════════════════════════════════════════════
static HardwareSerial _gpsSerial(2);
static TinyGPSPlus    _gps;

void initGPS() {
    _gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    is_GPS = true;
}

void readGPS() {
    while (_gpsSerial.available()>0) _gps.encode(_gpsSerial.read());
    if (_gps.location.isValid()) {
        gps_data.lat = _gps.location.lat();
        gps_data.lng = _gps.location.lng();
        gps_data.fix = (_gps.satellites.isValid() && _gps.satellites.value()>=3);
    } else { gps_data.fix=false; }
    gps_data.alt       = _gps.altitude.isValid()   ? _gps.altitude.meters()  : 0.0f;
    gps_data.speed_kmh = _gps.speed.isValid()      ? _gps.speed.kmph()       : 0.0f;
    gps_data.hdop      = _gps.hdop.isValid()        ? _gps.hdop.hdop()        : 99.9f;
    gps_data.sats      = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    if (_gps.time.isValid()) {
        gps_data.hour   = _gps.time.hour();
        gps_data.minute = _gps.time.minute();
        gps_data.second = _gps.time.second();
    }
}

// ═══════════════════════════════════════════════════════════════
//  INA219
// ═══════════════════════════════════════════════════════════════
static Adafruit_INA219 _ina(0x40);

static uint8_t estPct2S(float v) {
    if (v>=8.4f) return 100;
    if (v<=6.0f) return 0;
    return (uint8_t)(((v-6.0f)/2.4f)*100.0f);
}

bool initINA() {
    if (!_ina.begin()) return false;
    _ina.setCalibration_32V_2A();
    return true;
}

void readINA() {
    float bus=_ina.getBusVoltage_V(), shunt=_ina.getShuntVoltage_mV();
    ina_data.bus_v      = bus+shunt/1000.0f;
    ina_data.shunt_mv   = shunt;
    ina_data.current_ma = _ina.getCurrent_mA();
    ina_data.power_mw   = _ina.getPower_mW();
    ina_data.batt_pct   = estPct2S(ina_data.bus_v);
}

// ═══════════════════════════════════════════════════════════════
//  LoRa
// ═══════════════════════════════════════════════════════════════
static SX1262        _radio = new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, _sharedSPI);
static volatile bool _loraTxDone = true;

void IRAM_ATTR _onTxDone() { _loraTxDone = true; }

bool initLoRa(float frequency=LORA_FREQ) {
    _sharedSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    int state = _radio.begin(frequency, LORA_BW, LORA_SF, LORA_CR,
                             LORA_SYNCWORD, LORA_TX_POWER, LORA_PREAMBLE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] Init failed: %d\n", state);
        return false;
    }
    _radio.setDio2AsRfSwitch(true);
    _radio.setDio1Action(_onTxDone);
    return true;
}

void sendLoRa(const String &msg) {
    if (!_loraTxDone) return;
    _loraTxDone = false;
    _radio.finishTransmit();
    String tmp = msg;
    int state = _radio.startTransmit(tmp);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("[LoRa] TX err: %d\n", state); _loraTxDone=true; }
}

void sendLoRa(const uint8_t *buf, size_t len) {
    if (!_loraTxDone) return;
    _loraTxDone = false;
    _radio.finishTransmit();
    int state = _radio.startTransmit(const_cast<uint8_t*>(buf), len);
    if (state != RADIOLIB_ERR_NONE) { Serial.printf("[LoRa] TX err: %d\n", state); _loraTxDone=true; }
}

String receiveLoRa() { String s; return (_radio.receive(s)==RADIOLIB_ERR_NONE)?s:""; }
int   loraRSSI() { return (int)_radio.getRSSI(); }
float loraSNR()  { return _radio.getSNR(); }

// ═══════════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════════
static MPU6050 _mpu;
static float   _accelLSB = 2048.0f;
static float   _gyroLSB  = 65.5f;

bool initMPU(uint8_t accel_range, uint16_t gyro_range, uint8_t bw_hz) {
    _mpu.initialize();
    if (!_mpu.testConnection()) return false;
    switch (accel_range) {
        case 2:  _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  _accelLSB=16384.0f; break;
        case 4:  _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  _accelLSB=8192.0f;  break;
        case 8:  _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  _accelLSB=4096.0f;  break;
        case 16: _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); _accelLSB=2048.0f;  break;
        default: return false;
    }
    switch (gyro_range) {
        case 250:  _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  _gyroLSB=131.0f; break;
        case 500:  _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  _gyroLSB=65.5f;  break;
        case 1000: _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000); _gyroLSB=32.8f;  break;
        case 2000: _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); _gyroLSB=16.4f;  break;
        default: return false;
    }
    uint8_t cfg;
    if      (bw_hz>=260) cfg=0; else if (bw_hz>=184) cfg=1;
    else if (bw_hz>=94)  cfg=2; else if (bw_hz>=44)  cfg=3;
    else if (bw_hz>=21)  cfg=4; else if (bw_hz>=10)  cfg=5; else cfg=6;
    _mpu.setDLPFMode(cfg);
    return true;
}

void readMPU() {
    int16_t ax,ay,az,gx,gy,gz;
    _mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    mpu_data.ax = (ax/_accelLSB)*9.81f;
    mpu_data.ay = (ay/_accelLSB)*9.81f;
    mpu_data.az = (az/_accelLSB)*9.81f;
    mpu_data.gx = gx/_gyroLSB;
    mpu_data.gy = gy/_gyroLSB;
    mpu_data.gz = gz/_gyroLSB;
    mpu_data.temp = _mpu.getTemperature()/340.0f + 36.53f;
}

// ═══════════════════════════════════════════════════════════════
//  CAM LOGGING  (ส่งข้อมูลไปให้ ESP32-CAM เขียน SD)
// ═══════════════════════════════════════════════════════════════
static bool _camReady = false;

void camInit() {
    _camSerial.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    delay(100);
    _camSerial.println("HEADER,time_ms,phase,ax,ay,az,avg_az,a_xandy,total_a,temp_c,pressure_hpa,altitude_m,thermal_max,thermal_min,bus_v,current_ma,gps_lat,gps_lon,gps_alt,gps_sats");
    _camReady = true;
    Serial.println("[CAM] Serial link OK");
}

void camPhoto() {
    if (_camReady) _camSerial.println("P");
}

void sdLog(unsigned long time_ms, int phase,
           float ax, float ay, float az, float avg_az_v, float a_xandy_v, float total_a_v,
           float temp, float pressure, float altitude, float thermal_max, float thermal_min,
           float bus_v, float current_ma, double lat, double lon, float gps_alt, int sats) {
    if (!_camReady) return;
    _camSerial.printf("D,%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.6f,%.6f,%.2f,%d\n",
             time_ms,phase,ax,ay,az,avg_az_v,a_xandy_v,total_a_v,
             temp,pressure,altitude,thermal_max,thermal_min,
             bus_v,current_ma,lat,lon,gps_alt,sats);
}

// ═══════════════════════════════════════════════════════════════
//  FLIGHT STATE MACHINE
// ═══════════════════════════════════════════════════════════════
static float         _accelBuf[WINDOW_SIZE][3];
static int           _bufIdx=0;
static unsigned long _launchTime=0, _ascentTime=0, _landCandTime=0;
static float         _prevAlt=0.0f;
static bool          _landCandidate=false;

static void cutSensorPower() {
    digitalWrite(POWER_CUT_PIN, LOW);
    Serial.println("[POWER] Sensor rail OFF");
}

static void pushAccel(float ax, float ay, float az) {
    _accelBuf[_bufIdx][0]=ax; _accelBuf[_bufIdx][1]=ay; _accelBuf[_bufIdx][2]=az;
    _bufIdx = (_bufIdx+1) % WINDOW_SIZE;
    float sx=0, sy=0, sz=0;
    for (int i=0; i<WINDOW_SIZE; i++) { sx+=_accelBuf[i][0]; sy+=_accelBuf[i][1]; sz+=_accelBuf[i][2]; }
    avg_ax=sx/WINDOW_SIZE; avg_ay=sy/WINDOW_SIZE; avg_az=sz/WINDOW_SIZE;
    a_xandy = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay);
    total_a  = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);
}

void flightInit() {
    pinMode(POWER_CUT_PIN, OUTPUT);
    digitalWrite(POWER_CUT_PIN, HIGH);
    memset(_accelBuf, 0, sizeof(_accelBuf));
    Serial.println("[FLIGHT] State machine ready");
}

bool flightUpdate(float ax, float ay, float az, float altitude) {
    unsigned long now = millis();
    pushAccel(ax, ay, az);

    switch (flight_phase) {

        case PHASE_WAITING:
            if (avg_az <= LAUNCH_AZ_THRESHOLD) {
                flight_phase = PHASE_ASCENT;
                _launchTime  = now;
                _prevAlt     = altitude;
                Serial.println(">>> LAUNCH DETECTED <<<");
                camPhoto();
            }
            break;

        case PHASE_ASCENT:
            if ((now - _launchTime) > NORMAL_EJECT_DELAY_MS) {
                if (a_xandy >= EJECT_ANGLE_THRESHOLD || total_a < FREEFALL_THRESHOLD) {
                    Serial.printf("[ASCENT->DESCENT] a_xy=%.2f total_a=%.2f\n", a_xandy, total_a);
                    flight_phase = PHASE_DESCENT;
                    camPhoto();
                    _ascentTime  = now;
                    _prevAlt     = altitude;
                    break;
                }
            }
            if ((now - _launchTime) > EMERGENCY_EJECT_MS) {
                Serial.printf("[ASCENT->DESCENT] Failsafe t=%.1fs\n", now/1000.0f);
                flight_phase = PHASE_DESCENT;
                _ascentTime  = now;
                _prevAlt     = altitude;
            }
            break;

        case PHASE_DESCENT: {
            bool altStable   = (fabsf(altitude - _prevAlt) < LAND_ALT_DELTA_M);
            bool accelStatic = (total_a >= LAND_ACCEL_LO && total_a <= LAND_ACCEL_HI);
            if (altStable && accelStatic) {
                if (!_landCandidate) {
                    _landCandidate = true;
                    _landCandTime  = now;
                    Serial.println("[LAND?] Candidate...");
                } else if ((now - _landCandTime) >= LAND_CONFIRM_MS) {
                    Serial.println(">>> LANDED — cutting power <<<");
                    flight_phase = PHASE_LANDED;
                    cutSensorPower();
                    return true;
                }
            } else {
                if (_landCandidate) Serial.println("[LAND?] Reset");
                _landCandidate = false;
            }
            if ((now - _ascentTime) > LAND_FAILSAFE_MS) {
                Serial.println("[LAND] Failsafe — cutting power");
                flight_phase = PHASE_LANDED;
                cutSensorPower();
                return true;
            }
            _prevAlt = altitude;
            break;
        }

        case PHASE_LANDED:
            break;
    }

    if (flight_phase != PHASE_LANDED) _prevAlt = altitude;
    return false;
}

const char* flightPhaseName() {
    switch (flight_phase) {
        case PHASE_WAITING: return "READY";
        case PHASE_ASCENT:  return "ASCENT";
        case PHASE_DESCENT: return "DESCENT";
        case PHASE_LANDED:  return "LANDED";
        default:            return "BOOT";
    }
}

// ═══════════════════════════════════════════════════════════════
//  TELEMETRY
// ═══════════════════════════════════════════════════════════════
static uint32_t packet_count = 0;

static String buildTelemetry() {
    String s; s.reserve(240);
    s+=TEAM_ID;                              s+=",";
    s+=String(millis()/1000.0f,1);           s+=",";
    s+=packet_count;                         s+=",";
    s+="F";                                  s+=",";
    s+=flightPhaseName();                    s+=",";
    s+=is_BMP?String(bmp_data.altitude,1):"0"; s+=",";
    s+=is_BMP?String(bmp_data.temp,1):"0";     s+=",";
    s+=is_BMP?String(bmp_data.pressure,1):"0"; s+=",";
    s+=is_INA?String(ina_data.bus_v,2):"0";      s+=",";
    s+=is_INA?String(ina_data.current_ma,0):"0"; s+=",";
    s+=is_INA?String(ina_data.batt_pct):"0";     s+=",";
    s+=(is_GPS&&gps_data.fix)?String(gps_data.lat,6):"0"; s+=",";
    s+=(is_GPS&&gps_data.fix)?String(gps_data.lng,6):"0"; s+=",";
    s+=is_GPS?String(gps_data.alt,1):"0";  s+=",";
    s+=is_GPS?String(gps_data.sats):"0";   s+=",";
    s+=is_MPU?String(mpu_data.ax,2):"0";   s+=",";
    s+=is_MPU?String(mpu_data.ay,2):"0";   s+=",";
    s+=is_MPU?String(mpu_data.az,2):"0";   s+=",";
    s+=is_MPU?String(mpu_data.gx,1):"0";   s+=",";
    s+=is_MPU?String(mpu_data.gy,1):"0";   s+=",";
    s+=is_MPU?String(mpu_data.gz,1):"0";   s+=",";
    s+=is_MPU?String(mpu_data.temp,1):"0"; s+=",";
    s+=is_AMG?String(amg_data.min_t,1):"0"; s+=",";
    s+=is_AMG?String(amg_data.max_t,1):"0"; s+=",";
    s+=is_AMG?String(amg_data.avg_t,1):"0"; s+=",";
    s+=String(a_xandy,2); s+=",";
    s+=String(avg_az,2);  s+=",";
    s+=String(total_a,2); s+=",";
    s+=is_LoRa?String(loraRSSI()):"0";
    return s;
}

// ═══════════════════════════════════════════════════════════════
//  DEBUG PRINT
// ═══════════════════════════════════════════════════════════════
static void printDebug() {
    Serial.printf("\n---- PKT #%lu  t=%.1fs  [%s] ----\n",
                  packet_count, millis()/1000.0f, flightPhaseName());

    if (is_BMP)
        Serial.printf("  [BMP] alt=%.1f m   temp=%.1f C   pres=%.1f hPa\n",
                      bmp_data.altitude, bmp_data.temp, bmp_data.pressure);
    else
        Serial.println("  [BMP] --");

    if (is_MPU)
        Serial.printf("  [MPU] ax=%.2f  ay=%.2f  az=%.2f m/s2  |  gx=%.1f  gy=%.1f  gz=%.1f deg/s  |  temp=%.1f C\n",
                      mpu_data.ax, mpu_data.ay, mpu_data.az,
                      mpu_data.gx, mpu_data.gy, mpu_data.gz, mpu_data.temp);
    else
        Serial.println("  [MPU] --");

    if (is_INA)
        Serial.printf("  [INA] %.2f V   %.0f mA   %.0f mW   batt=%d%%\n",
                      ina_data.bus_v, ina_data.current_ma, ina_data.power_mw, ina_data.batt_pct);
    else
        Serial.println("  [INA] --");

    if (is_GPS)
        Serial.printf("  [GPS] fix=%s   lat=%.6f   lon=%.6f   alt=%.1f m   sats=%d   hdop=%.1f\n",
                      gps_data.fix ? "YES" : "NO",
                      gps_data.lat, gps_data.lng, gps_data.alt,
                      gps_data.sats, gps_data.hdop);
    else
        Serial.println("  [GPS] --");

    if (is_AMG)
        Serial.printf("  [AMG] min=%.1f C   max=%.1f C   avg=%.1f C   hotpx=%d\n",
                      amg_data.min_t, amg_data.max_t, amg_data.avg_t, amg_data.hot_idx);
    else
        Serial.println("  [AMG] --");

    Serial.printf("  [FLT] a_xy=%.2f   avg_az=%.2f   total_a=%.2f m/s2\n",
                  a_xandy, avg_az, total_a);

    if (is_LoRa)
        Serial.printf("  [RF]  RSSI=%d dBm   SNR=%.1f dB\n", loraRSSI(), loraSNR());

    Serial.printf("  [TX]  %s\n", buildTelemetry().c_str());
}

// ═══════════════════════════════════════════════════════════════
//  BEACON
// ═══════════════════════════════════════════════════════════════
static void sendBeacon() {
    if (!is_LoRa) return;
    String b = String(TEAM_ID) + ",LANDED,";
    b += (is_GPS&&gps_data.fix) ? String(gps_data.lat,6) : "0"; b += ",";
    b += (is_GPS&&gps_data.fix) ? String(gps_data.lng,6) : "0"; b += ",";
    b += String(gps_data.alt,1);
    sendLoRa(b);
    Serial.printf("[BEACON] lat=%.5f lon=%.5f alt=%.1fm sats=%d\n",
                  gps_data.lat, gps_data.lng, gps_data.alt, gps_data.sats);
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);   // Vext ON — power sensors
    delay(500);                    // รอไฟ sensor เสถียร

    Serial.begin(115200);
    delay(3000);

    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);        // OLED RST ต้อง HIGH
    Wire.end();
    delay(20);
    pinMode(I2C_SDA, INPUT_PULLUP);  // เปิด internal pull-up แทน resistor ภายนอก
    pinMode(I2C_SCL, INPUT_PULLUP);
    delay(10);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(10000);          // 10 kHz — ช้าพอสำหรับ internal pull-up
    delay(50);
    i2cScan();                     // แสดง I2C address ที่พบตอน boot

    if (initLoRa())         { is_LoRa=true; Serial.println("[LoRa] OK"); }
    else                      Serial.println("[LoRa] FAIL");

    if (initMPU(16,500,94)) { is_MPU=true;  Serial.println("[MPU] OK"); }
    else                    { Serial.println("[MPU] FAIL"); i2cRecover(); }

    if (initBMP()) {
        is_BMP=true;
        Serial.print("[BMP] OK — calibrating... ");
        calibrateBMP(50);
        Serial.printf("P0=%.2f hPa\n", sea_level_hpa);
    } else          { Serial.println("[BMP] FAIL"); i2cRecover(); }

    initGPS();
    Serial.println("[GPS] OK");

    for (int i = 0; i < 3 && !is_INA; i++) {
        if (initINA()) { is_INA=true; Serial.println("[INA] OK"); }
        else           { delay(200); i2cRecover(); }
    }
    if (!is_INA) Serial.println("[INA] FAIL (optional)");

    delay(500);  // AMG8833 ต้องรอไฟเสถียรก่อน
    for (int i = 0; i < 3 && !is_AMG; i++) {
        if (initAMG()) { is_AMG=true; Serial.println("[AMG] OK"); }
        else           { delay(200); i2cRecover(); }
    }
    if (!is_AMG) Serial.println("[AMG] FAIL (optional)");

    camInit();
    flightInit();

    Serial.println("\n=== CanSat Skyline ready — waiting for launch ===\n");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
    static unsigned long prevTime=0, lastBeacon=0;
    unsigned long now = millis();

    if (is_GPS) readGPS();

    if (flight_phase == PHASE_LANDED) {
        if ((now-lastBeacon) >= BEACON_INTERVAL) { lastBeacon=now; sendBeacon(); }
        return;
    }

    if (now-prevTime < TX_INTERVAL_MS) return;
    prevTime = now;

    if (is_MPU) readMPU();
    if (is_BMP) readBMP();
    if (is_INA) readINA();
    if (is_AMG) readAMG();

    float altitude  = is_BMP ? bmp_data.altitude : 0.0f;
    bool justLanded = flightUpdate(mpu_data.ax, mpu_data.ay, mpu_data.az, altitude);

    if (justLanded) {
        sdLog(now,(int)flight_phase,
              mpu_data.ax,mpu_data.ay,mpu_data.az,avg_az,a_xandy,total_a,
              bmp_data.temp,bmp_data.pressure,bmp_data.altitude,
              amg_data.max_t,amg_data.min_t,ina_data.bus_v,ina_data.current_ma,
              gps_data.lat,gps_data.lng,gps_data.alt,gps_data.sats);
        lastBeacon = now;
        return;
    }

    packet_count++;
    printDebug();
    if (is_LoRa) sendLoRa(buildTelemetry());

    sdLog(now,(int)flight_phase,
          mpu_data.ax,mpu_data.ay,mpu_data.az,avg_az,a_xandy,total_a,
          bmp_data.temp,bmp_data.pressure,bmp_data.altitude,
          amg_data.max_t,amg_data.min_t,ina_data.bus_v,ina_data.current_ma,
          gps_data.lat,gps_data.lng,gps_data.alt,gps_data.sats);
}