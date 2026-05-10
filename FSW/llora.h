// llora.h
// SX1276/RFM95 LoRa radio (point-to-point, NOT LoRaWAN)
// For Heltec ESP32 WiFi LoRa V2 (onboard SX1276 @ 923 MHz)

#ifndef LLORA_H
#define LLORA_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>     // by Sandeep Mistry

// ----- Heltec ESP32 WiFi LoRa V2 onboard SX1276 pins -----
// (For an external RFM95 on a breadboard, change these to your wiring.)
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26

// ----- Frequency (Hz) -----
// AS923 / Thailand                      -> 923E6
// EU868                                 -> 868E6
// US915                                 -> 915E6
#define Fq          923E6

// ----- Radio params (tune for range vs throughput) -----
// SF7..SF12 (higher = more range, slower)
// BW 125E3 / 250E3 / 500E3
// CR 5..8 (4/5..4/8)
// TX power 2..20 dBm (PA_BOOST allowed up to 20 on SX1276)
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_SYNCWORD  0x12
#define LORA_TX_POWER  17

extern bool is_LoRa;

bool initLoRa(long frequency);

// Send: string version is convenient for CSV telemetry
void sendLoRa(const String &msg);
void sendLoRa(const uint8_t *buf, size_t len);

// Receive (call from loop on ground station). Returns "" if no packet.
String receiveLoRa();

int  loraRSSI();
float loraSNR();

#endif
