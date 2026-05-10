// llora.cpp
#include "llora.h"

bool is_LoRa = false;

bool initLoRa(long frequency) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(frequency)) {
        return false;
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNCWORD);
    LoRa.setTxPower(LORA_TX_POWER);   // PA_BOOST is the default for Heltec
    LoRa.enableCrc();

    return true;
}

void sendLoRa(const String &msg) {
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();        // blocking; pass true for async if needed
}

void sendLoRa(const uint8_t *buf, size_t len) {
    LoRa.beginPacket();
    LoRa.write(buf, len);
    LoRa.endPacket();
}

String receiveLoRa() {
    String s = "";
    int sz = LoRa.parsePacket();
    if (sz > 0) {
        while (LoRa.available()) {
            s += (char)LoRa.read();
        }
    }
    return s;
}

int   loraRSSI() { return LoRa.packetRssi(); }
float loraSNR()  { return LoRa.packetSnr();  }
