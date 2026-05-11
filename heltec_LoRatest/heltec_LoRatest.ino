#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  26

long Fq = 923E6;

void initLoRa(long Fq = 923E6) {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(Fq)) {
    Serial.println("LoRa Not Found!!!");
    while (true);
  }

  LoRa.setSignalBandwidth(125E3);  // must match sender: 125 kHz
  LoRa.setSpreadingFactor(7);      // must match sender: SF7
  LoRa.setSyncWord(0x12);          // must match sender: 0x12
  LoRa.enableCrc();                // must match sender: CRC on

  Serial.println("LoRa Ready — waiting for packets...");
}

String receive() {
  String incoming = "";
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    Serial.printf("[RX] %s  |  RSSI=%d dBm  SNR=%.1f dB\n",
                  incoming.c_str(),
                  LoRa.packetRssi(),
                  LoRa.packetSnr());
  }
  return incoming;
}

void setup() {
  Serial.begin(115200);
  initLoRa(Fq);
}

void loop() {
  receive();   // non-blocking, call as fast as possible — no delay
}