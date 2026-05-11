#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// GY-NEO-M8N wiring: GPS TX -> board RXPin, GPS RX -> board TXPin
// Adjust pins to match your wiring on Heltec WiFi LoRa V3
static const int RXPin = 45, TXPin = 46;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // UART1

void setup()
{
  Serial.begin(115200);
  GPSSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  Serial.println("GY-NEO-M8N | Heltec WiFi LoRa V3");
  Serial.println("Sats | HDOP | Latitude   | Longitude   | Alt(m) | Speed(km/h) | UTC Time");
  Serial.println("---------------------------------------------------------------------------");
}

void loop()
{
  while (GPSSerial.available() > 0)
    gps.encode(GPSSerial.read());

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000)
  {
    lastPrint = millis();

    // Satellites
    Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : -1);
    Serial.print(" | ");

    // HDOP
    if (gps.hdop.isValid())
      Serial.print(gps.hdop.hdop(), 1);
    else
      Serial.print("N/A");
    Serial.print(" | ");

    // Latitude / Longitude
    if (gps.location.isValid())
    {
      Serial.print(gps.location.lat(), 6);
      Serial.print(" | ");
      Serial.print(gps.location.lng(), 6);
    }
    else
    {
      Serial.print("N/A        | N/A");
    }
    Serial.print(" | ");

    // Altitude
    if (gps.altitude.isValid())
      Serial.print(gps.altitude.meters(), 1);
    else
      Serial.print("N/A");
    Serial.print(" | ");

    // Speed
    if (gps.speed.isValid())
      Serial.print(gps.speed.kmph(), 1);
    else
      Serial.print("N/A");
    Serial.print(" | ");

    // UTC time
    if (gps.time.isValid())
    {
      char buf[10];
      sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      Serial.print(buf);
    }
    else
    {
      Serial.print("N/A");
    }

    Serial.println();

    if (millis() > 10000 && gps.charsProcessed() < 10)
      Serial.println("ERROR: No GPS data — check TX/RX wiring and baud rate");
  }
}
