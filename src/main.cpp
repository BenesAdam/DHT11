#include <Arduino.h>
#include "DHT11.hpp"

const unsigned char dataPin = 32;
cDHT11 dht11(dataPin);

void setup()
{
  Serial.begin(115200U);
}

void loop()
{
  dht11.LoadData();

  if (dht11.AreDataOkey())
  {
    Serial.printf("--- [%i] ---\n", millis() / 1000);
    Serial.printf("Humidity: %.1f%%\n", dht11.GetHumidity());
    Serial.printf("Temperature: %.1f°C\n", dht11.GetTemperature());
  }

  delay(1000);
}