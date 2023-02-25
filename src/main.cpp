#include <Arduino.h>

const unsigned char dataPin = 32;
const uint32_t TIMEOUT = UINT32_MAX;
uint32_t MAX_WAIT_TIME = 1000U; // 1000us -> 1ms

#define DHT_DEBUG 0

#if (DHT_DEBUG == 1)
#define DHT_PRINT_DEBUG(...) Serial.printf(__VA_ARGS__);
#else
#define DHT_PRINT_DEBUG
#endif

void setup()
{
  Serial.begin(115200U);
}

uint32_t ExpectLevel(uint8_t level)
{
  uint32_t start = micros();
  while ((digitalRead(dataPin) != level))
  {
    if (micros() >= (start + MAX_WAIT_TIME))
    {
      return TIMEOUT;
    }
  }
  return micros() - start;
}

void SendStartSignal()
{
  // Send start signal - set LOW for ~18ms
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, LOW);
  delay(18U);
  digitalWrite(dataPin, HIGH);
  pinMode(dataPin, INPUT);
}

bool WaitForStartSending()
{
  // Wait for response signal
  if (ExpectLevel(LOW) == TIMEOUT)
  {
    DHT_PRINT_DEBUG("Response signal (LOW) not arrived.\n");
    return false;
  }

  // Wait for pull up voltage
  if (ExpectLevel(HIGH) == TIMEOUT)
  {
    DHT_PRINT_DEBUG("Pull up not arrived.\n");
    return false;
  }

  // Wait for start
  if (ExpectLevel(LOW) == TIMEOUT)
  {
    DHT_PRINT_DEBUG("Communication not started.\n");
    return false;
  }

  return true;
}

bool GetData(uint8_t data[5U])
{
  // Read 80 bit of data
  // 40 tuples of bits
  //   - first bit is start (~50us)
  //   - second bit is data
  //     - logical 0 (~26-28us)
  //     - logical 1 (~70us)
  uint8_t rawData[80U];
  for (uint8_t i = 0U; i < 80U; i += 2)
  {
    // Measure start and data length
    const uint8_t startBitLength = ExpectLevel(HIGH);
    const uint8_t dataBitLength = ExpectLevel(LOW);

    rawData[i] = startBitLength;
    rawData[i + 1] = dataBitLength;
  }

  // Get 40bit of data
  uint8_t commonBitIndex = 0U;
  for (uint8_t i = 0U; i < 5U; ++i)
  {
    uint32_t actualByte = 0U;
    const uint8_t bitIndexEnd = commonBitIndex + (2U * 8U);
    for (/*commonBitIndex define before*/; commonBitIndex < bitIndexEnd; commonBitIndex += 2U)
    {
      const uint8_t startBitLength = rawData[commonBitIndex];
      const uint8_t dataBitLength = rawData[commonBitIndex + 1U];

      if (startBitLength == TIMEOUT)
      {
        DHT_PRINT_DEBUG("Start bit timeout.\n");
        return false;
      }

      if (dataBitLength == TIMEOUT)
      {
        DHT_PRINT_DEBUG("Data bit timeout.\n");
        return false;
      }

      actualByte <<= 1U;
      if (dataBitLength > startBitLength)
      {
        actualByte |= 1U;
      }
    }

    data[i] = actualByte;
  }

  return true;
}

void PrintReceivedData(uint8_t data[5U])
{
  // Print unprocessed data
  DHT_PRINT_DEBUG("Recieved data:\n");
  for (uint8_t i = 0U; i < 5U; i++)
  {
    DHT_PRINT_DEBUG("[%i] %i\n", i, data[i]);
  }
}

bool EvaluateChecksum(uint8_t data[5U])
{
  // Evaluate checksum
  uint16_t fullChecksum = 0U;
  for (uint8_t i = 0U; i < 4U; i++)
  {
    fullChecksum += static_cast<uint16_t>(data[i]);
  }

  if (static_cast<uint8_t>(fullChecksum) != data[4U])
  {
    DHT_PRINT_DEBUG("Invalid checksum.\n");
    return false;
  }

  return true;
}

void ProcessData(uint8_t data[5U], float &outHumidity, float &outTemperature)
{
  // Process humidity data
  float humidity = data[0] + (data[1] * 0.1F);

  // Process temperature data
  float temperature = data[2] + ((data[3] & 0x7F) * 0.1F);
  if (data[3] & 0x80)
  {
    temperature *= -1.0F;
  }

  // Set output
  outHumidity = humidity;
  outTemperature = temperature;
}

bool GetHumidityAndTemperature(float &outHumidity, float &outTemperature)
{
  uint8_t data[5U];
  
  SendStartSignal();

  if (!WaitForStartSending())
  {
    return false;
  }

  if (!GetData(data))
  {
    return false;
  }

  PrintReceivedData(data);

  if (!EvaluateChecksum(data))
  {
    return false;
  }

  ProcessData(data, outHumidity, outTemperature);

  return true;
}

void loop()
{
  float humidity, temperature;
  bool areDataOk = GetHumidityAndTemperature(humidity, temperature);

  if (areDataOk)
  {
    Serial.printf("--- [%i] ---\n", millis() / 1000);
    Serial.printf("Humidity: %.1f%%\n", humidity);
    Serial.printf("Temperature: %.1f°C\n", temperature);
  }

  delay(1000);
}