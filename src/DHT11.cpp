#include "DHT11.hpp"

//********************************************************************
// Defines
//********************************************************************
#define EXIT_ON_FAIL(condition) \
  if (!(condition))             \
  {                             \
    return false;               \
  }

#define EXIT_ON_TIMEOUT(condition, ...) \
  if ((condition) == TIMEOUT)           \
  {                                     \
    DHT_PRINT_DEBUG(__VA_ARGS__)        \
    return false;                       \
  }

//********************************************************************
// Methods implementations
//********************************************************************

//********************************************************************
//
//! Constructor
//
//********************************************************************
cDHT11::cDHT11(const uint8_t arg_dataPin) : dataPin(arg_dataPin),
                                            areDataOkey(false),
                                            humidity(0.0F),
                                            temperature(0.0F)
{
}

//********************************************************************
//
//! Returns true if data are okey
//
//********************************************************************
bool cDHT11::AreDataOkey() const
{
  return areDataOkey;
}

//********************************************************************
//
//! Returns humidity percentage
//
//********************************************************************
float cDHT11::GetHumidity() const
{
  return humidity;
}

//********************************************************************
//
//! Returns temperature in °C
//
//********************************************************************
float cDHT11::GetTemperature() const
{
  return temperature;
}

//********************************************************************
//
//! Load data from DHT11 and returns true if data are loaded
//! successfuly
//
//********************************************************************
bool cDHT11::LoadData()
{
  areDataOkey = ExecuteLoadingProcedure();
  return areDataOkey;
}

//********************************************************************
//
//! Do actual data loading procedure
//
//********************************************************************
bool cDHT11::ExecuteLoadingProcedure()
{
  uint8_t rawData[cDHT11::RawDataSize];
  uint8_t data[cDHT11::DataSize];

  /*        */ SendStartSignal();
  EXIT_ON_FAIL(WaitForStartSending());
  /*        */ ReceiveRawData(rawData);
  EXIT_ON_FAIL(ProcessRawData(rawData, data));
  /*        */ PrintReceivedData(data);
  EXIT_ON_FAIL(EvaluateChecksum(data));
  /*        */ ProcessData(data);

  return true;
}

//********************************************************************
//
//! Send start signal by pull signal down for ~18ms
//
//********************************************************************
void cDHT11::SendStartSignal()
{
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, LOW);
  delay(18U);
  digitalWrite(dataPin, HIGH);
  pinMode(dataPin, INPUT);
}

//********************************************************************
//
//! What for signal to go:
//!   1. LOW -> response signal from DHT11 arrived
//!   2. HIGH -> DHT11 pulled up signal
//!   3. LOW -> DHT11 starts sending data
//
//********************************************************************
bool cDHT11::WaitForStartSending()
{
  EXIT_ON_TIMEOUT(ExpectLevel(LOW), "Response signal (LOW) not arrived.\n");
  EXIT_ON_TIMEOUT(ExpectLevel(HIGH), "Pull up from DHT11 not presented.\n");
  EXIT_ON_TIMEOUT(ExpectLevel(LOW), "Communication not started.\n");

  return true;
}

//********************************************************************
//
//! Wait for expected level and return time in us or timeout flag
//
//********************************************************************
uint32_t cDHT11::ExpectLevel(const uint8_t level)
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

//********************************************************************
//
//! Receive raw data. Raw data are consist of 40 pair of length of
//! signals. First signal is start bit (~50us) and second signals is
//! data bit where logical 0 is represented by lenght of ~26-28us in
//! level LOW and logical 1 is represented by length of ~70us in
//! level HIGH
//
//********************************************************************
void cDHT11::ReceiveRawData(uint8_t rawData[cDHT11::RawDataSize])
{
  for (uint8_t i = 0U; i < cDHT11::RawDataSize; i += 2)
  {
    rawData[i] = ExpectLevel(HIGH);
    rawData[i + 1] = ExpectLevel(LOW);
  }
}

//********************************************************************
//
//! Generate 40bits (5 bytes) of data represents:
//!   1. byte: humidity integral part
//!   2. byte: humidity decimal part
//!   3. byte: temperature integral part
//!   4. byte: temperature decimal part (first bit is sign bit)
//!   5. byte: checksum (last byte of sum of all previous bytes)
//
//********************************************************************
bool cDHT11::ProcessRawData(uint8_t rawData[cDHT11::RawDataSize], uint8_t data[cDHT11::DataSize])
{
  uint8_t bitIndex = 0U;

  // For every byte
  for (uint8_t byteIndex = 0U; byteIndex < cDHT11::DataSize; byteIndex++)
  {
    uint32_t actualByte = 0U;
    const uint8_t bitIndexEnd = bitIndex + (2U * 8U);

    // For next 16 bits of raw data
    for (/*bitIndex*/; bitIndex < bitIndexEnd; bitIndex += 2U)
    {
      // Get bits
      const uint8_t startBitLength = rawData[bitIndex];
      const uint8_t dataBitLength = rawData[bitIndex + 1U];

      // Check bits
      EXIT_ON_TIMEOUT(startBitLength, "Start bit timeout.\n");
      EXIT_ON_TIMEOUT(dataBitLength, "Data bit timeout.\n");

      // Append bit to actual byte
      actualByte <<= 1U;
      if (dataBitLength > startBitLength)
      {
        actualByte |= 1U;
      }
    }

    data[byteIndex] = actualByte;
  }

  return true;
}

//********************************************************************
//
//! Debug printing of received data
//
//********************************************************************
void cDHT11::PrintReceivedData(uint8_t data[cDHT11::DataSize])
{
  DHT_PRINT_DEBUG("Recieved data:\n");
  for (uint8_t i = 0U; i < cDHT11::DataSize; i++)
  {
    DHT_PRINT_DEBUG("[%i] %i\n", i, data[i]);
  }
}

//********************************************************************
//
//! Calculate checksum from first 4 bytes and compare it with
//! reveived checksum
//
//********************************************************************
bool cDHT11::EvaluateChecksum(uint8_t data[cDHT11::DataSize])
{
  uint16_t fullChecksum = 0U;
  for (uint8_t i = 0U; i < cDHT11::DataSize - 1U; i++)
  {
    fullChecksum += static_cast<uint16_t>(data[i]);
  }

  if (static_cast<uint8_t>(fullChecksum) != data[CHECKSUM])
  {
    DHT_PRINT_DEBUG("Invalid checksum.\n");
    return false;
  }

  return true;
}

//********************************************************************
//
//! Calculate and store humidity and temperature from received data
//
//********************************************************************
void cDHT11::ProcessData(uint8_t data[cDHT11::DataSize])
{
  // Process humidity data
  humidity = data[HUMIDITY_INTEGRAL] + (data[HUMIDITY_DECIMAL] * 0.1F);

  // Process temperature data
  temperature = data[TEMPERATURE_INTEGRAL] + ((data[TEMPERATURE_DECIMAL] & 0x7F) * 0.1F);
  if (data[TEMPERATURE_DECIMAL] & 0x80)
  {
    temperature *= -1.0F;
  }
}
