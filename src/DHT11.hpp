#ifndef DHT_HPP
#define DHT_HPP

//********************************************************************
// Includes
//********************************************************************
#include <Arduino.h>

//********************************************************************
// Defines
//********************************************************************
// #define DHT_DEBUG 1

#if defined(DHT_DEBUG) && DHT_DEBUG == 1
#define DHT_PRINT_DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define DHT_PRINT_DEBUG(...)
#endif

//********************************************************************
// Class
//********************************************************************
// cDHT11 is class for DHT11 humidity and temperature sensor.
// It contains methods used for communicating with the sensor
// and processing the received data.
class cDHT11
{
public:
  //------------------------------------------------------------------
  //--- Public methods -----------------------------------------------
  //------------------------------------------------------------------
  cDHT11(const uint8_t arg_dataPin);
  bool LoadData();
  bool AreDataOkey() const;
  float GetHumidity() const;
  float GetTemperature() const;

private:
  //------------------------------------------------------------------
  //--- Type definitions ---------------------------------------------
  //------------------------------------------------------------------
  enum eData
  {
    HUMIDITY_INTEGRAL,
    HUMIDITY_DECIMAL,
    TEMPERATURE_INTEGRAL,
    TEMPERATURE_DECIMAL,
    CHECKSUM
  };

  //------------------------------------------------------------------
  //--- Constants ----------------------------------------------------
  //------------------------------------------------------------------
  const uint8_t dataPin;
  static const uint8_t DataSize = 5U;
  static const uint8_t RawDataSize = 80U;
  static const uint32_t TIMEOUT = UINT32_MAX;
  static const uint32_t MAX_WAIT_TIME = 1000U; // 1000us -> 1ms;

  //------------------------------------------------------------------
  //--- Attributes ---------------------------------------------------
  //------------------------------------------------------------------
  uint8_t data[cDHT11::DataSize];
  bool areDataOkey;
  float humidity;
  float temperature;

  //------------------------------------------------------------------
  //--- Private methods ----------------------------------------------
  //------------------------------------------------------------------
  bool ExecuteLoadProcedure();
  void SendStartSignal();
  bool WaitForStartSending();
  uint32_t ExpectLevel(const uint8_t level);
  void ReceiveRawData(uint8_t rawData[80U]);
  bool ProcessRawData(uint8_t rawData[80U]);
  void PrintReceivedData();
  bool EvaluateChecksum();
  void ProcessData();
};

#endif // DHT_HPP
