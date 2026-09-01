#pragma once
#include <cstdint>

Class MS5611 {
public:
  

private:
  int spiChannel = 0; // change once we assign pin
  int csPin = 0; // Change once we assign a pin

  enum class OSR: uint8_t { // found from command data sheet OSR (8 bits per) pg 10
        OSR_256  = 0x40,
        OSR_512  = 0x42,
        OSR_1024 = 0x44,
        OSR_2048 = 0x46,
        OSR_4096 = 0x48
  };

  // factory calibration coefficients read from PROM addresses 1-6.

  uint16_t C1_pressureSens = 0;            //SENS
  uint16_t C2_pressureOffset = 0;          //OFF
  uint16_t C3_tempCoefPressureSens = 0;    //TCS
  uint16_t C4_tempCoefPressureOffset = 0;  //TCO
  uint16_t C5_refTemp = 0;                 //T_ref
  uint16_t C6_tempCoefTemp = 0;            //TEMPSENS


  uint32_t D1_rawPressure = 0;             // D1
  uint32_t D2_rawTemperature = 0;          // D2

  float pressureDiff = 0.0f;               // dT
  float actualTemp = 0.0f;                 // Temp

  int64_t tempOffSet();                    // OFF
  int64_t tempSense();                     // SENS
  int32_t tempCompPressure();              // P
  
};
