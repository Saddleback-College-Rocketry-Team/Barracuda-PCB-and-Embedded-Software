#pragma once
#include <cstdint>
#include "stm32h7xx_hal.h"

class MS5611 {
public:
  MS5611(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);
  bool init();

  void readPressureAndTemperature();

  float getPressure();     // mbar
  float getTemperature();  // deg C
  float getAltitude();     // m, relative to seaLevelPressure

  enum class OSR : uint8_t { // D1 convert commands, datasheet pg 10 
    OSR_256  = 0x40,
    OSR_512  = 0x42,
    OSR_1024 = 0x44,
    OSR_2048 = 0x46,
    OSR_4096 = 0x48
  };

  void setOSR(OSR osr) { this->osr = osr; }
  void setSeaLevelPressure(float mbar) { seaLevelPressure = mbar; }

private:
  SPI_HandleTypeDef* hspi;   // TODO(pins)
  GPIO_TypeDef* csPort;      // TODO(pins)
  uint16_t csPin;            // TODO(pins)
  OSR osr = OSR::OSR_4096;

  // factory calibration coefficients read from PROM addresses 1-6.
  uint16_t C1_pressureSens = 0;            // SENS_T1
  uint16_t C2_pressureOffset = 0;          // OFF_T1
  uint16_t C3_tempCoefPressureSens = 0;    // TCS
  uint16_t C4_tempCoefPressureOffset = 0;  // TCO
  uint16_t C5_refTemp = 0;                 // T_REF
  uint16_t C6_tempCoefTemp = 0;            // TEMPSENS

  uint32_t D1_rawPressure = 0;             // D1
  uint32_t D2_rawTemperature = 0;          // D2

  int32_t pressureDiff = 0;                // dT
  int32_t actualTemp = 0;                  // TEMP, 0.01 deg C (2000 = 20.00 C)
  int32_t actualPressure = 0;              // P, 0.01 mbar (100009 = 1000.09 mbar)

  float seaLevelPressure = 1013.25f;       // mbar

  int64_t tempOffSet();                    // OFF
  int64_t tempSense();                     // SENS
  int32_t tempCompPressure();              // P

  // SPI helpers
  void sendCommand(uint8_t cmd);
  uint16_t readProm(uint8_t address);
  uint32_t readAdc();
  uint32_t conversionTimeMs();
};
