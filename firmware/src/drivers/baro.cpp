#include "baro.hpp"
#include <cmath>
#include <cstring>

constexpr uint8_t CMD_RESET     = 0x1E;
constexpr uint8_t CMD_ADC_READ  = 0x00;
constexpr uint8_t CMD_PROM_READ = 0xA0;
constexpr uint8_t CMD_D2_OFFSET = 0x10;  // D2 command = D1 command + 0x10

MS5611::MS5611(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : hspi(hspi), csPort(csPort), csPin(csPin) {}


void MS5611::sendCommand(uint8_t cmd) {
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, &cmd, 1, 10);
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

uint16_t MS5611::readProm(uint8_t address) {
  uint8_t tx[3] = {uint8_t(CMD_PROM_READ | (address << 1)), 0, 0};
  uint8_t rx[3] = {};
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, tx, rx, 3, 10);
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
  return uint16_t((rx[1] << 8) | rx[2]);
}

uint32_t MS5611::readAdc() {
  uint8_t tx[4] = {CMD_ADC_READ, 0, 0, 0};
  uint8_t rx[4] = {};
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi, tx, rx, 4, 10);
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
  return (uint32_t(rx[1]) << 16) | (uint32_t(rx[2]) << 8) | rx[3];
}

// Max conversion times, datasheet pg 3: 0.60 / 1.17 / 2.28 / 4.54 / 9.04 ms
uint32_t MS5611::conversionTimeMs() {
  switch (osr) {
    case OSR::OSR_256:  return 1;
    case OSR::OSR_512:  return 2;
    case OSR::OSR_1024: return 3;
    case OSR::OSR_2048: return 5;
    case OSR::OSR_4096: return 10;
  }
  return 10;
}

// ---------- Public ----------

bool MS5611::init() {
  HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
  sendCommand(CMD_RESET);
  HAL_Delay(3);  // PROM reload after reset takes 2.8 ms

  C1_pressureSens           = readProm(1);
  C2_pressureOffset         = readProm(2);
  C3_tempCoefPressureSens   = readProm(3);
  C4_tempCoefPressureOffset = readProm(4);
  C5_refTemp                = readProm(5);
  C6_tempCoefTemp           = readProm(6);


  if (C1_pressureSens == 0 || C1_pressureSens == 0xFFFF) return false;

  readPressureAndTemperature();
  return true;
}

void MS5611::readPressureAndTemperature() {
  sendCommand(static_cast<uint8_t>(osr));                  // convert D1
  HAL_Delay(conversionTimeMs());
  D1_rawPressure = readAdc();

  sendCommand(static_cast<uint8_t>(osr) + CMD_D2_OFFSET);  // convert D2
  HAL_Delay(conversionTimeMs());
  D2_rawTemperature = readAdc();

  // Temperature, datasheet pg 8
  pressureDiff = int32_t(D2_rawTemperature) - int32_t(C5_refTemp) * 256;               // dT = D2 - C5 * 2^8
  actualTemp   = 2000 + int32_t(int64_t(pressureDiff) * C6_tempCoefTemp / 8388608);    // TEMP = 2000 + dT * C6 / 2^23

  actualPressure = tempCompPressure();

  // Second order temperature correction below 20 C, datasheet pg 9
  if (actualTemp < 2000) {
    actualTemp -= int32_t(int64_t(pressureDiff) * pressureDiff / 2147483648LL);        // T2 = dT^2 / 2^31
  }
}

float MS5611::getPressure()    { return actualPressure / 100.0f; }
float MS5611::getTemperature() { return actualTemp / 100.0f; }

float MS5611::getAltitude() {
  return 44330.0f * (1.0f - powf(getPressure() / seaLevelPressure, 0.190295f));
}

// ---------- Compensation, datasheet pg 8-9 ----------
// These use actualTemp BEFORE the T2 correction is applied (datasheet order).

int64_t MS5611::tempOffSet() {
  // OFF = C2 * 2^16 + (C4 * dT) / 2^7
  int64_t OFF = int64_t(C2_pressureOffset) * 65536 + int64_t(C4_tempCoefPressureOffset) * pressureDiff / 128;

  if (actualTemp < 2000) {
    int64_t t = actualTemp - 2000;
    int64_t OFF2 = 5 * t * t / 2;
    if (actualTemp < -1500) {
      int64_t u = actualTemp + 1500;
      OFF2 += 7 * u * u;
    }
    OFF -= OFF2;
  }
  return OFF;
}

int64_t MS5611::tempSense() {
  // SENS = C1 * 2^15 + (C3 * dT) / 2^8
  int64_t SENS = int64_t(C1_pressureSens) * 32768 + int64_t(C3_tempCoefPressureSens) * pressureDiff / 256;

  if (actualTemp < 2000) {
    int64_t t = actualTemp - 2000;
    int64_t SENS2 = 5 * t * t / 4;
    if (actualTemp < -1500) {
      int64_t u = actualTemp + 1500;
      SENS2 += 11 * u * u / 2;
    }
    SENS -= SENS2;
  }
  return SENS;
}

int32_t MS5611::tempCompPressure() {
  // P = (D1 * SENS / 2^21 - OFF) / 2^15
  return int32_t((int64_t(D1_rawPressure) * tempSense() / 2097152 - tempOffSet()) / 32768);
}
