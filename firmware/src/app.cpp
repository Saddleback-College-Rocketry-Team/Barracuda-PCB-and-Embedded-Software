#include "main.h"
#include "app.h"
#include "baro.hpp"

extern SPI_HandleTypeDef hspi1;  // created by CubeIDE in main.c

static MS5611 baro(&hspi1, BARO_CS_GPIO_Port, BARO_CS_Pin);

// Globals so you can watch them in the debugger (Live Expressions)
volatile float pressure_mbar = 0;
volatile float temperature_c = 0;
volatile float altitude_m = 0;

void app_setup(void) {
  if (!baro.init()) {
    while (1) {}  // sensor not answering: check wiring, CS pin, PS tied to GND
  }
  baro.setSeaLevelPressure(baro.getPressure());  // zero altitude on the pad
}

void app_loop(void) {
  baro.readPressureAndTemperature();
  pressure_mbar = baro.getPressure();
  temperature_c = baro.getTemperature();
  altitude_m    = baro.getAltitude();
}
