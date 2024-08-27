#include "pl_adxl355.h"

//==============================================================================

const spi_host_device_t host = HSPI_HOST;
const int mosiPin = 5;
const int misoPin = 18;
const int sclkPin = 19;
const int sclkFrequency = 10000000;
const int csPin = 21;
const auto range = PL::Adxl355_Range::range2g;

//==============================================================================

extern "C" void app_main(void) {
  auto spi = std::make_shared<PL::Spi>(host, mosiPin, misoPin, sclkPin);
  PL::Adxl355 adxl355(spi, sclkFrequency, csPin);
  adxl355.Initialize();

  // Set ADXL355 range
  adxl355.SetRange(range);
  // Enable ADXL355 measurement
  adxl355.EnableMeasurement();

  while (1) {
    // Read and print the accelerations
    PL::Adxl355_Accelerations accelerations;
    adxl355.ReadAccelerations(accelerations);
    printf("Accelerations: X: %f g, Y: %f g, Z: %f g\n", accelerations.x, accelerations.y, accelerations.z);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}