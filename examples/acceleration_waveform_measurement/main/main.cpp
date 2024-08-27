#include "pl_adxl355.h"

//==============================================================================

const spi_host_device_t host = HSPI_HOST;
const int mosiPin = 5;
const int misoPin = 18;
const int sclkPin = 19;
const int sclkFrequency = 10000000;
const int csPin = 21;
const auto range = PL::Adxl355_Range::range2g;
const auto outputDataRate = PL::Adxl355_OutputDataRate::odr4000;
const float timeStepMs = 1.0 / 4000.0 * 1000;
const int numberOfPoints = 10;

//==============================================================================

extern "C" void app_main(void) {
  auto spi = std::make_shared<PL::Spi>(host, mosiPin, misoPin, sclkPin);
  PL::Adxl355 adxl355(spi, sclkFrequency, csPin);
  adxl355.Initialize();

  // Set ADXL355 range
  adxl355.SetRange(range);
  // Set ADXL355 output data rate
  adxl355.SetOutputDataRate(outputDataRate);
  // Enable ADXL355 measurement
  adxl355.EnableMeasurement();

  while (1) {
    // Clear the FIFO
    adxl355.ClearFifo();
    printf("FIFO cleared\n");

    vTaskDelay(1);

    for (int i = 0; i < numberOfPoints; i++) {
      // Read and print the accelerations from the FIFO
      PL::Adxl355_Accelerations accelerations;
      adxl355.ReadAccelerationsFromFifo(accelerations);
      printf("Time: %f ms, Accelerations: X: %f g, Y: %f g, Z: %f g\n", i * timeStepMs, accelerations.x, accelerations.y, accelerations.z);
    }
    printf("\n");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}