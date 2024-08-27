#include "pl_adxl355.h"

//==============================================================================

const spi_host_device_t host = HSPI_HOST;
const int mosiPin = 5;
const int misoPin = 18;
const int sclkPin = 19;
const int sclkFrequency = 10000000;
const int csPin = 21;
const auto range = PL::Adxl355_Range::range2g;
auto activityAxes = PL::Adxl355_Axes::x | PL::Adxl355_Axes::y;
float activityThreshold = 1.5;
uint8_t activityCount = 5;

//==============================================================================

extern "C" void app_main(void) {
  auto spi = std::make_shared<PL::Spi>(host, mosiPin, misoPin, sclkPin);
  PL::Adxl355 adxl355(spi, sclkFrequency, csPin);
  adxl355.Initialize();

  // Set ADXL355 range
  adxl355.SetRange(range);
  // Set activity detection axes
  adxl355.SetActivityDetectionAxes(activityAxes);
  // Set activity detection threshold 
  adxl355.SetActivityDetectionThreshold(activityThreshold);
  // Set activity detection count
  adxl355.SetActivityDetectionCount(activityCount);
  // Enable ADXL355 measurement
  adxl355.EnableMeasurement();

  while (1) {
    PL::Adxl355_Status status;
    adxl355.ReadStatus(status);
    printf((bool)(status & PL::Adxl355_Status::activity) ? "Activity detected\n" : "No activity detected\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}