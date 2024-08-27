#include "pl_adxl355.h"

//==============================================================================

const spi_host_device_t host = HSPI_HOST;
const int mosiPin = 5;
const int misoPin = 18;
const int sclkPin = 19;
const int sclkFrequency = 10000000;
const int csPin = 21;

//==============================================================================

extern "C" void app_main(void) {
  auto spi = std::make_shared<PL::Spi>(host, mosiPin, misoPin, sclkPin);
  PL::Adxl355 adxl355(spi, sclkFrequency, csPin);
  adxl355.Initialize();

  // Reset the ADXL355
  adxl355.Reset();

  // Read and print the device info
  PL::Adxl355_DeviceInfo deviceInfo;
  adxl355.ReadDeviceInfo(deviceInfo);
  printf("Vendor ID (should be 0xAD): 0x%02X\n", deviceInfo.vendorId);
  printf("Family ID (should be 0x1D): 0x%02X\n", deviceInfo.familyId);
  printf("Device ID (should be 0xED): 0x%02X\n", deviceInfo.deviceId);
  printf("Revision ID: 0x%02X\n\n", deviceInfo.revisionId);

  // Execute the self-test and print the results
  PL::Adxl355_Accelerations accelerations;
  adxl355.SelfTest(accelerations);
  printf ("Self test (should be 0.1...0.6 g, 0.1...0.6 g, 0.5...3.0 g): %f, %f, %f\n", accelerations.x, accelerations.y, accelerations.z);
}