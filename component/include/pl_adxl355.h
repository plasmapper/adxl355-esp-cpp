#pragma once
#include "pl_adxl355_types.h"
#include "pl_spi.h"

//==============================================================================

namespace PL {

//==============================================================================

/// @brief ADXL355 controller class
class Adxl355 : public Lockable {
public:
  /// @brief Default SPI frequency
  static constexpr uint32_t defaultSpiFrequency = 10000000;
  /// @brief Temperature intercept, LSB
  static constexpr uint16_t temperatureInterceptLsb = 1885;
  /// @brief Temperature intercept, °C
  static constexpr float temperatureInterceptDegC = 25;
  /// @brief Temperature slope, LSB/°C
  static constexpr float temperatureSlope = -9.05;
  /// @brief Acceleration scale factor for ±2 g range, g/LSB
  static constexpr float accelerationScaleFactorRange2G = 3.9e-6;
  /// @brief Acceleration scale factor for ±4 g range, g/LSB
  static constexpr float accelerationScaleFactorRange4G = 7.8e-6;
  /// @brief Acceleration scale factor for ±8 g range, g/LSB
  static constexpr float accelerationScaleFactorRange8G = 15.6e-6;
  /// @brief Maximum number of the FIFO samples
  static constexpr uint8_t maxNumberOfFifoSamples = 96;

  /// @brief Initializes a new instance of the ADXL355 class.
  /// @param spi SPI interface
  /// @param sclkFrequency SCLK frequency in Hz
  /// @param csPin CS pin
  Adxl355(std::shared_ptr<Spi> spi, int sclkFrequency, int csPin);
  ~Adxl355();
  Adxl355(const Adxl355&) = delete;
  Adxl355& operator=(const Adxl355&) = delete;

  esp_err_t Lock(TickType_t timeout = portMAX_DELAY) override;
  
  esp_err_t Unlock() override;

  /// @brief Initializes the ADXL355 controller
  /// @return error code
  esp_err_t Initialize();

  /// @brief Reads the device information
  /// @param deviceInfo device information (should be vendorId: 0xAD, familyId: 0x1D, deviceId: 0xED)
  /// @return error code
  esp_err_t ReadDeviceInfo(Adxl355_DeviceInfo& deviceInfo);

  /// @brief Reads the device status
  /// @param status device status
  /// @return error code
  esp_err_t ReadStatus(Adxl355_Status& status);

  /// @brief Reads the number of valid data samples present in the FIFO buffer
  /// @param numberOfFifoSamples number of samples
  /// @return error code
  esp_err_t ReadNumberOfFifoSamples(uint8_t& numberOfFifoSamples);

  /// @brief Reads the raw temperature
  /// @param rawTemperature raw temperature (1885 LSB at 25°C, −9.05 LSB/°C)
  /// @return error code
  esp_err_t ReadRawTemperature(uint16_t& rawTemperature);

  /// @brief Reads the temperature
  /// @param temperature temperature, °C
  /// @return error code
  esp_err_t ReadTemperature(float& temperature);

  /// @brief Reads the raw X-, Y- and Z-axis accelerations
  /// @param rawAccelerations raw accelerations
  /// @return error code
  esp_err_t ReadRawAccelerations(Adxl355_RawAccelerations& rawAccelerations);

  /// @brief Reads the X-, Y- and Z-axis accelerations
  /// @param accelerations accelerations, g
  /// @return error code
  esp_err_t ReadAccelerations(Adxl355_Accelerations& accelerations);

  /// @brief Discards the valid 3-axis samples in the FIFO
  /// @return error code
  esp_err_t ClearFifo();

  /// @brief Reads the raw X-, Y- and Z-axis accelerations from the FIFO
  /// @param rawAccelerations raw accelerations
  /// @param timeout timeout in FreeRTOS ticks
  /// @return error code
  esp_err_t ReadRawAccelerationsFromFifo(Adxl355_RawAccelerations& rawAccelerations);
  
  /// @brief Reads the X-, Y- and Z-axis accelerations from the FIFO
  /// @param accelerations accelerations, g
  /// @param timeout timeout in FreeRTOS ticks
  /// @return error code
  esp_err_t ReadAccelerationsFromFifo(Adxl355_Accelerations& accelerations);

  /// @brief Reads the raw X-, Y- and Z-axis acceleration offsets
  /// @param rawOffsets raw acceleration offsets
  /// @return error code
  esp_err_t ReadRawOffsets(Adxl355_RawAccelerations& rawOffsets);

  /// @brief Reads the X-, Y- and Z-axis acceleration offsets
  /// @param offsets acceleration offsets, g
  /// @return error code
  esp_err_t ReadOffsets(Adxl355_Accelerations& offsets);

  /// @brief Sets the raw X-, Y- and Z-axis acceleration offsets
  /// @param rawOffsets raw acceleration offsets
  /// @return error code
  esp_err_t SetRawOffsets(Adxl355_RawAccelerations rawOffsets);

  /// @brief Sets the X-, Y- and Z-axis acceleration offsets
  /// @param offsets acceleration offsets, g
  /// @return error code
  esp_err_t SetOffsets(Adxl355_Accelerations offsets);

  /// @brief Reads the axes for which activity detection is enabled
  /// @param axes axes for which activity detection is enabled 
  /// @return error code
  esp_err_t ReadActivityDetectionAxes(Adxl355_Axes& axes);

  /// @brief Enables and disables activity detection
  /// @param axes axes for which activity detection should be enabled 
  /// @return error code
  esp_err_t SetActivityDetectionAxes(Adxl355_Axes axes);

  /// @brief Reads the raw activity detection threshold
  /// @param rawThreshold raw activity detection threshold
  /// @return error code
  esp_err_t ReadRawActivityDetectionThreshold(uint32_t& rawThreshold);

  /// @brief Reads the activity detection threshold
  /// @param threshold activity detection threshold, g
  /// @return error code
  esp_err_t ReadActivityDetectionThreshold(float& threshold);

  /// @brief Sets the raw activity detection threshold
  /// @param rawThreshold raw activity detection threshold (max: 524288)
  /// @param rawThreshold error code
  esp_err_t SetRawActivityDetectionThreshold(uint32_t rawThreshold);

  /// @brief Sets the activity detection threshold
  /// @param threshold activity detection threshold, g
  /// @return error code
  esp_err_t SetActivityDetectionThreshold(float threshold);

  /// @brief Reads the number of consecutive events above threshold required to detect activity
  /// @param count activity detection count
  /// @return error code
  esp_err_t ReadActivityDetectionCount(uint8_t& count);

  /// @brief Sets the number of consecutive events above threshold required to detect activity
  /// @param count activity detection count
  /// @return error code
  esp_err_t SetActivityDetectionCount(uint8_t count);

  /// @brief Reads the high-pass filter frequency
  /// @param frequency high-pass filter frequency
  /// @return error code
  esp_err_t ReadHpfFrequency(Adxl355_HpfFrequency& frequency);

  /// @brief Sets the high-pass filter frequency
  /// @param frequency high-pass filter frequency
  /// @return error code
  esp_err_t SetHpfFrequency(Adxl355_HpfFrequency frequency);

  /// @brief Reads the output data rate
  /// @param outputDataRate output data rate
  /// @return error code
  esp_err_t ReadOutputDataRate(Adxl355_OutputDataRate& outputDataRate);

  /// @brief Sets the output data rate
  /// @param outputDataRate output data rate
  /// @return error code
  esp_err_t SetOutputDataRate(Adxl355_OutputDataRate outputDataRate);

  /// @brief Reads the watermark number of samples stored in the FIFO that triggers a FIFO_FULL condition
  /// @param watermark FIFO watermark
  /// @return error code
  esp_err_t ReadFifoWatermark(uint8_t& watermark);

  /// @brief Sets the watermark number of samples stored in the FIFO that triggers a FIFO_FULL condition
  /// @param watermark output data rate
  /// @return error code
  esp_err_t SetFifoWatermark(uint8_t watermark);

  /// @brief Reads the interrupts
  /// @param interrupts interrupts
  /// @return error code
  esp_err_t ReadInterrupts(Adxl355_Interrupts& interrupts);
 
  /// @brief Sets the interrupts
  /// @param interrupts interrupts
  /// @return error code
  esp_err_t SetInterrupts(Adxl355_Interrupts interrupts);

  /// @brief Reads the synchronization mode
  /// @param synchronization synchronization mode
  /// @return error code
  esp_err_t ReadSynchronization(Adxl355_Synchronization& synchronization);
 
  /// @brief Sets the synchronization mode
  /// @param synchronization synchronization mode
  /// @return error code
  esp_err_t SetSynchronization(Adxl355_Synchronization synchronization);

  /// @brief Enables the external clock
  /// @return error code
  esp_err_t EnableExternalClock();

  /// @brief Disables the external clock
  /// @return error code
  esp_err_t DisableExternalClock();

  /// @brief Checksif the external clock is enabled
  /// @param isEnabled true if the external clock is enabled
  /// @return error code
  esp_err_t IsExternalClockEnabled(bool& isEnabled);

  /// @brief Reads the acceleration range
  /// @param range acceleration range
  /// @return error code
  esp_err_t ReadRange(Adxl355_Range& range);
  
  /// @brief Reads the acceleration scale factor
  /// @param accelerationScaleFactor acceleration scale factor, g/LSB
  /// @return error code
  esp_err_t ReadAccelerationScaleFactor(float& accelerationScaleFactor);

  /// @brief Sets the acceleration range
  /// @param range acceleration range
  /// @return error code
  esp_err_t SetRange(Adxl355_Range range);

  /// @brief Reads the interrupt polarity
  /// @param polarity interrupt polarity
  /// @return error code
  esp_err_t ReadInterruptPolarity(Adxl355_InterruptPolarity& polarity);
 
  /// @brief Sets the interrupt polarity
  /// @param polarity interrupt polarity
  /// @return error code
  esp_err_t SetInterruptPolarity(Adxl355_InterruptPolarity polarity);

  /// @brief Reads the I2C speed
  /// @param speed I2C speed
  /// @return error code
  esp_err_t ReadI2CSpeed(Adxl355_I2CSpeed& speed);
 
  /// @brief Sets the I2C speed
  /// @param speed I2C speed
  /// @return error code
  esp_err_t SetI2CSpeed(Adxl355_I2CSpeed speed);

  /// @brief Switches mode to measurement mode
  /// @return error code
  esp_err_t EnableMeasurement();

  /// @brief Switches mode to standby mode
  /// @return error code
  esp_err_t DisableMeasurement();

  /// @brief Checks measurement mode
  /// @param isEnabled true if measurement is enabled
  /// @return error code
  esp_err_t IsMeasurementEnabled(bool& isEnabled);
  
  /// @brief Enables the temperature processing
  /// @return error code
  esp_err_t EnableTemperature();

  /// @brief Disables the temperature processing
  /// @return error code
  esp_err_t DisableTemperature();

  /// @brief Checks if the temperature processing is enabled
  /// @param isEnabled true if the temperature processing is enabled
  /// @return error code
  esp_err_t IsTemperatureEnabled(bool& isEnabled);

  /// @brief Enables the data-ready output
  /// @return error code
  esp_err_t EnableDataReady();

  /// @brief Disables the data-ready output
  /// @return error code
  esp_err_t DisableDataReady();

  /// @brief Checks if the data-ready output is enabled
  /// @param isReady true if the data-ready output is enabled
  /// @return error code
  esp_err_t IsDataReadyEnabled(bool& isEnabled);
  
  /// @brief Performs the self-test of the device
  /// @param accelerations X-, Y- and Z-axis test accelerations (should be X: 0.1...0.6 g, Y: 0.1...0.6 g, Z: 0.5...3.0 g)
  /// @return error code
  esp_err_t SelfTest(Adxl355_Accelerations& accelerations);

  /// @brief Resets the device
  /// @return error code
  esp_err_t Reset();

private:
  SpiController spiController;
  std::shared_ptr<Spi> spi;

  esp_err_t Read(uint8_t address, uint8_t& dest);
  esp_err_t Read(uint8_t address, void* dest, size_t numberOfRegisters);
  esp_err_t Write(uint8_t address, uint8_t value);
  esp_err_t Write(uint8_t address, void* src, size_t numberOfRegisters);
};

//==============================================================================

}