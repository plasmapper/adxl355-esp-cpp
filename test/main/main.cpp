#include "unity.h"
#include "pl_adxl355.h"

//==============================================================================

const spi_host_device_t host = HSPI_HOST;
const int mosiPin = 5;
const int misoPin = 18;
const int sclkPin = 19;
const int sclkFrequency = 10000000;
const int csPin = 21;
auto spi = std::make_shared<PL::Spi>(host, mosiPin, misoPin, sclkPin);
PL::Adxl355 adxl355(spi, sclkFrequency, csPin);

//==============================================================================

void TestDeviceId();
void TestGetNumberOfFifoSamples();
void TestOffsets();
void TestActivityDetectionAxes();
void TestActivityThreshold();
void TestActivityCount();
void TestHpfFrequencyAndOutputDataRate();
void TestFifoWatermark();
void TestInterrupts();
void TestSynchronizationAndExternalClock();
void TestRangeIntPolarityAndI2CSpeed();
void TestPower();
void TestReset();

//==============================================================================

extern "C" void app_main(void) {
  printf("Connect ADXL355 to MOSI pin %d, MISO pin %d, SCLK pin %d and CS pin %d\n", mosiPin, misoPin, sclkPin, csPin);

  UNITY_BEGIN();
  TEST_ASSERT(adxl355.Initialize() == ESP_OK);

  RUN_TEST(TestDeviceId);
  RUN_TEST(TestGetNumberOfFifoSamples);
  RUN_TEST(TestOffsets);
  RUN_TEST(TestActivityDetectionAxes);
  RUN_TEST(TestActivityThreshold);
  RUN_TEST(TestActivityCount);
  RUN_TEST(TestHpfFrequencyAndOutputDataRate);
  RUN_TEST(TestFifoWatermark);
  RUN_TEST(TestInterrupts);
  RUN_TEST(TestSynchronizationAndExternalClock);
  RUN_TEST(TestRangeIntPolarityAndI2CSpeed);
  RUN_TEST(TestPower);
  RUN_TEST(TestReset);
  UNITY_END();
}

//==============================================================================

void TestDeviceId() {
  PL::Adxl355_DeviceInfo deviceInfo;
  TEST_ASSERT(adxl355.ReadDeviceInfo(deviceInfo) == ESP_OK);;
  TEST_ASSERT_EQUAL(0xAD, deviceInfo.vendorId);
  TEST_ASSERT_EQUAL(0x1D, deviceInfo.familyId);
  TEST_ASSERT_EQUAL(0xED, deviceInfo.deviceId);
}

//==============================================================================

void TestGetNumberOfFifoSamples() {
  TEST_ASSERT(adxl355.Reset() == ESP_OK);
  TEST_ASSERT(adxl355.SetOutputDataRate(PL::Adxl355_OutputDataRate::odr4000) == ESP_OK);
  TEST_ASSERT(adxl355.EnableMeasurement() == ESP_OK);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  uint8_t numberOfFifoSamples;
  TEST_ASSERT(adxl355.ReadNumberOfFifoSamples(numberOfFifoSamples) == ESP_OK);
  TEST_ASSERT_EQUAL(PL::Adxl355::maxNumberOfFifoSamples, numberOfFifoSamples);
}

//==============================================================================

void TestOffsets() {
  float scaleFactor;
  TEST_ASSERT(adxl355.ReadAccelerationScaleFactor(scaleFactor) == ESP_OK);
  
  PL::Adxl355_RawAccelerations rawOffsetsToSet(1000, 2000, -4000);
  TEST_ASSERT(adxl355.SetRawOffsets(rawOffsetsToSet) == ESP_OK);
  
  PL::Adxl355_RawAccelerations rawOffsetsSet;
  TEST_ASSERT(adxl355.ReadRawOffsets(rawOffsetsSet) == ESP_OK);
  TEST_ASSERT_INT32_WITHIN(16, rawOffsetsToSet.x, rawOffsetsSet.x);
  TEST_ASSERT_INT32_WITHIN(16, rawOffsetsToSet.y, rawOffsetsSet.y);
  TEST_ASSERT_INT32_WITHIN(16, rawOffsetsToSet.z, rawOffsetsSet.z);
  
  PL::Adxl355_Accelerations offsetsSet;
  TEST_ASSERT(adxl355.ReadOffsets(offsetsSet) == ESP_OK);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, rawOffsetsToSet.x * scaleFactor, offsetsSet.x);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, rawOffsetsToSet.y * scaleFactor, offsetsSet.y);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, rawOffsetsToSet.z * scaleFactor, offsetsSet.z);

  PL::Adxl355_Accelerations offsetsToSet(0.1, 0.2, -0.4);
  TEST_ASSERT(adxl355.SetOffsets(offsetsToSet) == ESP_OK);
  
  TEST_ASSERT(adxl355.ReadRawOffsets(rawOffsetsSet) == ESP_OK);
  TEST_ASSERT_INT32_WITHIN(16, offsetsToSet.x / scaleFactor, rawOffsetsSet.x);
  TEST_ASSERT_INT32_WITHIN(16, offsetsToSet.y / scaleFactor, rawOffsetsSet.y);
  TEST_ASSERT_INT32_WITHIN(16, offsetsToSet.z / scaleFactor, rawOffsetsSet.z);
  TEST_ASSERT(adxl355.ReadOffsets(offsetsSet) == ESP_OK);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, offsetsToSet.x, offsetsSet.x);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, offsetsToSet.y, offsetsSet.y);
  TEST_ASSERT_FLOAT_WITHIN(16 * scaleFactor, offsetsToSet.z, offsetsSet.z);
}

//==============================================================================

void TestActivityDetectionAxes() {
  PL::Adxl355_Axes axesToSet = PL::Adxl355_Axes::x | PL::Adxl355_Axes::z;
  TEST_ASSERT(adxl355.SetActivityDetectionAxes(axesToSet) == ESP_OK);
  PL::Adxl355_Axes axesSet;
  TEST_ASSERT(adxl355.ReadActivityDetectionAxes(axesSet) == ESP_OK);
  TEST_ASSERT_EQUAL((uint8_t)axesToSet, (uint8_t)axesSet);
}

//==============================================================================

void TestActivityThreshold() {
  float scaleFactor;
  TEST_ASSERT(adxl355.ReadAccelerationScaleFactor(scaleFactor) == ESP_OK);
  
  uint32_t rawThresholdToSet = 1000;
  TEST_ASSERT(adxl355.SetRawActivityDetectionThreshold(rawThresholdToSet) == ESP_OK);
  
  uint32_t rawThresholdSet;
  TEST_ASSERT(adxl355.ReadRawActivityDetectionThreshold(rawThresholdSet) == ESP_OK);
  TEST_ASSERT_UINT32_WITHIN(8, rawThresholdToSet, rawThresholdSet);
  float thresholdSet;
  TEST_ASSERT(adxl355.ReadActivityDetectionThreshold(thresholdSet) == ESP_OK);
  TEST_ASSERT_FLOAT_WITHIN(8 * scaleFactor, rawThresholdToSet * scaleFactor, thresholdSet);

  float thresholdToSet = 0.1;
  TEST_ASSERT(adxl355.SetActivityDetectionThreshold(thresholdToSet) == ESP_OK);
  
  TEST_ASSERT(adxl355.ReadRawActivityDetectionThreshold(rawThresholdSet) == ESP_OK);
  TEST_ASSERT_UINT32_WITHIN(thresholdToSet / scaleFactor, rawThresholdSet, 8);
  TEST_ASSERT(adxl355.ReadActivityDetectionThreshold(thresholdSet) == ESP_OK);
  TEST_ASSERT_FLOAT_WITHIN(thresholdToSet, thresholdSet, 8 * scaleFactor);
}

//==============================================================================

void TestActivityCount() {
  uint8_t activityCountToSet = 10;
  TEST_ASSERT(adxl355.SetActivityDetectionCount(activityCountToSet) == ESP_OK);
  uint8_t activityCountSet;
  TEST_ASSERT(adxl355.ReadActivityDetectionCount(activityCountSet) == ESP_OK);
  TEST_ASSERT_EQUAL(activityCountToSet, activityCountSet);
}

//==============================================================================

void TestHpfFrequencyAndOutputDataRate() {
  for (uint8_t freqToSet = 0; freqToSet <= (uint8_t)PL::Adxl355_HpfFrequency::hpf0_0238; freqToSet++) {
    for (uint8_t odrToSet = 0; odrToSet <= (uint8_t)PL::Adxl355_OutputDataRate::odr3_906; odrToSet++) {
      TEST_ASSERT(adxl355.SetHpfFrequency((PL::Adxl355_HpfFrequency)freqToSet) == ESP_OK);
      TEST_ASSERT(adxl355.SetOutputDataRate((PL::Adxl355_OutputDataRate)odrToSet) == ESP_OK);
      PL::Adxl355_HpfFrequency freqSet;
      PL::Adxl355_OutputDataRate odrSet;
      TEST_ASSERT(adxl355.ReadHpfFrequency(freqSet) == ESP_OK);
      TEST_ASSERT(adxl355.ReadOutputDataRate(odrSet) == ESP_OK);
      TEST_ASSERT_EQUAL(freqToSet, (uint8_t)freqSet);
      TEST_ASSERT_EQUAL(odrToSet, (uint8_t)odrSet);
    }
  }
}

//==============================================================================

void TestFifoWatermark() {
  uint8_t watermarkToSet = 15;
  TEST_ASSERT(adxl355.SetFifoWatermark(watermarkToSet) == ESP_OK);
  uint8_t watermarkSet;
  TEST_ASSERT(adxl355.ReadFifoWatermark(watermarkSet) == ESP_OK);
  TEST_ASSERT_EQUAL(watermarkToSet, watermarkSet);
}

//==============================================================================

void TestInterrupts() {
  PL::Adxl355_Interrupts interruptsToSet = PL::Adxl355_Interrupts::dataReadyInt1 | PL::Adxl355_Interrupts::fifoFullInt2;
  TEST_ASSERT(adxl355.SetInterrupts(interruptsToSet) == ESP_OK);
  PL::Adxl355_Interrupts interruptsSet;
  TEST_ASSERT(adxl355.ReadInterrupts(interruptsSet) == ESP_OK);
  TEST_ASSERT_EQUAL((uint8_t)interruptsToSet, (uint8_t)interruptsSet);
}

//==============================================================================

void TestSynchronizationAndExternalClock() {
  for (uint8_t syncToSet = 0; syncToSet <= (uint8_t)PL::Adxl355_Synchronization::externalWithInterpolation; syncToSet++) {
    for (uint8_t extClockToSet = 0; extClockToSet <= 1; extClockToSet++) {
      TEST_ASSERT(adxl355.SetSynchronization((PL::Adxl355_Synchronization)syncToSet) == ESP_OK);
      if (extClockToSet) {
        TEST_ASSERT(adxl355.EnableExternalClock() == ESP_OK);
      }
      else {
        TEST_ASSERT(adxl355.DisableExternalClock() == ESP_OK);
      }
      vTaskDelay(1);
      PL::Adxl355_Synchronization syncSet;
      bool extClockSet;
      TEST_ASSERT(adxl355.ReadSynchronization(syncSet) == ESP_OK);
      TEST_ASSERT(adxl355.IsExternalClockEnabled(extClockSet) == ESP_OK);
      TEST_ASSERT_EQUAL(syncToSet, (uint8_t)syncSet);
      TEST_ASSERT_EQUAL(extClockToSet, (uint8_t)extClockSet);
    }
  }
}

//==============================================================================

void TestRangeIntPolarityAndI2CSpeed() {
  for (uint8_t rangeToSet = 0; rangeToSet <= (uint8_t)PL::Adxl355_Range::range8g; rangeToSet++) {
    for (uint8_t intPolToSet = 0; intPolToSet <= (uint8_t)PL::Adxl355_InterruptPolarity::activeHigh; intPolToSet++) {
      for (uint8_t i2CSpeedToSet = 0; i2CSpeedToSet <= (uint8_t)PL::Adxl355_I2CSpeed::highSpeed; i2CSpeedToSet++) {
        TEST_ASSERT(adxl355.SetRange((PL::Adxl355_Range)rangeToSet) == ESP_OK);
        TEST_ASSERT(adxl355.SetInterruptPolarity((PL::Adxl355_InterruptPolarity)intPolToSet) == ESP_OK);
        TEST_ASSERT(adxl355.SetI2CSpeed((PL::Adxl355_I2CSpeed)i2CSpeedToSet) == ESP_OK);
        
        PL::Adxl355_Range rangeSet;
        float scaleFactorSet;
        PL::Adxl355_InterruptPolarity intPolSet;
        PL::Adxl355_I2CSpeed i2CSpeedSet;
        TEST_ASSERT(adxl355.ReadRange(rangeSet) == ESP_OK);
        TEST_ASSERT(adxl355.ReadAccelerationScaleFactor(scaleFactorSet) == ESP_OK);
        TEST_ASSERT(adxl355.ReadInterruptPolarity(intPolSet) == ESP_OK);
        TEST_ASSERT(adxl355.ReadI2CSpeed(i2CSpeedSet) == ESP_OK);
        
        TEST_ASSERT_EQUAL(rangeToSet, (uint8_t)rangeSet);
        switch ((PL::Adxl355_Range)rangeToSet) {
          case PL::Adxl355_Range::range2g:
            TEST_ASSERT_EQUAL (PL::Adxl355::accelerationScaleFactorRange2G, scaleFactorSet);
            break;
          case PL::Adxl355_Range::range4g:
            TEST_ASSERT_EQUAL (PL::Adxl355::accelerationScaleFactorRange4G, scaleFactorSet);
            break;
          case PL::Adxl355_Range::range8g:
            TEST_ASSERT_EQUAL (PL::Adxl355::accelerationScaleFactorRange8G, scaleFactorSet);
            break;
        }
        TEST_ASSERT_EQUAL(intPolToSet, (uint8_t)intPolSet);
        TEST_ASSERT_EQUAL(i2CSpeedToSet, (uint8_t)i2CSpeedSet);
      }
    }
  }
}

//==============================================================================

void TestPower() {
  for (uint8_t measurementToSet = 0; measurementToSet <= 1; measurementToSet++) {
    for (uint8_t temperatureToSet = 0; temperatureToSet <= 1; temperatureToSet++) {
      for (uint8_t dataReadyToSet = 0; dataReadyToSet <= 1; dataReadyToSet++) {
        if (measurementToSet) {
          TEST_ASSERT(adxl355.EnableMeasurement() == ESP_OK);
        }
        else {
          TEST_ASSERT(adxl355.DisableMeasurement() == ESP_OK);
        }
        if (temperatureToSet) {
          TEST_ASSERT(adxl355.EnableTemperature() == ESP_OK);
        }
        else {
          TEST_ASSERT(adxl355.DisableTemperature() == ESP_OK);
        }
        if (dataReadyToSet) {
          TEST_ASSERT(adxl355.EnableDataReady() == ESP_OK);
        }
        else {
          TEST_ASSERT(adxl355.DisableDataReady() == ESP_OK);
        }
        
        bool measurementSet, temperatureSet, dataReadySet;
        TEST_ASSERT(adxl355.IsMeasurementEnabled(measurementSet) == ESP_OK);
        TEST_ASSERT(adxl355.IsTemperatureEnabled(temperatureSet) == ESP_OK);
        TEST_ASSERT(adxl355.IsDataReadyEnabled(dataReadySet) == ESP_OK);
        TEST_ASSERT_EQUAL(measurementToSet, (uint8_t)measurementSet);
        TEST_ASSERT_EQUAL(temperatureToSet, (uint8_t)temperatureSet);
        TEST_ASSERT_EQUAL(dataReadyToSet, (uint8_t)dataReadySet); 
      }
    }
  }
}

//==============================================================================

void TestReset() {
  TEST_ASSERT(adxl355.Reset() == ESP_OK);
  TEST_ASSERT(adxl355.SetOutputDataRate(PL::Adxl355_OutputDataRate::odr4000) == ESP_OK);
  TEST_ASSERT(adxl355.EnableMeasurement() == ESP_OK);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  uint8_t numberOfFifoSamples;
  TEST_ASSERT(adxl355.ReadNumberOfFifoSamples(numberOfFifoSamples) == ESP_OK);
  TEST_ASSERT_EQUAL(PL::Adxl355::maxNumberOfFifoSamples, numberOfFifoSamples);
  TEST_ASSERT(adxl355.Reset() == ESP_OK);
  TEST_ASSERT(adxl355.ReadNumberOfFifoSamples(numberOfFifoSamples) == ESP_OK);
  TEST_ASSERT_EQUAL(0, numberOfFifoSamples);
}