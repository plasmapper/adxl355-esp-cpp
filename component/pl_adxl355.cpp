#include "pl_adxl355.h"
#include "esp_check.h"

//==============================================================================

static const char* TAG = "pl_adxl355";

//==============================================================================

namespace PL {

//==============================================================================

const uint8_t spiMode = 8;
const int spiNumberOfAddressBits = 8;
const int spiMaxSclkMisoDelay = 30;

const uint8_t ADXL355_REG_DEVID_AD = 0x00;
const uint8_t ADXL355_REG_DEVID_MST = 0x01;
const uint8_t ADXL355_REG_PARTID = 0x02;
const uint8_t ADXL355_REG_REVID = 0x03;
const uint8_t ADXL355_REG_STATUS = 0x04;
const uint8_t ADXL355_REG_FIFO_ENTRIES = 0x05;
const uint8_t ADXL355_REG_TEMP2 = 0x06;
const uint8_t ADXL355_REG_TEMP1 = 0x07;
const uint8_t ADXL355_REG_XDATA3 = 0x08;
const uint8_t ADXL355_REG_XDATA2 = 0x09;
const uint8_t ADXL355_REG_XDATA1 = 0x0A;
const uint8_t ADXL355_REG_YDATA3 = 0x0B;
const uint8_t ADXL355_REG_YDATA2 = 0x0C;
const uint8_t ADXL355_REG_YDATA1 = 0x0D;
const uint8_t ADXL355_REG_ZDATA3 = 0x0E;
const uint8_t ADXL355_REG_ZDATA2 = 0x0F;
const uint8_t ADXL355_REG_ZDATA1 = 0x10;
const uint8_t ADXL355_REG_FIFO_DATA = 0x11;
const uint8_t ADXL355_REG_OFFSET_X_H = 0x1E;
const uint8_t ADXL355_REG_OFFSET_X_L = 0x1F;
const uint8_t ADXL355_REG_OFFSET_Y_H = 0x20;
const uint8_t ADXL355_REG_OFFSET_Y_L = 0x21;
const uint8_t ADXL355_REG_OFFSET_Z_H = 0x22;
const uint8_t ADXL355_REG_OFFSET_Z_L = 0x23;
const uint8_t ADXL355_REG_ACT_EN = 0x24;
const uint8_t ADXL355_REG_ACT_THRESH_H = 0x25;
const uint8_t ADXL355_REG_ACT_THRESH_L = 0x26;
const uint8_t ADXL355_REG_ACT_COUNT = 0x27;
const uint8_t ADXL355_REG_FILTER = 0x28;
const uint8_t ADXL355_REG_FIFO_SAMPLES = 0x29;
const uint8_t ADXL355_REG_INT_MAP = 0x2A;
const uint8_t ADXL355_REG_SYNC = 0x2B;
const uint8_t ADXL355_REG_RANGE = 0x2C;
const uint8_t ADXL355_REG_POWER_CTL = 0x2D;
const uint8_t ADXL355_REG_SELF_TEST = 0x2E;
const uint8_t ADXL355_REG_RESET = 0x2F;

const uint8_t ADXL355_REG_STATUS_DATA_RDY = 0x01;
const uint8_t ADXL355_REG_STATUS_FIFO_FULL = 0x02;
const uint8_t ADXL355_REG_STATUS_FIFO_OVR = 0x04;
const uint8_t ADXL355_REG_STATUS_ACTIVITY = 0x08;
const uint8_t ADXL355_REG_STATUS_NVM_BUSY = 0x10;

const uint8_t ADXL355_REG_FILTER_HPF_SHIFT = 4;
const uint8_t ADXL355_REG_FILTER_HPF_MASK = 0xF0;
const uint8_t ADXL355_REG_FILTER_ODR_MASK = 0x0F;

const uint8_t ADXL355_REG_SYNC_SYNC_MAP = 0x03;
const uint8_t ADXL355_REG_SYNC_EXT_CLK = 0x04;

const uint8_t ADXL355_REG_RANGE_RANGE_MASK = 0x03;
const uint8_t ADXL355_REG_RANGE_RANGE_PM2G = 0x01;
const uint8_t ADXL355_REG_RANGE_RANGE_PM4G = 0x02;
const uint8_t ADXL355_REG_RANGE_RANGE_PM8G = 0x03;
const uint8_t ADXL355_REG_RANGE_INT_POL_SHIFT = 6;
const uint8_t ADXL355_REG_RANGE_INT_POL_MASK = 0x40;
const uint8_t ADXL355_REG_RANGE_I2C_HS_SHIFT = 7;
const uint8_t ADXL355_REG_RANGE_I2C_HS_MASK = 0x80;

const uint8_t ADXL355_REG_POWER_CTL_STANDBY = 0x01;
const uint8_t ADXL355_REG_POWER_CTL_TEMP_OFF = 0x02;
const uint8_t ADXL355_REG_POWER_CTL_DRDY_OFF = 0x04;

const uint8_t ADXL355_REG_SELF_TEST_ST1 = 0x01;
const uint8_t ADXL355_REG_SELF_TEST_ST2 = 0x02;

const uint8_t ADXL355_REG_RESET_RESET_CODE = 0x52;

//==============================================================================

Adxl355::Adxl355(std::shared_ptr<Spi> spi, int sclkFrequency, int csPin) :
    spiController(spi, spiMode, sclkFrequency, csPin), spi(spi) {
  spiController.SetNumberOfAddressBits(spiNumberOfAddressBits);
  spiController.SetMaxSclkMisoDelay(spiMaxSclkMisoDelay);
}

//==============================================================================

Adxl355::~Adxl355() { }

//==============================================================================

esp_err_t Adxl355::Lock(TickType_t timeout) {
  return spiController.Lock(timeout);
}

//==============================================================================

esp_err_t Adxl355::Unlock() {
  return spiController.Unlock();
}

//==============================================================================

esp_err_t Adxl355::Initialize() {
  return spiController.Initialize();
}

//==============================================================================

esp_err_t Adxl355::ReadDeviceInfo(Adxl355_DeviceInfo& deviceInfo) {
  LockGuard lg(*this, *spi);
  uint8_t data[4];
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_DEVID_AD, &data, sizeof(data)), TAG, "read failed");
  deviceInfo.vendorId = data[0];
  deviceInfo.familyId = data[1];
  deviceInfo.deviceId = data[2];
  deviceInfo.revisionId = data[3];
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadStatus(Adxl355_Status& status) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_STATUS, (uint8_t&)status), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadNumberOfFifoSamples(uint8_t& numberOfFifoSamples) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_ENTRIES, numberOfFifoSamples), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadRawTemperature(uint16_t& rawTemperature) {
  LockGuard lg(*this, *spi);
  uint8_t data[2];
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_TEMP2, &data, sizeof(data)), TAG, "read failed");
  ((uint8_t*)&rawTemperature)[0] = data[1];
  ((uint8_t*)&rawTemperature)[1] = data[0];
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadTemperature(float& temperature) {
  LockGuard lg(*this, *spi);
  uint16_t rawTemperature;
  ESP_RETURN_ON_ERROR(ReadRawTemperature(rawTemperature), TAG, "read raw temperature failed");
  temperature = (((int16_t)rawTemperature) - ((int16_t)temperatureInterceptLsb)) / temperatureSlope + temperatureInterceptDegC;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadRawAccelerations(Adxl355_RawAccelerations& rawAccelerations) {
  LockGuard lg(*this, *spi);
  uint8_t data[9];
  rawAccelerations.x = rawAccelerations.y = rawAccelerations.z = 0;

  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_XDATA3, &data, sizeof(data)), TAG, "read failed");
  
  ((uint8_t*)&rawAccelerations.x)[1] = data[2];
  ((uint8_t*)&rawAccelerations.x)[2] = data[1];
  ((uint8_t*)&rawAccelerations.x)[3] = data[0];
  
  ((uint8_t*)&rawAccelerations.y)[1] = data[5];
  ((uint8_t*)&rawAccelerations.y)[2] = data[4];
  ((uint8_t*)&rawAccelerations.y)[3] = data[3];
  
  ((uint8_t*)&rawAccelerations.z)[1] = data[8];
  ((uint8_t*)&rawAccelerations.z)[2] = data[7];
  ((uint8_t*)&rawAccelerations.z)[3] = data[6];
  
  rawAccelerations.x /= 4096;
  rawAccelerations.y /= 4096;
  rawAccelerations.z /= 4096;
  
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadAccelerations(Adxl355_Accelerations& accelerations) {
  LockGuard lg(*this, *spi);
  Adxl355_RawAccelerations rawAccelerations;
  ESP_RETURN_ON_ERROR(ReadRawAccelerations(rawAccelerations), TAG, "read raw accelerations failed");
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  accelerations.x = rawAccelerations.x * scaleFactor;
  accelerations.y = rawAccelerations.y * scaleFactor;
  accelerations.z = rawAccelerations.z * scaleFactor;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ClearFifo() {
  LockGuard lg(*this, *spi);
  uint8_t numberOfFifoSamples;
  uint8_t data[3];
  do {
    ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_ENTRIES, numberOfFifoSamples), TAG, "read failed");
    if (numberOfFifoSamples)
      ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_DATA, &data, sizeof(data)), TAG, "read failed");
  } while (numberOfFifoSamples);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadRawAccelerationsFromFifo(Adxl355_RawAccelerations& rawAccelerations) {
  LockGuard lg(*this, *spi);
  uint8_t data[3];
  rawAccelerations.x = rawAccelerations.y = rawAccelerations.z = 0;
  
  do {
    ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_DATA, &data, sizeof(data)), TAG, "read failed");
  } while (!(data[2] & 0x03));
  
  if (data[2] & 0x02) {
    ESP_RETURN_ON_ERROR(ESP_FAIL, TAG, "no valid acceleration data in FIFO");
  }
    
  ((uint8_t*)&rawAccelerations.x)[1] = data[2];
  ((uint8_t*)&rawAccelerations.x)[2] = data[1];
  ((uint8_t*)&rawAccelerations.x)[3] = data[0];
  
  uint8_t numberOfFifoSamples;
  do {
    ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_ENTRIES, numberOfFifoSamples), TAG, "read failed");
  }
  while(numberOfFifoSamples < 2);
  
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_DATA, &data, sizeof(data)), TAG, "read failed");
  ((uint8_t*)&rawAccelerations.y)[1] = data[2];
  ((uint8_t*)&rawAccelerations.y)[2] = data[1];
  ((uint8_t*)&rawAccelerations.y)[3] = data[0];
  
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_DATA, &data, sizeof(data)), TAG, "read failed");
  ((uint8_t*)&rawAccelerations.z)[1] = data[2];
  ((uint8_t*)&rawAccelerations.z)[2] = data[1];
  ((uint8_t*)&rawAccelerations.z)[3] = data[0];
  
  rawAccelerations.x /= 4096;
  rawAccelerations.y /= 4096;
  rawAccelerations.z /= 4096;
  
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadAccelerationsFromFifo(Adxl355_Accelerations& accelerations) {
  LockGuard lg(*this, *spi);
  Adxl355_RawAccelerations rawAccelerations;
  ESP_RETURN_ON_ERROR(ReadRawAccelerationsFromFifo(rawAccelerations), TAG, "read raw accelerations from fifo failed");
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  accelerations.x = rawAccelerations.x * scaleFactor;
  accelerations.y = rawAccelerations.y * scaleFactor;
  accelerations.z = rawAccelerations.z * scaleFactor;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadRawOffsets(Adxl355_RawAccelerations& rawOffsets) {
  LockGuard lg(*this, *spi);
  uint8_t data[6];
  rawOffsets.x = rawOffsets.y = rawOffsets.z = 0;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_OFFSET_X_H, &data, sizeof(data)), TAG, "read failed");
  ((uint8_t*)&rawOffsets.x)[2] = data[1];
  ((uint8_t*)&rawOffsets.x)[3] = data[0];
  ((uint8_t*)&rawOffsets.y)[2] = data[3];
  ((uint8_t*)&rawOffsets.y)[3] = data[2];
  ((uint8_t*)&rawOffsets.z)[2] = data[5];
  ((uint8_t*)&rawOffsets.z)[3] = data[4];
  rawOffsets.x /= 4096;
  rawOffsets.y /= 4096;
  rawOffsets.z /= 4096;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadOffsets(Adxl355_Accelerations& offsets) {
  LockGuard lg(*this, *spi);
  Adxl355_RawAccelerations rawOffsets;
  ESP_RETURN_ON_ERROR(ReadRawOffsets(rawOffsets), TAG, "read raw offsets failed");
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  offsets.x = rawOffsets.x * scaleFactor;
  offsets.y = rawOffsets.y * scaleFactor;
  offsets.z = rawOffsets.z * scaleFactor;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetRawOffsets(Adxl355_RawAccelerations rawOffsets) {
  LockGuard lg(*this, *spi);
  uint8_t data[6];
  rawOffsets.x *= 4096;
  rawOffsets.y *= 4096;
  rawOffsets.z *= 4096;
  data[0] = ((uint8_t*)&rawOffsets.x)[3];
  data[1] = ((uint8_t*)&rawOffsets.x)[2];
  data[2] = ((uint8_t*)&rawOffsets.y)[3];
  data[3] = ((uint8_t*)&rawOffsets.y)[2];
  data[4] = ((uint8_t*)&rawOffsets.z)[3];
  data[5] = ((uint8_t*)&rawOffsets.z)[2];
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_OFFSET_X_H, &data, sizeof(data)), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetOffsets(Adxl355_Accelerations offsets) {
  LockGuard lg(*this, *spi);
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  ESP_RETURN_ON_ERROR(SetRawOffsets(Adxl355_RawAccelerations(offsets.x / scaleFactor, offsets.y / scaleFactor, offsets.z / scaleFactor)), TAG, "set raw offsets failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadActivityDetectionAxes(Adxl355_Axes& axes) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_ACT_EN, (uint8_t&)axes), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetActivityDetectionAxes(Adxl355_Axes axes) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_ACT_EN, (uint8_t)axes), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadRawActivityDetectionThreshold(uint32_t& rawThreshold) {
  LockGuard lg(*this, *spi);
  uint8_t data[2];
  rawThreshold = 0;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_ACT_THRESH_H, &data, sizeof(data)), TAG, "read failed");
  ((uint8_t*)&rawThreshold)[0] = data[1];
  ((uint8_t*)&rawThreshold)[1] = data[0];
  rawThreshold <<= 3;
  return ESP_OK; 
}

//==============================================================================

esp_err_t Adxl355::ReadActivityDetectionThreshold(float& threshold) {
  LockGuard lg(*this, *spi);
  uint32_t rawThreshold;
  ESP_RETURN_ON_ERROR( ReadRawActivityDetectionThreshold(rawThreshold), TAG, "read raw threshold failed");
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  threshold = rawThreshold * scaleFactor;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetRawActivityDetectionThreshold(uint32_t rawThreshold) {
  LockGuard lg(*this, *spi);
  uint8_t data[2];
  rawThreshold >>= 3;
  data[0] = ((uint8_t*)&rawThreshold)[1];
  data[1] = ((uint8_t*)&rawThreshold)[0];
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_ACT_THRESH_H, &data, sizeof(data)), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetActivityDetectionThreshold(float threshold) {
  LockGuard lg(*this, *spi);
  float scaleFactor;
  ESP_RETURN_ON_ERROR(ReadAccelerationScaleFactor(scaleFactor), TAG, "read acceleration scale factor failed");
  ESP_RETURN_ON_ERROR(SetRawActivityDetectionThreshold(threshold / scaleFactor), TAG, "set raw activity detection threshold failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadActivityDetectionCount(uint8_t& count) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_ACT_COUNT, count), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetActivityDetectionCount(uint8_t count) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_ACT_COUNT, count), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadHpfFrequency(Adxl355_HpfFrequency& frequency) {
  LockGuard lg(*this, *spi);
  uint8_t filterRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FILTER, filterRegister), TAG, "read failed");
  frequency = (Adxl355_HpfFrequency)((filterRegister & ADXL355_REG_FILTER_HPF_MASK) >> ADXL355_REG_FILTER_HPF_SHIFT);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetHpfFrequency(Adxl355_HpfFrequency frequency) {
  LockGuard lg(*this, *spi);
  uint8_t filterRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FILTER, filterRegister), TAG, "read failed");
  filterRegister &= ~ADXL355_REG_FILTER_HPF_MASK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_FILTER, filterRegister | ((uint8_t)frequency << ADXL355_REG_FILTER_HPF_SHIFT)), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadOutputDataRate(Adxl355_OutputDataRate& outputDataRate) {
  LockGuard lg(*this, *spi);
  uint8_t filterRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FILTER, filterRegister), TAG, "read failed");
  outputDataRate = (Adxl355_OutputDataRate)(filterRegister & ADXL355_REG_FILTER_ODR_MASK);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetOutputDataRate(Adxl355_OutputDataRate outputDataRate) {
  LockGuard lg(*this, *spi);
  uint8_t filterRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FILTER, filterRegister), TAG, "read failed");
  filterRegister &= ~ADXL355_REG_FILTER_ODR_MASK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_FILTER, filterRegister | (uint8_t)outputDataRate), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadFifoWatermark(uint8_t& watermark) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_FIFO_SAMPLES, watermark), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetFifoWatermark(uint8_t watermark) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_FIFO_SAMPLES, watermark), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadInterrupts(Adxl355_Interrupts& interrupts) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_INT_MAP, (uint8_t&)interrupts), TAG, "read failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetInterrupts(Adxl355_Interrupts interrupts) {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_INT_MAP, (uint8_t)interrupts), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadSynchronization(Adxl355_Synchronization& synchronization) {
  LockGuard lg(*this, *spi);
  uint8_t syncRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_SYNC, syncRegister), TAG, "read failed");
  synchronization = (Adxl355_Synchronization)(syncRegister & ADXL355_REG_SYNC_SYNC_MAP);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetSynchronization(Adxl355_Synchronization synchronization) {
  LockGuard lg(*this, *spi);
  uint8_t syncRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_SYNC, syncRegister), TAG, "read failed");
  syncRegister &= ~ADXL355_REG_SYNC_SYNC_MAP;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SYNC, syncRegister | (uint8_t)synchronization), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::EnableExternalClock() {
  LockGuard lg(*this, *spi);
  uint8_t syncRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_SYNC, syncRegister), TAG, "read failed");
  syncRegister &= ~ADXL355_REG_SYNC_EXT_CLK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SYNC, syncRegister), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::DisableExternalClock() {
  LockGuard lg(*this, *spi);
  uint8_t syncRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_SYNC, syncRegister), TAG, "read failed");
  syncRegister |= ADXL355_REG_SYNC_EXT_CLK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SYNC, syncRegister), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::IsExternalClockEnabled(bool& isEnabled) {
  LockGuard lg(*this, *spi);
  uint8_t syncRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_SYNC, syncRegister), TAG, "read failed");
  isEnabled = !(syncRegister & ADXL355_REG_SYNC_EXT_CLK);
  return ESP_OK;
}
  
//==============================================================================

esp_err_t Adxl355::ReadRange(Adxl355_Range& range) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  range = (Adxl355_Range)(rangeRegister & ADXL355_REG_RANGE_RANGE_MASK);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetRange(Adxl355_Range range) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  rangeRegister &= ~ADXL355_REG_RANGE_RANGE_MASK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_RANGE, rangeRegister | (uint8_t)range), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadAccelerationScaleFactor(float& accelerationScaleFactor) {
  LockGuard lg(*this, *spi);
  Adxl355_Range range;
  ESP_RETURN_ON_ERROR(ReadRange(range), TAG, "read range failed");
  
  switch (range) {
    case Adxl355_Range::range2g:
      accelerationScaleFactor = accelerationScaleFactorRange2G;
      break;
    case Adxl355_Range::range4g:
      accelerationScaleFactor = accelerationScaleFactorRange4G;
      break;
    case Adxl355_Range::range8g:
      accelerationScaleFactor = accelerationScaleFactorRange8G;
      break;
    default:
      accelerationScaleFactor = 1;
      break;
  }

  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadInterruptPolarity(Adxl355_InterruptPolarity& polarity) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  polarity = (Adxl355_InterruptPolarity)((rangeRegister & ADXL355_REG_RANGE_INT_POL_MASK) >> ADXL355_REG_RANGE_INT_POL_SHIFT);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetInterruptPolarity(Adxl355_InterruptPolarity polarity) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  rangeRegister &= ~ADXL355_REG_RANGE_INT_POL_MASK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_RANGE, rangeRegister | ((uint8_t)polarity << ADXL355_REG_RANGE_INT_POL_SHIFT)), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::ReadI2CSpeed(Adxl355_I2CSpeed& speed) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  speed = (Adxl355_I2CSpeed)((rangeRegister & ADXL355_REG_RANGE_I2C_HS_MASK) >> ADXL355_REG_RANGE_I2C_HS_SHIFT);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SetI2CSpeed(Adxl355_I2CSpeed speed) {
  LockGuard lg(*this, *spi);
  uint8_t rangeRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_RANGE, rangeRegister), TAG, "read failed");
  rangeRegister &= ~ADXL355_REG_RANGE_I2C_HS_MASK;
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_RANGE, rangeRegister | ((uint8_t)speed << ADXL355_REG_RANGE_I2C_HS_SHIFT)), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::EnableMeasurement() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister & ~ADXL355_REG_POWER_CTL_STANDBY), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::DisableMeasurement() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister | ADXL355_REG_POWER_CTL_STANDBY), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::IsMeasurementEnabled(bool& isEnabled) {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  isEnabled = !(powerCtlRegister & ADXL355_REG_POWER_CTL_STANDBY);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::EnableTemperature() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister & ~ADXL355_REG_POWER_CTL_TEMP_OFF), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::DisableTemperature() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister | ADXL355_REG_POWER_CTL_TEMP_OFF), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::IsTemperatureEnabled(bool& isEnabled) {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  isEnabled = !(powerCtlRegister & ADXL355_REG_POWER_CTL_TEMP_OFF);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::EnableDataReady() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister & ~ADXL355_REG_POWER_CTL_DRDY_OFF), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::DisableDataReady() {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_POWER_CTL, powerCtlRegister | ADXL355_REG_POWER_CTL_DRDY_OFF), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::IsDataReadyEnabled(bool& isEnabled) {
  LockGuard lg(*this, *spi);
  uint8_t powerCtlRegister;
  ESP_RETURN_ON_ERROR(Read(ADXL355_REG_POWER_CTL, powerCtlRegister), TAG, "read failed");
  isEnabled = !(powerCtlRegister & ADXL355_REG_POWER_CTL_DRDY_OFF);
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::SelfTest(Adxl355_Accelerations& accelerations) {
  LockGuard lg(*this, *spi);
  bool measurementIsEnabled;
  ESP_RETURN_ON_ERROR(IsMeasurementEnabled(measurementIsEnabled), TAG, "is measurement enabled failed");
  
  Adxl355_Accelerations accelNoForce, accelForce;

  ESP_RETURN_ON_ERROR(SetRange(Adxl355_Range::range8g), TAG, "set range failed");
  ESP_RETURN_ON_ERROR(DisableMeasurement(), TAG, "disable measurement failed");
  ESP_RETURN_ON_ERROR(ReadAccelerations(accelNoForce), TAG, "read accelerations failed");

  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SELF_TEST, ADXL355_REG_SELF_TEST_ST1), TAG, "write failed");
  vTaskDelay(2);
  ESP_RETURN_ON_ERROR(EnableMeasurement(), TAG, "enable measurement failed");
  vTaskDelay(2);
  ESP_RETURN_ON_ERROR(ReadAccelerations(accelNoForce), TAG, "read accelerations failed");

  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SELF_TEST, ADXL355_REG_SELF_TEST_ST2 | ADXL355_REG_SELF_TEST_ST1), TAG, "write failed");
  vTaskDelay(2);
  ESP_RETURN_ON_ERROR(ReadAccelerations(accelForce), TAG, "read accelerations failed");
  if (!measurementIsEnabled) {
    ESP_RETURN_ON_ERROR(DisableMeasurement(), TAG, "disable measurement failed");
  }
  
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_SELF_TEST, 0), TAG, "write failed");

  accelerations.x = accelForce.x - accelNoForce.x;
  accelerations.y = accelForce.y - accelNoForce.y;
  accelerations.z = accelForce.z - accelNoForce.z;
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::Reset() {
  LockGuard lg(*this, *spi);
  ESP_RETURN_ON_ERROR(Write(ADXL355_REG_RESET, ADXL355_REG_RESET_RESET_CODE), TAG, "write failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::Read(uint8_t address, uint8_t& dest) {
  return Read(address, &dest, 1);
}

//==============================================================================

esp_err_t Adxl355::Read(uint8_t address, void* dest, size_t numberOfRegisters) {
  ESP_RETURN_ON_FALSE(dest, ESP_ERR_INVALID_ARG, TAG, "dest is null");
  ESP_RETURN_ON_ERROR(spiController.Transaction(0, (address << 1) | 1, NULL, dest, numberOfRegisters * 8), TAG, "SPI transaction failed");
  return ESP_OK;
}

//==============================================================================

esp_err_t Adxl355::Write(uint8_t address, uint8_t value) {
  return Write(address, &value, 1);
}

//==============================================================================

esp_err_t Adxl355::Write(uint8_t address, void* src, size_t numberOfRegisters) {
  ESP_RETURN_ON_FALSE(src, ESP_ERR_INVALID_ARG, TAG, "dest is null");
  ESP_RETURN_ON_ERROR(spiController.Transaction(0, address << 1, src, NULL, numberOfRegisters * 8), TAG, "SPI transaction failed");
  return ESP_OK;
}

//==============================================================================

}