#pragma once
#include "pl_common.h"
#include "driver/spi_master.h"

//==============================================================================

namespace PL {

//==============================================================================

/// @brief ADXL355 status
enum class Adxl355_Status : uint8_t {
  /// @brief a complete x-axis, y-axis, and z-axis measurement was made and results can be read
  dataReady = 0x01,
  /// @brief FIFO watermark is reached
  fifoFull = 0x02,
  /// @brief FIFO has overrun, and the oldest data is lost
  fifoOverrun = 0x04,
  /// @brief acceleration activity is detected
  activity = 0x08,
  /// @brief NVM controller is busy with a refresh, programming, or a built in self test
  nvmBusy = 0x10
};
Adxl355_Status operator|(Adxl355_Status status1, Adxl355_Status status2);
Adxl355_Status operator&(Adxl355_Status status1, Adxl355_Status status2);

//==============================================================================

/// @brief One or multiple ADXL355 axes
enum class Adxl355_Axes : uint8_t {
  /// @brief no axis
  none = 0,
  /// @brief X-axis
  x = 0x01,
  /// @brief Y-axis
  y = 0x02,
  /// @brief Z-axis
  z = 0x04
};
Adxl355_Axes operator|(Adxl355_Axes axes1, Adxl355_Axes axes2);
Adxl355_Axes operator&(Adxl355_Axes axes1, Adxl355_Axes axes2);

//==============================================================================

/// @brief ADXL355 high-pass filter frequency
enum class Adxl355_HpfFrequency : uint8_t {
  /// @brief high-pass filter disabled
  none = 0x00,
  /// @brief ODR*24.700e-4
  hpf24_7 = 0x01,
  /// @brief ODR*6.2084e-4
  hpf6_2084 = 0x02,
  /// @brief ODR*1.5545e-4
  hpf1_5545 = 0x03,
  /// @brief ODR*0.3862e-4
  hpf0_3862 = 0x04,
  /// @brief ODR*0.0954e-4
  hpf0_0954 = 0x05,
  /// @brief ODR*0.0238e-4
  hpf0_0238 = 0x06
};

//==============================================================================

/// @brief ADXL355 output data rate
enum class Adxl355_OutputDataRate : uint8_t {
  /// @brief 4000 Hz (low-pass filter: 1000 Hz)
  odr4000 = 0x00,
  /// @brief 2000 Hz (low-pass filter: 500 Hz)
  odr2000 = 0x01,
  /// @brief 1000 Hz (low-pass filter: 250 Hz)
  odr1000 = 0x02,
  /// @brief 500 Hz (low-pass filter: 125 Hz)
  odr500 = 0x03,
  /// @brief 250 Hz (low-pass filter: 62.5 Hz)
  odr250 = 0x04,
  /// @brief 125 Hz (low-pass filter: 31.25 Hz)
  odr125 = 0x05,
  /// @brief 62.5 Hz (low-pass filter: 15.625 Hz)
  odr62_5 = 0x06,
  /// @brief 31.25 Hz (low-pass filter: 7.813 Hz)
  odr31_25 = 0x07,
  /// @brief 15.625 Hz (low-pass filter: 3.906 Hz)
  odr15_625 = 0x08,
  /// @brief 7.813 Hz (low-pass filter: 1.953 Hz)
  odr7_813 = 0x09,
  /// @brief 3.906 Hz (low-pass filter: 0.977 Hz)
  odr3_906 = 0x0A
};

//==============================================================================

/// @brief ADXL355 interrupts
enum class Adxl355_Interrupts : uint8_t {
  /// @brief no interrupts
  none = 0x00,
  /// @brief DATA_RDY interrupt enable on INT1
  dataReadyInt1 = 0x01,
  /// @brief FIFO_FULL interrupt enable on INT1
  fifoFullInt1 = 0x02,
  /// @brief FIFO_OVR interrupt enable on INT1
  fifoOverrunInt1 = 0x04,
  /// @brief activity interrupt enable on INT1
  activityInt1 = 0x08,
  /// @brief DATA_RDY interrupt enable on INT2
  dataReadyInt2 = 0x10,
  /// @brief FIFO_FULL interrupt enable on INT2
  fifoFullInt2 = 0x20,
  /// @brief FIFO_OVR interrupt enable on INT2
  fifoOverrunInt2 = 0x40,
  /// @brief activity interrupt enable on INT2
  activityInt2 = 0x80
};
Adxl355_Interrupts operator|(Adxl355_Interrupts int1, Adxl355_Interrupts int2);
Adxl355_Interrupts operator&(Adxl355_Interrupts int1, Adxl355_Interrupts int2);

//==============================================================================

/// @brief ADXL355 synchronization
enum class Adxl355_Synchronization : uint8_t {
  /// @brief internal
  internal = 0x00,
  /// @brief external
  external = 0x01,
  /// @brief external with interpolation filter
  externalWithInterpolation = 0x02
};

//==============================================================================

/// @brief ADXL355 acceleration range
enum class Adxl355_Range : uint8_t {
  /// @brief ±2 g
  range2g = 0x01,
  /// @brief ±4 g
  range4g = 0x02,
  /// @brief ±8 g
  range8g = 0x03
};

//==============================================================================

/// @brief ADXL355 interrupt polarity
enum class Adxl355_InterruptPolarity : uint8_t {
  /// @brief fast
  activeLow = 0x00,
  /// @brief high-speed
  activeHigh = 0x01
};

//==============================================================================

/// @brief ADXL355 acceleration range
enum class Adxl355_I2CSpeed : uint8_t {
  /// @brief fast
  fast = 0x00,
  /// @brief high-speed
  highSpeed = 0x01
};

//==============================================================================

/// @brief ADXL355 device information
struct Adxl355_DeviceInfo {
  /// @brief Vendor ID
  uint8_t vendorId;
  /// @brief Device family ID
  uint8_t familyId;
  /// @brief Device ID
  uint8_t deviceId;
  /// @brief Revision ID
  uint8_t revisionId;
};

//==============================================================================

/// @brief ADXL355 raw accelerations
struct Adxl355_RawAccelerations {
  /// @brief Raw X-axis acceleration
  int32_t x;
  /// @brief Raw Y-axis acceleration
  int32_t y;
  /// @brief Raw Z-axis acceleration
  int32_t z;

  Adxl355_RawAccelerations();
  Adxl355_RawAccelerations(int32_t x, int32_t y, int32_t z);
};

//==============================================================================

/// @brief ADXL355 accelerations in g
struct Adxl355_Accelerations {
  /// @brief X-axis acceleration, g
  float x;
  /// @brief Y-axis acceleration, g
  float y;
  /// @brief Z-axis acceleration, g
  float z;

  Adxl355_Accelerations();
  Adxl355_Accelerations(float x, float y, float z); 
};

//==============================================================================

}