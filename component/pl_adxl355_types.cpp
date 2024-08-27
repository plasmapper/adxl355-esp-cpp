#include "pl_adxl355_types.h"

//==============================================================================

namespace PL {

//==============================================================================

Adxl355_Status operator|(Adxl355_Status status1, Adxl355_Status status2) {
  return (Adxl355_Status)((uint8_t)status1 | (uint8_t)status2);
}

//==============================================================================

Adxl355_Status operator&(Adxl355_Status status1, Adxl355_Status status2) {
  return (Adxl355_Status)((uint8_t)status1 & (uint8_t)status2);
}

//==============================================================================

Adxl355_Axes operator|(Adxl355_Axes axis1, Adxl355_Axes axis2) {
  return (Adxl355_Axes)((uint8_t)axis1 | (uint8_t)axis2);
}

//==============================================================================

Adxl355_Axes operator&(Adxl355_Axes axis1, Adxl355_Axes axis2) {
  return (Adxl355_Axes)((uint8_t)axis1 & (uint8_t)axis2);
}

//==============================================================================

Adxl355_Interrupts operator|(Adxl355_Interrupts axis1, Adxl355_Interrupts axis2) {
  return (Adxl355_Interrupts)((uint8_t)axis1 | (uint8_t)axis2);
}

//==============================================================================

Adxl355_Interrupts operator&(Adxl355_Interrupts axis1, Adxl355_Interrupts axis2) {
  return (Adxl355_Interrupts)((uint8_t)axis1 & (uint8_t)axis2);
}

//==============================================================================

Adxl355_RawAccelerations::Adxl355_RawAccelerations() : x(0), y(0), z(0) {}

//==============================================================================

Adxl355_RawAccelerations::Adxl355_RawAccelerations(int32_t x, int32_t y, int32_t z) : x(x), y(y), z(z) {}

//==============================================================================

Adxl355_Accelerations::Adxl355_Accelerations() : x(0), y(0), z(0) {}

//==============================================================================

Adxl355_Accelerations::Adxl355_Accelerations(float x, float y, float z) : x(x), y(y), z(z) {}

//==============================================================================

}