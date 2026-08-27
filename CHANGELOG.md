# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Documentation that configuration setters other than SetHpfFrequency and SetInterrupts should not be called while measurement is enabled.

### Fixed
- Adxl355::ClearFifo exceeding the maximum SPI transaction size.
- Invalid SPI mode value.

## [1.2.1] - 2026-08-20
### Fixed
- Adxl355_InterruptPolarity and Adxl355_I2CSpeed documentation.

## [1.2.0] - 2026-08-19
### Added
- ReadShadowRegisters method.

## [1.1.2] - 2026-08-19
### Fixed
- ClearFifo possible infinite loop.
- ReadAccelerationScaleFactor not returning an error for an unexpected range value.
- SelfTest not setting a specific output data rate.
- Write src parameter not being const.
- Array arguments passed as &array.
- SetRawOffsets and SetRawActivityDetectionThreshold not clamping the arguments.

## [1.1.1] - 2026-08-13
### Fixed
- FIFO reading algorithm.

## [1.1.0] - 2026-08-12
### Added
- Timeout parameter to ReadRawAccelerationsFromFifo and ReadAccelerationsFromFifo.
- Tests for ReadStatus, ClearFifo, ReadAccelerations, ReadTemperature and SelfTest output values.

### Removed
- Redundant standby-mode acceleration read in SelfTest.

### Fixed
- SelfTest not restoring the acceleration range.
- SelfTest not clearing the FIFO.
- Write null check error message referring to dest instead of src.
- ReadRawAccelerationsFromFifo not validating the Y- and Z-axis FIFO status bits.

## [1.0.0] - 2024-08-27
Initial release.