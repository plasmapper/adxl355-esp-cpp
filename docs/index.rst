ADXL355 Component
=================

.. |COMPONENT| replace:: adxl355

.. |ESP_IDF_VERSION| replace:: 5.3
   
.. |VERSION| replace:: 1.0.0

.. include:: ../../../installation.rst

.. include:: ../../../sdkconfig_common.rst


Functions
---------

Initialization
^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::Adxl355`, :cpp:func:`PL::Adxl355::Initialize`.

Range Configuration and Scale Factor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadRange`, :cpp:func:`PL::Adxl355::SetRange`, :cpp:func:`PL::Adxl355::ReadAccelerationScaleFactor`.

Measurement Frequency Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadOutputDataRate`, :cpp:func:`PL::Adxl355::SetOutputDataRate`.

Measurement Enable/Disable
^^^^^^^^^^^^^^^^^^^^^^^^^^
Use :cpp:func:`PL::Adxl355::EnableMeasurement`, :cpp:func:`PL::Adxl355::DisableMeasurement`,
:cpp:func:`PL::Adxl355::IsMeasurementEnabled`.

Single Measurement
^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadAccelerations`/:cpp:func:`PL::Adxl355::ReadRawAccelerations`.

Waveform Measurement
^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadNumberOfFifoSamples`, :cpp:func:`PL::Adxl355::ClearFifo`,
:cpp:func:`PL::Adxl355::ReadAccelerationsFromFifo`/:cpp:func:`PL::Adxl355::ReadRawAccelerationsFromFifo`.

Offset Configuration
^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadOffsets`/:cpp:func:`PL::Adxl355::ReadRawOffsets`,
:cpp:func:`PL::Adxl355::SetOffsets`/:cpp:func:`PL::Adxl355::SetRawOffsets`.

Activity Detection
^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadActivityDetectionAxes`, :cpp:func:`PL::Adxl355::SetActivityDetectionAxes`,
:cpp:func:`PL::Adxl355::ReadActivityDetectionThreshold`/:cpp:func:`PL::Adxl355::ReadRawActivityDetectionThreshold`,
:cpp:func:`PL::Adxl355::SetActivityDetectionThreshold`/:cpp:func:`PL::Adxl355::SetRawActivityDetectionThreshold`,
:cpp:func:`PL::Adxl355::ReadActivityDetectionCount`, :cpp:func:`PL::Adxl355::SetActivityDetectionCount`.

Temperature Measurement
^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::EnableTemperature`, :cpp:func:`PL::Adxl355::DisableTemperature`,
:cpp:func:`PL::Adxl355::IsTemperatureEnabled`,
:cpp:func:`PL::Adxl355::ReadTemperature`/:cpp:func:`PL::Adxl355::ReadRawTemperature`.

Interrupt Configuration
^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadInterrupts`, :cpp:func:`PL::Adxl355::SetInterrupts`.

Device Information and Status
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::ReadDeviceInfo`, :cpp:func:`PL::Adxl355::ReadStatus`.

Self-test and Reset
^^^^^^^^^^^^^^^^^^^
:cpp:func:`PL::Adxl355::SelfTest`, :cpp:func:`PL::Adxl355::Reset`.

Other
^^^^^
:cpp:func:`PL::Adxl355::ReadHpfFrequency`, :cpp:func:`PL::Adxl355::SetHpfFrequency`,
:cpp:func:`PL::Adxl355::ReadFifoWatermark`, :cpp:func:`PL::Adxl355::SetFifoWatermark`,
:cpp:func:`PL::Adxl355::ReadSynchronization`, :cpp:func:`PL::Adxl355::SetSynchronization`,
:cpp:func:`PL::Adxl355::EnableExternalClock`, :cpp:func:`PL::Adxl355::DisableExternalClock`,
:cpp:func:`PL::Adxl355::IsExternalClockEnabled`
:cpp:func:`PL::Adxl355::ReadInterruptPolarity`, :cpp:func:`PL::Adxl355::SetInterruptPolarity`,
:cpp:func:`PL::Adxl355::ReadI2CSpeed`, :cpp:func:`PL::Adxl355::SetI2CSpeed`,
:cpp:func:`PL::Adxl355::EnableDataReady`, :cpp:func:`PL::Adxl355::DisableDataReady`,
:cpp:func:`PL::Adxl355::IsDataReadyEnabled`.

Thread safety
-------------

Class method thread safety is implemented by having the :cpp:class:`PL::Lockable` as a base class and creating the class object lock guard at the beginning of the methods.

::cpp:func:`PL::Adxl355::Initialize` and all transaction methods lock both
the :cpp:class:`PL::Adxl355` and the :cpp:class:`PL::Spi` objects for the duration of the method.

Examples
--------
| `Acceleration measurement <https://components.espressif.com/components/plasmapper/pl_adxl355/versions/1.0.0/examples/acceleration_measurement>`_
| `Acceleration waveform measurement <https://components.espressif.com/components/plasmapper/pl_adxl355/versions/1.0.0/examples/acceleration_waveform_measurement>`_
| `Activity detection <https://components.espressif.com/components/plasmapper/pl_adxl355/versions/1.0.0/examples/activity_detection>`_
| `Device info and self test <https://components.espressif.com/components/plasmapper/pl_adxl355/versions/1.0.0/examples/device_info_and_self_test>`_

API reference
-------------

.. toctree::
  
  api/adxl355