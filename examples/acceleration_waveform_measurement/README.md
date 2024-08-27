# ADXL355 Acceleration Waveform Measurement Example

1. SPI interface is initialized (host: HSPI_HOST, MOSI pin: 5, MISO pin: 18, SCLK pin: 19).
2. ADXL355 is initialized (frequency: 10 MHz, CS pin: 21).
3. Range is set to ±2 g.
4. Measurement frequency is set to 4000 Hz.
5. Measurement is enabled.
6. 10 time points of X-, Y- and Z-axis accelerations are read from the FIFO and printed every 5 seconds.