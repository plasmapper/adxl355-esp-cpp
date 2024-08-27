# ADXL355 Acceleration Measurement Example

1. SPI interface is initialized (host: HSPI_HOST, MOSI pin: 5, MISO pin: 18, SCLK pin: 19).
2. ADXL355 is initialized (frequency: 10 MHz, CS pin: 21).
3. Range is set to ±2 g.
4. Measurement is enabled.
5. X-, Y- and Z-axis accelerations are measured and printed every second.