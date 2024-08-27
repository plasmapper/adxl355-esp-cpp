# ADXL355 Activity Detection Example

1. SPI interface is initialized (host: HSPI_HOST, MOSI pin: 5, MISO pin: 18, SCLK pin: 19).
2. ADXL355 is initialized (frequency: 10 MHz, CS pin: 21).
3. Range is set to ±2 g.
4. Activity detection is enabled for X and Y axes with a threshold of 1.5 g and count 5.
5. Measurement is enabled.
6. Activity detection is checked every second and the results are printed.