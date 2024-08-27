# ADXL355 Device Information and Self-Test Example

1. SPI interface is initialized (host: HSPI_HOST, MOSI pin: 5, MISO pin: 18, SCLK pin: 19).
2. ADXL355 is initialized (frequency: 10 MHz, CS pin: 21).
3. ADXL355 is reset.
4. Device information is read and printed.
5. Device self-test is executed and the results are printed.