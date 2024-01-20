# WIRING NOTES FOR ARDUINO MEGA AND WAVESHARE DISPLAY BOARD

## NON-CONFIGURABLE
DIN_PIN  51   // Blue Wire - DIN (MOSI) pin of the display module
CLK_PIN  52   // Yellow Wire - SCK pin (Serial Clock) of the display module

## CONFIGURABLE
CS_PIN    10   // Orange Wire - SS (Slave Select) pin
DC_PIN    9    // Green Wire - DC (Data/Command) pin
RST_PIN   8    // White Wire - RST (Reset) pin
BUSY_PIN  7    // Purple Wire - Busy pin (if available)

## VCC AND GND
connect display pin VCC to Arduino 5V (Gray Wire)
connect display pin GND to Arduino GND (Brown Wire)