# WIRING NOTES FOR ARDUINO MEGA AND WAVESHARE DISPLAY BOARD

## NON-CONFIGURABLE
MOSI_PIN  51   // connect to the MOSI pin of the display module
SCK       52   // connect to SCK pin (Serial Clock) of the display module

## CONFIGURABLE
CS_PIN    10   // Choose your SS (Slave Select) pin
DC_PIN    9    // Choose your DC (Data/Command) pin
RST_PIN   8    // Choose your RST (Reset) pin
BUSY_PIN  7    // Choose your Busy pin (if available)

## VCC AND GND
connect display pin VCC to Arduino 5V
connect display pin GND to Arduino GND