#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_EPD.h>

#define EPD_CS 10
#define EPD_DC 9
#define EPD_RESET 8
#define EPD_BUSY 7

Adafruit_SSD1680 epd(200, 200, EPD_DC, EPD_RESET, EPD_CS, EPD_BUSY);

void setup()
{
}

void loop()
{
    epd.clearBuffer();
    epd.setTextSize(1);
    epd.setCursor(10, 10);
    epd.setTextColor(EPD_BLACK);
    epd.print("Hello, e-Paper!");

    epd.display();
    delay(5000); // Pause for 5 seconds
}
