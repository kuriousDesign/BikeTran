#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define READRATE_uS 700 // read rate in microseconds

// ENCODER
/*
PAD     FUNCTION    COLOR
PIN 1   A Signal    BLK
PIN 2   B Signal    RED
PIN 3   NC          WHITE
PIN 4   NC          YELLOW
PIN 5   ENC PWR-    ORANGE
PIN 6   ENC PWR+    GREEN
/*

//RS485 to TTL CONVERTER
/*
5V    RED
TXD   YEL
RXD   ORG
GND   BLK
*/

#define maxDataSize 5     // Maximum size of the byte array
#define READ_ABS_POS 0x02 // the byte you send to encoder to read absolute position of single turn

class Encoder
{
private:
    int PWM_PIN = 0;
    const float CPR = 16384.0; // counts per rev

    byte dataArray[maxDataSize]; // Byte array to store received data
    int dataSize = 0;            // Current size of the byte array

    bool readReqFlag = false;         // when true, means a read request has been sent to encoder
    unsigned long readReqTime_us = 0; // time that the read request was sent, need to wait about 600us beforing reading

    bool readEncoder();
    void writeEncoderCmd(byte cmd);
    void writeReadCmd();
    float parsePositionData();
    bool debug;

public:
    void init();
    bool run();
    bool newReadingFlag = false;
    float position = 0; // current position of the encoder, valid range is between 0.0 and 359.99
    void setDebug(bool setOn);
    void debugPrintln(String msg);
    void debugPrint(String msg);
};

#endif