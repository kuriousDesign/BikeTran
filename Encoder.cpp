#include "Encoder.h"
#include <Arduino.h>

extern bool FLIP_POSITIVE;

// ENCODER SETUP, set pinmode and setup pwm
void Encoder::init()
{
    // Initialize Serial1 for RS-485 communication with Encoder
    Serial1.begin(115200); // Set the baud rate to match your RS-485 network
    writeEncoderCmd(READ_ABS_POS);
}

void Encoder::writeEncoderCmd(byte cmd)
{
    Serial1.write(cmd);
}

void Encoder::writeReadCmd()
{
    Serial1.write(READ_ABS_POS);
}

// reads the encoder serial register and updates position, returns true if the read value is valid
bool Encoder::readEncoder()
{
    bool isValid = true;
    dataSize = 0;
    while (Serial1.available())
    {
        byte receivedByte = Serial1.read();
        // Serial.print(receivedByte);
        // Serial.print(" ");

        // Check if there's still space in the array to store the byte
        if (dataSize < maxDataSize)
        {
            dataArray[dataSize] = receivedByte;
            dataSize++;
        }
        else
        {
            isValid = false;
            debugPrintln("Data array is full!");
            // You can handle the array being full in your own way here
        }
    }
    if (isValid && dataSize == 5)
    {
        byte result;
        for (int i = 0; i < 5 - 1; i++)
        {
            result ^= dataArray[i]; // XOR each byte together
            float sign = 1.0;
            if (FLIP_POSITIVE)
            {
                sign = -1.0;
            }
            Encoder::position = sign * parsePositionData();
            // debugPrint("encoder raw position: ");
            // debugPrintln(String(Encoder::position));
        }
        if (result != dataArray[4])
        {
            // isValid = false;
            debugPrint("check bit mismatch, calculated result: ");
            debugPrint(String(result));
            debugPrint(" , checkbit: ");
            // Serial.println(dataArray[4]);
        }
    }
    else if (dataSize != 5)
    {
        isValid = false;
        debugPrint("dataSize was not equal to 5, it was: ");
        debugPrintln(String(dataSize));
    }
    return isValid;
}

float Encoder::parsePositionData()
{
    if (dataArray[0] != READ_ABS_POS)
    {
        // debugPrint("Control Field Byte did not match 0x02, it was : ");
        // debugPrintln(String(dataArray[0]));
    }
    uint16_t counts = dataArray[2] + (dataArray[3] << 8);
    return (float(counts) / CPR) * 360.0; // output angle between 0.0 and 359.9 deg
}

// this will read the encoder at the controlled rate, returns true if new encoder value is ready
bool Encoder::run()
{
    unsigned long timeNow = micros();
    newReadingFlag = false;
    bool attemptedReading = false;

    if (readReqFlag && timeNow - READRATE_uS >= readReqTime_us)
    {
        readReqFlag = false;
        if (Encoder::readEncoder())
        {
            newReadingFlag = true;
        }
        attemptedReading = true;
    }

    if (!readReqFlag)
    {
        Encoder::writeReadCmd();
        readReqTime_us = timeNow;
        readReqFlag = true;
    }

    return attemptedReading;
}

void Encoder::debugPrint(String msg)
{
    if (debug)
    {
        Serial.print(msg);
    }
}

void Encoder::debugPrintln(String msg)
{
    if (debug)
    {
        Serial.println(msg);
    }
}
void Encoder::setDebug(bool setOn)
{
    debug = setOn;
    debugPrintln("encoder debug now available");
}