#ifndef ENCODERTRACKER_H
#define ENCODERTRACKER_H

#include <Arduino.h>
// #include <vector>

#define NUM_FILTER 3

class EncoderTracker
{
public:
    EncoderTracker() : currentDegrees(0), previousEncoderValue(0) {}
    // takes in actual current encoder value and outputs current degrees, handles rollover
    double calculateDegrees(int rawEncoderValue)
    {
        encoderValue = rawEncoderValue - zeroOffsetEncoderValue;

        // Calculate the change in encoder value since the previous reading
        int encoderChange = encoderValue - previousEncoderValue;

        // Handle rollover when the encoder value goes from 359 to 0 or vice versa
        if (encoderChange > 180)
        {
            encoderChange -= 360;
        }
        else if (encoderChange < -180)
        {
            encoderChange += 360;
        }

        // Update the previous encoder value for the next calculation
        previousEncoderValue = encoderValue;

        // Update the current degrees traveled without bounding
        currentDegrees += encoderChange;

        return currentDegrees;
    }
    //
    double calculateFilteredVelocity(double currentPosition)
    {
        static int index = 0;
        sumFilteredVelocity -= storedPositions[index];
        sumFilteredVelocity += currentPosition;
        index++;
        return 1000.0 * sumFilteredVelocity / (NUM_FILTER - 1) * timeStep_ms;
    }
    void zeroPosition()
    {
        zeroOffsetEncoderValue = encoderValue + zeroOffsetEncoderValue;
        previousEncoderValue = 0;
        currentDegrees = 0.0;
    }

private:
    double timeStep_ms = 0.60;
    double currentDegrees = 0;    // Current degrees traveled
    int previousEncoderValue = 0; // Previous encoder value
    int zeroOffsetEncoderValue = 0;
    int encoderValue = 0;
    double currentVelocity = 0.0;
    double storedPositions[NUM_FILTER];
    double sumFilteredVelocity;
};

#endif