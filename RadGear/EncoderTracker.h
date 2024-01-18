#ifndef ENCODERTRACKER_H
#define ENCODERTRACKER_H

#include <Arduino.h>
// #include <vector>

#define NUM_FILTER 3 // this must be at least 2
#define IS_MOVING_SPEED_THRESHOLD 50.0

class EncoderTracker
{
public:
    EncoderTracker(int tStep) : timeStep_ms(tStep) {}
    // takes in actual current encoder value and outputs current degrees, handles rollover
    double calculatePosition(double rawEncoderValue)
    {
        encoderValue = rawEncoderValue - zeroOffsetEncoderValue;

        // Calculate the change in encoder value since the previous reading
        int encoderChange = encoderValue - previousEncoderValue;

        // Handle rollover when the encoder value goes from 359 to 0 or vice versa
        if (encoderChange > 180.0)
        {
            encoderChange -= 360.0;
        }
        else if (encoderChange < -180.0)
        {
            encoderChange += 360.0;
        }

        // Update the previous encoder value for the next calculation
        previousEncoderValue = encoderValue;

        // Update the current degrees traveled without bounding
        currentPosition += encoderChange;

        return currentPosition;
    }
    // calculate average velocity from last few values and update isMoving status
    double calculateFilteredVelocity()
    {
        static int index = 0;
        sumFilteredVelocity -= storedPositions[index];
        sumFilteredVelocity += currentPosition;
        index++;
        if (index >= NUM_FILTER)
        {
            index = 0;
        }
        currentVelocity = 1000.0 * sumFilteredVelocity / (NUM_FILTER - 1) * timeStep_ms;

        if (abs(currentVelocity) >= IS_MOVING_SPEED_THRESHOLD)
        {
            isMoving = true;
        }
        else
        {
            isMoving = false;
        }

        return currentVelocity;
    }
    void zeroPosition()
    {
        zeroOffsetEncoderValue = encoderValue + zeroOffsetEncoderValue;
        previousEncoderValue = 0;
        currentPosition = 0.0;
    }

    double currentVelocity = 0.0; // filtered
    double currentPosition = 0;   // Current degrees traveled
    bool isMoving = false;

private:
    double timeStep_ms = 0.60;

    double previousEncoderValue = 0.0; // Previous encoder value
    double zeroOffsetEncoderValue = 0.0;
    double encoderValue = 0.0; // offset is applied

    double storedPositions[NUM_FILTER];
    double sumFilteredVelocity = 0.0;
};

#endif