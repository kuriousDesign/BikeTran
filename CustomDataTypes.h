#ifndef CUSTOMDATATYPES_H
#define CUSTOMDATATYPES_H

#include <Arduino.h>

// Info Types
enum InfoTypes : uint8_t
{
    SHIFT_DATA = 0,
    MOTION_DATA = 1,
    DIAGNOSTIC_DATA = 2,
};

#define MOTIONDATAPACKETSIZE 5 // the floats are rounded to int16_t during serial transmission
struct MotionData
{
    uint8_t actualGear = 1; // range is from 1 to NUM_GEARS, does not start at 0
    float actualPosition = 0.0;
    float actualVelocity = 0.0; // Initialize the stored position
};

#define SHIFTDATAPACKETSIZE 5 // 5 bytes
struct ShiftData
{
    uint8_t targetGear = 1; // range is from 1 to NUM_GEARS, does not start at 0
    int16_t targetPosition = 0;
    int16_t startingPosition = 0;
};

#define NUM_DIAGNOSTICS_ARRAY 800
struct DiagnosticData
{
    int16_t numOfDataPoints;
    int16_t cmd[NUM_DIAGNOSTICS_ARRAY];
    int16_t error[NUM_DIAGNOSTICS_ARRAY];
    uint8_t targetGear = 0;
    uint8_t actualGear = 0;
    int16_t targetPosition = 0;
    int16_t actualPosition = 0;
};

#endif