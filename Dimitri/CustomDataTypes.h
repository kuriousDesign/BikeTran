#ifndef CUSTOMDATATYPES_H
#define CUSTOMDATATYPES_H

#include <Arduino.h>

enum Modes : int32_t //
{
    ABORTING = -3,
    KILLED = -2,
    ERROR = -1,
    INACTIVE = 0,
    RESETTING = 50,
    IDLE = 100,
    SHIFTING = 200, // while shifting, the controller is active
    MANUAL = 1100,
};

enum Events : int
{
    NONE_EVENT = 0,
    UP_SHIFT_REQ = 1,
    SUPER_UP_SHIFT_REQ = 2,
    DOWN_SHIFT_REQ = 3,
    SUPER_DOWN_SHIFT_REQ = 4,
};

// Info Types
enum InfoTypes : uint8_t
{
    SHIFT_DATA = 0,
    MOTION_DATA = 1,
    DIAGNOSTIC_DATA = 2,
    ERROR_DATA = 3,
    WARNING_DATA = 4,
};

#define MOTIONDATAPACKETSIZE 5 // the floats are rounded to int16_t during serial transmission
struct MotionData
{
    int8_t actualGear = 1; // range is from 1 to NUM_GEARS, does not start at 0
    //float actualPosition = 0.0;
    //float actualVelocity = 0.0; // Initialize the stored position
    //float targetPosition = 0.0;
};

#define SHIFTDATAPACKETSIZE 5 // 5 bytes
struct ShiftData
{
    int8_t targetGear = 1; // range is from 1 to NUM_GEARS, does not start at 0
    //int16_t targetPosition = 0;
    //int16_t startingPosition = 0;
};

#define NUM_DIAGNOSTICS_ARRAY 900
struct DiagnosticData
{
    int16_t numOfDataPoints;
    int16_t cmd[NUM_DIAGNOSTICS_ARRAY];
    int16_t error[NUM_DIAGNOSTICS_ARRAY];
    int8_t targetGear = 0;
    int8_t actualGear = 0;
    int16_t targetPosition = 0;
    int16_t actualPosition = 0;
};

#define FAULT_DATA_LIST_LENGTH 4
#define FAULTDATAPACKETSIZE (1 + FAULT_DATA_LIST_LENGTH)
struct FaultData
{
    bool present = false;
    uint8_t list[FAULT_DATA_LIST_LENGTH];
};

enum Errors : uint8_t //
{
    NONE_ERROR = 0,
    CONTROLLER_SHIFT_TIMED_OUT = 1,
    CONTROLLER_FAULT_DURING_HOMING = 2,
    HOMING_NUDGE_RETRIES_EXCEEDED = 3,
    ENCODER_MISREAD_COUNT_EXCEEDED = 4,
    MOTOR_NOT_AT_TARGET_WHILE_IDLE = 5,
    HOMING_ROUTINE_ERROR = 50, //reserved 51- 59 for homing errors

};

enum Warnings : uint8_t //
{
    NONE_WARNING = 0,
};

struct StopWatch
{
    unsigned long startTime = 0;
    unsigned long stopTime = 0;
    unsigned long prevScanTime = 0;
    unsigned int maxLoopTime = 0;
    unsigned int loopCnt = 0;
};

#endif