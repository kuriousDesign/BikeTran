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

enum OperatingModes : uint8_t //
{
  AUTO = 0,
  MANUAL_CLUTCH_JOGGING = 1,
  MANUAL_LINEAR_P = 2,
  MANUAL_LINEAR_S = 3,
  IO_CHECKOUT = 4,
  MANUAL_CLUTCH_ENGAGE = 5,
  LINEAR_P_HOMING = 6,
  LINEAR_S_HOMING = 7,
};

String getOperatingModeToString(OperatingModes mode)
{
    switch (mode)
    {
    case AUTO:
        return "AUTO";

    case MANUAL_CLUTCH_JOGGING:
        return "MANUAL_CLUTCH_JOGGING";

    case MANUAL_LINEAR_P:
        return "MANUAL_LINEAR_P";

    case MANUAL_LINEAR_S:
        return "MANUAL_LINEAR_S";

    case IO_CHECKOUT:
        return "IO_CHECKOUT";

    case MANUAL_CLUTCH_ENGAGE:
        return "MANUAL_CLUTCH_ENGAGE";

    default:
        return "UNKNOWN";
    }
}

enum DiagnosticModes
{
  UI = 0,
  SERIAL_OUTPUT = 1,
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

#define NUM_DIAGNOSTICS_ARRAY 100
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
    String list[FAULT_DATA_LIST_LENGTH];
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

struct CmdData
{

};


#endif