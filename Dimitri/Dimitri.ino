#define VERSION_NUMBER 35

// #define SCAN_TIME_US 500  // how frequently the loop updates
#define UPDATE_TIME_US 400 // time that the motor velocities are updated, motor run() are called at half this rate
#define NUM_MOTORS 3

enum Motors
{
  CLUTCH = 0,
  LINEAR_P = 1,
  LINEAR_S = 2,
};

enum DiagnosticModes
{
  UI = 0,
  SERIAL_OUTPUT = 1,
};

#include <Arduino.h>
#include "Arduino.h"
// #include "ISerial.h"
#include "Encoder.h"
#include "Shifter.h"
#include "StateManager.h"
#include "CustomDataTypes.h"
#include "GearMap.h"
#include "Motor.h"
// #include "TimerOne.h"
#include "SerialLogging.h"

// OPERATING MODES: IO_CHECKOUT, MANUAL_CLUTCH_JOGGING, MANUAL_CLUTCH_ENGAGE, MANUAL_LINEAR_P, MANUAL_LINEAR_S, AUTO
const OperatingModes OPERATING_MODE = OperatingModes::MANUAL_CLUTCH_JOGGING; // set to OperatingModes::AUTO to run the system in debug mode
const DiagnosticModes DIAGNOSTIC_MODE = DiagnosticModes::SERIAL_OUTPUT;

const double MANUAL_LINEAR_JOG_PWR = 30.0;
const double MANUAL_CLUTCH_JOG_PWR = 70.0;
const double LinearHomingPwr = 100.0;
const int LinearNudgeTimeMsDuringHomingJog = 50;
const double ClutchHoldingPwr = 25.0;
const int clutchSolenoidJostleTimeMs = 20;
const double LinearKp = 500.0; //@11.05VDC
const double LinearKd = 0.0;
uint16_t LinearNudgeTimeMs = 5; // 15
const double LinearNudgePower = 80.0;
const double POSITION_CLUTCH_PEDALING = 0.0;   // deg
const double POSITION_CLUTCH_SHIFTING = 228.0; // deg
const double LINEAR_P_HOME_OFFSET = 6.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear
const double LINEAR_S_HOME_OFFSET = 0.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear

const unsigned long DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS = 200; // ms, time to wait after clutch reaches shifting position before linear motor motion is allowed
const bool AUTO_RESET = false;                                                        // if this is true, the system will automatically reset when inactive (no errors)
const bool SHIFT_BUTTONS_DISABLED = false;                                            // if this is true, the shift buttons will be disabled (ignored)
const bool SIM_MODE = false;                                                          // set to true to simulate motor behavior (encoders positions for now, TODO: simulate lim switches)
uint8_t motorId = 0;

////////////////////////////////////////////////////
// PINOUT
////////////////////////////////////////////////////

// MEGA PWM PINS: 2,3,4,5,6,7,8,9,10,11,12,13,44,45,46
// MEGA EXTERNAL INTERRUPTS FOR ENCODER A SIGNAL PINS: 2, 3, 18, 19, 20, 21 (B SIGNAL CAN USE NORMAL DIG INPUTS)

// LINEAR MOTOR (P) PRIMARY - WIRED TO SHIELD MOTOR A - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
#define PIN_LINEAR_P_PWM 3
#define PIN_LINEAR_P_DIR 12

// LINEAR MOTOR S (SECONDARY) - WIRED TO SHIELD MOTOR B - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
#define PIN_LINEAR_S_PWM 11
#define PIN_LINEAR_S_DIR 13

// CLUTCH MOTOR - WIRED TO STANDALONE MOTOR BOARD A - RED MOTOR WIRE IS CONNECTED TO OUT1 AND BLACK TO OUT2
#define PIN_CLUTCH_PWM 44     // --> wires to ENA pin of L298N driver board
#define PIN_CLUTCH_DIR_IN1 45 // --> wires to IN1 of L298N driver board
#define PIN_CLUTCH_DIR_IN2 46 // --> wires to IN2 of L298N driver board

#define PIN_LINEAR_P_ENC_A 18 // green wire of enc
#define PIN_LINEAR_P_ENC_B 19 // yellow wire of enc

#define PIN_LINEAR_S_ENC_A 20 // green wire of enc
#define PIN_LINEAR_S_ENC_B 52 // yellow wire of enc

#define PIN_CLUTCH_ENC_A 21 // green wire of enc??
#define PIN_CLUTCH_ENC_B 2  // yellow wire of enc??

#define PIN_EINK_BIT0 22 // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER DISPLAY ON E-INK
#define PIN_EINK_BIT1 24
#define PIN_EINK_BIT2 26
#define PIN_EINK_BIT3 28
#define NUM_BITS 4

#define PIN_CLUTCH_POS_LIM 30 // detects 228 deg
#define PIN_CLUTCH_NEG_LIM 31 // detects 0 deg
#define PIN_SHIFT_UP 32       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 33     // RED WIRE ON SHIFTER

#define PIN_CLUTCH_SOL -1 // not used

struct Inputs
{
  bool ShiftDownSw = false;
  bool ShiftUpSw = false;
  bool ClutchNegLimSw = false;
  bool ClutchPosLimSw = false;
};
Inputs inputs;

////////////////////////////////////////////////////
// OUTPUTS
////////////////////////////////////////////////////

struct Outputs
{
  bool DisplayBits[NUM_BITS] = {false, false, false, false};
};
Outputs outputs;

// ENCODERS
// NOTE: follow Motors enum for order below
// Example with rollover support
Encoder encoders[NUM_MOTORS] = {
    Encoder(PIN_CLUTCH_ENC_A, PIN_CLUTCH_ENC_B), // rollover handling done in Motor class
    Encoder(PIN_LINEAR_P_ENC_A, PIN_LINEAR_P_ENC_B),
    Encoder(PIN_LINEAR_S_ENC_A, PIN_LINEAR_S_ENC_B)};

// 394.68deg reading after 360.0 deg rotation.
const double CLUTCH_PULSES_PER_UNIT = 2.0 * 445.12 / 360.0; // 480.0 / 360.0; //120PPR /360deg for the 600rpm motor
// notes from joe on AUG 19 2025 - 12mm per turn, 6.62 mm per gear
const double LINEAR_P_PULSES_PER_UNIT = 1.0; // 14.0 / 11.0 * 2.0 * 8600.0 / (double(NUM_GEARS) - 1.0); // 9200 is the max position, 488 is the min position
// 12 mm per turn, 6.13mm per gear
const double LINEAR_S_PULSES_PER_UNIT = LINEAR_P_PULSES_PER_UNIT;

Motor::Cfg clutchMotorCfg = {
  name : "clutch",                             // name
  homingDir : Motor::HomingDir::POSITIVE,      // homing direction
  homingType : Motor::HomingType::HOME_SWITCH, // homing type
  homeOffsetFromZero : 0.0,                    // home offset from zero
  homingPwr : 100.0,
  unit : "deg",
  pulsesPerUnit : CLUTCH_PULSES_PER_UNIT, // unit and pulses per unit
  maxVelocity : 1000.0,                   // max velocity
  softLimitPositive : 721.0,              // soft limit positive
  softLimitNegative : -721.0,             // soft limit negative
  invertEncoderDir : false,               // invert encoder
  encoderRollover : false,                // enable encoder rollover
  invertMotorDir : true,                  // invert motor direction
  positionTol : 3.0,                      // position tolerance
  zeroVelocityTol : 5.0,                  // zero velocity tolerance, units/sec
  kP : 10.0,                              // kP
  kD : 1.0,                               // kD
  nudgeTimeMs : 50.0,                     // nudge time
  nudgePower : 100.0                      // nudge power

};

Motor::Cfg linearPrimaryMotorCfg = {
  name : "linear_primary",
  homingDir : Motor::HomingDir::POSITIVE,
  homingType : Motor::HomingType::HARDSTOP,
  homeOffsetFromZero : LINEAR_P_HOME_OFFSET, // units
  homingPwr : 30.0,
  unit : "gear",
  pulsesPerUnit : LINEAR_P_PULSES_PER_UNIT,
  maxVelocity : 20.0,
  softLimitPositive : (6 + 0.5),
  softLimitNegative : 0.5,
  invertEncoderDir : true,
  encoderRollover : false,
  invertMotorDir : true,
  positionTol : 0.02,
  zeroVelocityTol : 0.05,
  kP : LinearKp,
  kD : LinearKd,
  nudgeTimeMs : LinearNudgeTimeMs,
  nudgePower : LinearNudgePower
};

Motor::Cfg linearSecondaryMotorCfg = {
  name : "linear_secondary",
  homingDir : Motor::HomingDir::NEGATIVE,
  homingType : Motor::HomingType::HARDSTOP,
  homeOffsetFromZero : LINEAR_S_HOME_OFFSET, // units
  homingPwr : 30.0,
  unit : "gear",
  pulsesPerUnit : LINEAR_S_PULSES_PER_UNIT,
  maxVelocity : 20.0,
  softLimitPositive : (3 + 0.5),
  softLimitNegative : 0.5,
  invertEncoderDir : false,
  encoderRollover : false,
  invertMotorDir : true,
  positionTol : 0.02,
  zeroVelocityTol : 0.05,
  kP : LinearKp,
  kD : LinearKd,
  nudgeTimeMs : LinearNudgeTimeMs,
  nudgePower : LinearNudgePower
};

// NOTE: follow Motors enum for order below
Motor::Cfg motorCfgs[NUM_MOTORS] = {
    // name, homeDir, homeType, unit, pulsesPerUnit, maxVelocity, softLimitPositive, softLimitNegative, invertEncoder, invertMotorDir, positionTol, zeroVelocityTol, kP, kD, nudgeTimeMs, nudgePower
    clutchMotorCfg,
    linearPrimaryMotorCfg,
    linearSecondaryMotorCfg};

// NOTE: follow Motors enum for order below
Motor motors[NUM_MOTORS] = {
    Motor(PIN_CLUTCH_DIR_IN1, PIN_CLUTCH_PWM, &encoders[Motors::CLUTCH], &motorCfgs[Motors::CLUTCH], SIM_MODE, PIN_CLUTCH_DIR_IN2, PIN_CLUTCH_NEG_LIM),
    Motor(PIN_LINEAR_P_DIR, PIN_LINEAR_P_PWM, &encoders[Motors::LINEAR_P], &motorCfgs[Motors::LINEAR_P], SIM_MODE),
    Motor(PIN_LINEAR_S_DIR, PIN_LINEAR_S_PWM, &encoders[Motors::LINEAR_S], &motorCfgs[Motors::LINEAR_S], SIM_MODE)};

MotionData motionData;
GearMap gearMap = GearMap();

bool sw = false;
int time_now;
long last_time = millis();

long shiftTimeMs = 0;
long shiftStartTimeMs = 0;

// SHIFTER
Shifter shifter(PIN_SHIFT_UP, PIN_SHIFT_DOWN);
bool gearChangeReq = false;
int shiftTargetGearParam = 0;

// SERIAL SHIFTER DATA
int serialShiftReqType = 0;
int serialShiftTargetGearParam = 0;

// SHIFT DATA
ShiftData shiftData;

// PDController controller = PDController(1.20, 0.000); // NOTE: modify these parameters to improve the control, start with pd set to zero

bool atTarget = false;

// ISERIAL
// ISerial iSerial;
long tempLong;
float tempFloat;

// FAULTS
FaultData errors;

// DIAGNOSTICS
bool recording = false;
unsigned long lastDisplayed_ms = 0;
DiagnosticData diagnosticData;
// int16_t cmdRefArray[NUM_DIAGNOSTICS_ARRAY];

// int i_d = 0;
String jsonString = "";

// LOOP SCAN RATE
unsigned long lastUpdateUs = 0;

const bool SOL_ON = true;
StateManager loopState = StateManager("mainLoop");
StateManager clutchDeviceState = StateManager("clutchEngageDisengageState");

void setup()
{
  SerialLogging::init();
  if (DIAGNOSTIC_MODE == DiagnosticModes::SERIAL_OUTPUT)
  {
    Serial.println("");
    String versionNumberString = "VERSION NUMBER ---> " + String(VERSION_NUMBER);
    SerialLogging::info(versionNumberString.c_str());
    String operatingModeString = "OPERATING MODE ---> " + getOperatingModeToString(OPERATING_MODE);
    SerialLogging::info(operatingModeString.c_str());
    loopState.SetDebug(true);
    clutchDeviceState.SetDebug(true);
    motors[Motors::CLUTCH].setDebug(true);
    motors[Motors::LINEAR_P].setDebug(true);
    motors[Motors::LINEAR_S].setDebug(true);
  }
  loopState.transitionToStep(Modes::ABORTING);
  pinMode(PIN_CLUTCH_POS_LIM, INPUT_PULLUP);
  pinMode(PIN_CLUTCH_NEG_LIM, INPUT_PULLUP);
  pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
  pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
  pinMode(PIN_CLUTCH_DIR_IN1, OUTPUT);
  pinMode(PIN_CLUTCH_DIR_IN2, OUTPUT);
  pinMode(PIN_CLUTCH_PWM, OUTPUT);
  pinMode(PIN_LINEAR_P_PWM, OUTPUT);
  pinMode(PIN_LINEAR_P_DIR, OUTPUT);
  pinMode(PIN_LINEAR_S_PWM, OUTPUT);
  pinMode(PIN_LINEAR_S_DIR, OUTPUT);
  for (int i = 0; i < NUM_BITS; i++)
  {
    pinMode(PIN_EINK_BIT0 + 2 * i, OUTPUT);
  }

  shiftData.targetGear = 0;
  lastUpdateUs = micros();

  unsigned long timeNowMs = millis();
  while (OPERATING_MODE == OperatingModes::IO_CHECKOUT)
  {
    // SerialLogging::info("OPERATING MODE: IO_CHECKOUT");
    //  iSerial.debug = true;
    readInputs();
    Serial.println("state of clutch neg lim sw at the pedaling position: " + String(inputs.ClutchNegLimSw ? "ON" : "OFF"));
    Serial.println("state of clutch positive lim sw at the shifting position: " + String(inputs.ClutchPosLimSw ? "ON" : "OFF"));
    Serial.println("state of shift up switch: " + String(inputs.ShiftUpSw ? "ON" : "OFF"));
    Serial.println("state of shift down switch: " + String(inputs.ShiftDownSw ? "ON" : "OFF"));
    delay(1000);
  }
  // Timer1.initialize(UPDATE_TIME_US); // Initialize timer to trigger every X microseconds
  // Timer1.attachInterrupt(updateMotors, UPDATE_TIME_US);
}

// TASK: MOVE CLUTCH TO SHIFTING POSITION
// 0. VALIDATE CLUTCH MOTOR IS NEAR 0DEG
// 10. MOVE MOTOR TO DISENGAGED POSITION (CFG PARAM)
// 20. ACTIVELY HOLD MOTOR POSITION AND WAIT DELAY BEFORE RETURNING TRUE

// if reset is true, it resets the clutchState step to 0, otherwise run the clutchStateMachine
bool disengageClutch(bool reset = false)
{
  if (reset)
  {
    clutchDeviceState.transitionToStep(0);
  }
  else
  {
    clutchDeviceState.run();

    switch (clutchDeviceState.Step)
    {
    case 0:
      clutchDeviceState.transitionToStep(1);
      break;

    case 1: // ENABLE CLUTCH MOTOR
      motors[Motors::CLUTCH].enable();
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(10);
      }
      break;

    case 10: // MOVE MOTOR TO SHIFTING POSITION (CFG PARAM)
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_SHIFTING);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchDeviceState.transitionToStep(11);
      }
      break;

    case 11: // WAIT FOR MOTOR TO REACH POSITION
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        motors[Motors::CLUTCH].hold_position();
        clutchDeviceState.transitionToStep(20);
      }
      break;

    case 20: // ACTIVELY HOLD MOTOR POSITION AND WAIT DELAY
      motors[Motors::CLUTCH].hold_position();
      if (clutchDeviceState.getStepActiveTime() > DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS)
      {
        clutchDeviceState.transitionToStep(1000);
      }

    case 1000: // DONE: HOLD CLUTCH AND HOLDING PWR TO KEEP IT IN DISENGAGED POSITION
      motors[Motors::CLUTCH].hold_position();
      return true;
      break;

    default:
      break;
    }
  }
}

bool engageClutch(bool reset = false)
{
  if (reset)
  {
    clutchDeviceState.transitionToStep(0);
  }
  else
  {
    clutchDeviceState.run();

    switch (clutchDeviceState.Step)
    {
    case 0:
      if (abs(motors[Motors::CLUTCH].actualPosition - POSITION_CLUTCH_SHIFTING) < 5.0)
      {
        clutchDeviceState.transitionToStep(1);
      }
      else
      {
        triggerError("ENGAGE_CLUTCH: position not within tolerance prior to moving to engaged position");
      }
      break;

    case 1: // ENABLE CLUTCH MOTOR
      motors[Motors::CLUTCH].enable();
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(10);
      }
      break;

    case 10: // MOVE MOTOR TO PEDALING POSITION (CFG PARAM)
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING + 360.0);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchDeviceState.transitionToStep(11);
      }
      break;

    case 11: // WAIT FOR MOTOR TO REACH POSITION
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(20);
      }
      break;

    case 20:
      clutchDeviceState.StepDescription("ENGAGE_CLUTCH: wait before resetting position");
      if (clutchDeviceState.getStepActiveTime() > 5)
      {
        clutchDeviceState.transitionToStep(80);
      }
    case 80:
      clutchDeviceState.StepDescription("ENGAGE_CLUTCH: reset the position near zero");
      // at this point the motor position will be near 0 or 360 or 720 or -360 or -720, so i need reset the position so that its closer to zero
      setClutchRolloverPosition();
      clutchDeviceState.transitionToStep(90);
      break;

    case 90:
      clutchDeviceState.StepDescription("ENGAGE_CLUTCH: check neg lim sw");
      if (inputs.ClutchNegLimSw)
      {
        clutchDeviceState.transitionToStep(1000);
      }
      else
      {
        // if the neg lim sw is not pressed, then we need to hold the clutch motor in position
        triggerError("ENGAGE_CLUTCH: negative limit switch not pressed");
      }
      clutchDeviceState.transitionToStep(1000);
      break;

    case 1000: // DONE: HOLD CLUTCH AND HOLDING PWR TO KEEP IT IN ENGAGED POSITION
      motors[Motors::CLUTCH].hold_position();
      return true;
      break;

    default:
      break;
    }
  }
}

// Sets the clutch motor position to the nearest rollover position around 0
bool setClutchRolloverPosition()
{
  double newPosition = fmod(motors[Motors::CLUTCH].actualPosition, 360.0);
  motors[Motors::CLUTCH].setPosition(newPosition);
  return true;
}

StopWatch stopWatch;
// unsigned long timeNow = millis();

bool resetReq = false;

int tempInt = 0;
bool isHomed = false;
int prevStep;

// checks if there are any request for gear shifts, with priortity given to the physical buttons on the bike
int checkForGearShiftRequests()
{
  int shiftReqType = Shifter::ShiftTypes::NONE;
  if (!SHIFT_BUTTONS_DISABLED)
  {
    shiftReqType = shifter.checkShiftReq();
  }

  if (shiftReqType == Shifter::ShiftTypes::NONE)
  {
    shiftReqType = serialShiftReqType;
    shiftTargetGearParam = serialShiftTargetGearParam;
  }
  serialShiftReqType = Shifter::ShiftTypes::NONE; // reset the value
  serialShiftTargetGearParam = 0;
  return shiftReqType;
}

// updates the target gear and corresponding target position
bool processShiftReqNum(int shiftType, int targetGearParam)
{
  bool targetChanged = false;
  switch (shiftType)
  {
  case Shifter::ShiftTypes::NONE:
    break;
  case Shifter::ShiftTypes::SUPER_UP:
    SerialLogging::info("SUPER UP SHIFT");
    targetChanged = setTargetGear(NUM_GEARS);
    break;
  case Shifter::ShiftTypes::UP:
    SerialLogging::info("UP SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear + 1);
    break;
  case Shifter::ShiftTypes::SUPER_DOWN:
    SerialLogging::info("SUPER DOWN SHIFT");
    targetChanged = setTargetGear(1);
    break;
  case Shifter::ShiftTypes::DOWN:
    SerialLogging::info("DOWN SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear - 1);
    break;
  case Shifter::ShiftTypes::ABS:
    SerialLogging::info("ABS SHIFT TO: {}", targetGearParam);
    targetChanged = setTargetGear(targetGearParam);
    break;
  }
  return targetChanged;
}

void printMotionData()
{
  SerialLogging::info("Target Gear: {}", shiftData.targetGear);
  SerialLogging::info("Actual Gear: {}", motionData.actualGear);
}

// void sendShiftData()
// {
//   if (iSerial.isConnected)
//   {
//     sendInfoDataHeader(InfoTypes::SHIFT_DATA); // MODIFY THIS PER INFO TYPE
//     char data[SHIFTDATAPACKETSIZE];
//     serializeShiftData(&shiftData, data); // MODIFY THIS PER INFO TYPE
//     iSerial.taskPrintData(data, SHIFTDATAPACKETSIZE);
//     iSerial.writeNewline();
//   }
// }

// converts the ShiftData struct data into byte array to be used for sending over serial port
void serializeShiftData(ShiftData *msgPacket, char *data)
{
  // sending uint8_t vals
  uint8_t *q = (uint8_t *)data;
  *q = msgPacket->targetGear;
  q++;

  // sending int16_t vals
  int16_t *q16 = (int16_t *)q;
  //*q16 = msgPacket->targetPosition;
  // q16++;
  //*q16 = msgPacket->startingPosition;
  // q16++;

  /*
  float *f = (float *)q;
  *f = msgPacket->actualVelocity;
  f++;
  *f = msgPacket->referenceVelocity;
  f++;
  */
}

// void sendMotionData()
// {
//   if (iSerial.isConnected)
//   {
//     sendInfoDataHeader(InfoTypes::MOTION_DATA); // MODIFY THIS PER INFO TYPE
//     char data[MOTIONDATAPACKETSIZE];
//     serializeMotionData(&motionData, data); // MODIFY THIS PER INFO TYPE
//     iSerial.taskPrintData(data, MOTIONDATAPACKETSIZE);
//     iSerial.writeNewline();
//     // iSerial.debugPrint("motionData.actualGear: ");
//     // iSerial.debugPrintln(String(motionData.actualGear));
//   }
// }

// packetSize: 5
// note: converts the float values to int16_t to reduce packet size
void serializeMotionData(MotionData *msgPacket, char *data)
{
  // sending uint8_t vals
  uint8_t *q = (uint8_t *)data;
  *q = msgPacket->actualGear;
  q++;

  // sending int16_t vals
  int16_t *q16 = (int16_t *)q;
  //*q16 = round(msgPacket->actualPosition);
  // q16++;
  //*q16 = round(msgPacket->actualVelocity);
  // q16++;

  /*
  float *f = (float *)q;
  *f = msgPacket->actualPosition;
  f++;
  *f = msgPacket->actualVelocity;
  f++;
  */
}

void clearErrors()
{
  errors.present = false;
  for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  {
    errors.list[i] = "";
  }
}

void triggerError(String code)
{
  bool isNewCode = true;
  errors.present = true;
  for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  {
    if (errors.list[i] == code)
    {
      isNewCode = false;
    }
    if (errors.list[i] == "" && isNewCode)
    {
      errors.list[i] = code;
      break;
    }
  }
}

// void sendErrorData()
// {
//   if (iSerial.isConnected)
//   {
//     sendInfoDataHeader(InfoTypes::ERROR_DATA); // MODIFY THIS PER INFO TYPE
//     char data[FAULTDATAPACKETSIZE];
//     serializeFaultData(&errors, data); // MODIFY THIS PER INFO TYPE
//     iSerial.taskPrintData(data, FAULTDATAPACKETSIZE);
//     iSerial.writeNewline();
//   }
// }

void serializeFaultData(FaultData *msgPacket, char *data)
{
  // // sending uint8_t vals
  // uint8_t *q = (uint8_t *)data;
  // *q = static_cast<uint8_t>(msgPacket->present);
  // q++;
  // for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  // {
  //   *q = msgPacket->list[i];
  //   q++;
  // }
}
// USED FOR MANUAL MODE - jogs motor at 100% pwr with shift switches
void runMotorManualMode(int motor_id)
{
  static double lastPosition = 0.0;

  if (inputs.ShiftUpSw)
  {
    // disengageClutch();
    motors[motor_id].enable();
    if (motors[motor_id].getState() == Motor::States::IDLE || motors[motor_id].getState() == Motor::States::JOGGING)
    {
      motors[motor_id].jogUsingPower(MANUAL_CLUTCH_JOG_PWR);
      String infoMsg = String(motors[motor_id].actualPosition) + " deg";
      SerialLogging::info(infoMsg.c_str());
    }
    if (motors[motor_id].actualPosition < lastPosition)
    {
      SerialLogging::error("motor position decreased when it was expected to increase");
      loopState.transitionToStep(Modes::ERROR);
    }
  }
  else if (inputs.ShiftDownSw)
  {
    // disengageClutch();
    motors[motor_id].enable();
    if (motors[motor_id].getState() == Motor::States::IDLE || motors[motor_id].getState() == Motor::States::JOGGING)
    {
      motors[motor_id].jogUsingPower(-MANUAL_CLUTCH_JOG_PWR);
      String infoMsgDown = String(motors[motor_id].actualPosition) + " deg";
      SerialLogging::info(infoMsgDown.c_str());
    }

    if (motors[motor_id].actualPosition > lastPosition)
    {
      SerialLogging::error("motor position increased when it was expected to decrease");
      loopState.transitionToStep(Modes::ERROR);
    }
  }
  else
  {
    motors[motor_id].disable();
    // disengageClutch(true);
    // iSerial.resetModeTime();
    lastPosition = motors[motor_id].actualPosition;
    // if(dimitriCfg.hasClutchSolenoid){
    //   digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
    // }
    // SerialLogging::info("CLUTCH MOTOR MANUAL MODE: press and hold up or down shift to move");
  }
}

StateManager stateGearMove = StateManager("moveLinearMotorsToGear");
bool moveLinearMotorsToGear(int8_t targetGear, bool reset = false)
{
  stateGearMove.run();
  if (reset)
  {
    stateGearMove.transitionToStep(0);
  }
  else
  {
    int8_t *gearPositions = gearMap.getGearPositions(shiftData.targetGear);
    switch (stateGearMove.Step)
    {
    case 0:
      stateGearMove.StepDescription("Moving linear motors to target gear position");

      motors[Motors::LINEAR_P].moveAbs(gearPositions[0]);
      motors[Motors::LINEAR_S].moveAbs(gearPositions[1]);
      if (motors[Motors::LINEAR_P].getState() == Motor::States::MOVING && motors[Motors::LINEAR_S].getState() == Motor::States::MOVING)
      {
        stateGearMove.transitionToStep(1);
      }
      break;
    case 1:
      if (motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        stateGearMove.transitionToStep(1000);
      }
      else if (stateGearMove.getStepActiveTime() > 5000)
      {
        triggerError("moveLinearMotorsToGear() - Linear motors did not reach target gear position in time");
        SerialLogging::info("moveLinearMotorsToGear() - Linear motors did not reach target gear position in time");
      }
      break;
    case 1000:
      stateGearMove.StepDescription("linear motors are now at target gear position");
      return true; // successfully moved to target gear position
    case 911:
      turnAllOff();
      break;
    }
  }
  return false;
}

void runLinearMotorManualMode(uint8_t motor_id)
{
  bool clutchIsDisengaged = true; // disengageClutch();
  motors[motor_id].enable();
  if (!motors[motor_id].getState() == Motor::States::IDLE || !motors[motor_id].getState() == Motor::States::JOGGING)
  {
    // wait for enable to kick in;
  }
  else if (inputs.ShiftUpSw && clutchIsDisengaged)
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, !motorCfgs[Motors::LINEAR].invertDir);
    String infoMsgUp = String(motors[motor_id].actualPosition) + " deg";
    SerialLogging::info(infoMsgUp.c_str());
    motors[motor_id].jogUsingPower(MANUAL_LINEAR_JOG_PWR);
    // SerialLogging::info("jogging motor %d positively - position: %f deg", motor_id, motors[motor_id].actualPosition);

    // motors[Motors::LINEAR].moveAbs(11.0);
  }
  else if (inputs.ShiftDownSw && clutchIsDisengaged)
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, motorCfgs[Motors::LINEAR].invertDir);
    String infoMsgDown = String(motors[motor_id].actualPosition) + " deg";
    SerialLogging::info(infoMsgDown.c_str());
    motors[motor_id].jogUsingPower(-MANUAL_LINEAR_JOG_PWR);
    // SerialLogging::info("jogging motor %d negatively - position: %f deg", motor_id, motors[motor_id].actualPosition);
    //  motors[Motors::LINEAR].moveAbs(6.0);
  }
  else
  {
    // digitalWrite(PIN_LINEAR_P_PWM, LOW);
    motors[motor_id].disable();
    // motors[Motors::LINEAR].disable();
  }
}

bool runMotorsThisScan = false;

void updateMotors()
{
  motors[motorId].update();
  motors[motorId].run();
  motorId = (motorId + 1) % NUM_MOTORS;

  // return true;  // Indicate that the update was performed
  //  runMotorsThisScan = !runMotorsThisScan;
  //  for (int i = 0; i < NUM_MOTORS; i++)
  //  {
  //    motors[i].update();
  //    if (runMotorsThisScan)
  //    {
  //      //motors[i].run();
  //    }
  //  }
}

StateManager stateManagerHRLM = StateManager("stateMngrHomeLinearMotors"); // state manager for homing routine linear motors
bool runHomingRoutineLinearMotors(bool reset = false)
{
  stateManagerHRLM.run();
  if (reset)
  {
    stateManagerHRLM.transitionToStep(0);
  }
  else
  {
    switch (stateManagerHRLM.Step)
    {
    case 0:
      stateManagerHRLM.StepDescription("Initializing homing process for linear motors");
      isHomed = false;
      stateManagerHRLM.transitionToStep(1);
      break;
    case 1:
      stateManagerHRLM.StepDescription("Initializing disengaging clutch motor routine");
      disengageClutch(true);
      stateManagerHRLM.transitionToStep(10);
      break;

    case 10:
      stateManagerHRLM.StepDescription("Moving clutch motor to disengaged position");
      if (disengageClutch())
      {
        stateManagerHRLM.transitionToStep(20);
      }
      break;

    case 20:
      stateManagerHRLM.StepDescription("Enabling linear motors");
      motors[Motors::LINEAR_P].enable();
      motors[Motors::LINEAR_S].enable();
      disengageClutch();
      if (motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        SerialLogging::info("HOMING - Moving primary motor to postive hardstop and secondary motor to negative hard stop");
        stateManagerHRLM.transitionToStep(50);
      }
      break;

    case 50:
      stateManagerHRLM.StepDescription("request homing for linear motors");

      disengageClutch();
      motors[Motors::LINEAR_P].home();
      motors[Motors::LINEAR_S].home();
      if (motors[Motors::LINEAR_P].ActiveProcess == Motor::MotorProcesses::HOME && motors[Motors::LINEAR_S].ActiveProcess == Motor::MotorProcesses::HOME)
      {
        if (!motors[Motors::LINEAR_P].isHomed && !motors[Motors::LINEAR_S].isHomed)
        {
          stateManagerHRLM.transitionToStep(51);
        }
      }
      break;

    case 51:
      stateManagerHRLM.StepDescription("waiting for homing to complete");
      if (motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        if (motors[Motors::LINEAR_P].isHomed && motors[Motors::LINEAR_S].isHomed)
        {
          stateManagerHRLM.transitionToStep(60);
        }
      }
      break;

    case 60:
      stateManagerHRLM.StepDescription("moving to gear 1");
      disengageClutch();
      shiftData.targetGear = 1;
      // issue gear position move cmds to linear motors
      moveLinearMotorsToGear(shiftData.targetGear);

      if (motors[Motors::LINEAR_P].getState() == Motor::States::MOVING && motors[Motors::LINEAR_S].getState() == Motor::States::MOVING)
      {
        SerialLogging::info("HOMING - Moving linear motors to 1st gear");
        stateManagerHRLM.transitionToStep(61);
      }
      break;
    case 61:
      stateManagerHRLM.StepDescription("waiting for linear motors to reach gear 1 positions");
      disengageClutch();
      if (motors[Motors::LINEAR_P].atPosition && motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].atPosition && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        SerialLogging::info("HOMING - Linear motors are at gear 1 positions");
        engageClutch(true);
        stateManagerHRLM.transitionToStep(70);
      }
      else if (stateManagerHRLM.getStepActiveTime() > 5000)
      {
        triggerError("runHomingRoutineLinearMotors() - Linear motors did not reach gear 1 positions in time");
        SerialLogging::error("HOMING - Linear motors did not reach gear 1 positions in time");
        // iSerial.status.step = 911; // go to error step
      }

    case 70:
      stateManagerHRLM.StepDescription("engaging clutch motor to pedaling position");
      {
        motors[Motors::CLUTCH].jogUsingPower(-100.0);
        if (engageClutch())
        {
          stateManagerHRLM.transitionToStep(80);
        }
        break;
      case 1000:
        stateManagerHRLM.StepDescription("linear motors are now homed");
        {
          // DONE HOMING
          isHomed = true;
          return true;
          break;
        }
      case 911:
        turnAllOff();
        break;
      }
    }
  }
  return false;
}

// sets the actual gear based on the actual position of the linear motor and atTarget flag
void checkActualGear()
{
  atTarget = false;
  bool atWiderTarget = abs(motors[Motors::LINEAR_P].actualPosition - shiftData.targetGear) < 2.0 * motorCfgs[Motors::LINEAR_P].positionTol;
  if (motors[Motors::LINEAR_P].atPosition || atWiderTarget)
  {
    motionData.actualGear = round(motors[Motors::LINEAR_P].actualPosition);
    if (shiftData.targetGear == motionData.actualGear)
    {
      atTarget = true;
    }
  }
}

byte *publishedData = new byte[3 * MOTOR_DATA_SIZE];
void updatePublishedDataChunk()
{
  int i = 0;
  int size = 0;

  for (i = 0; i < NUM_MOTORS; i++)
  {
    byte *motorData = motors[i].getMotorData();
    memcpy(publishedData + size, motorData, MOTOR_DATA_SIZE);
    size += MOTOR_DATA_SIZE;
    delete[] motorData;
  }
}

// sets the digital outputs for the gear number to be received by the display device
void updateGearNumberDigitalOutputs(int num)
{
  for (int i = 0; i < NUM_BITS; i++)
  {
    bool val = (num >> i) & 1;
    digitalWrite(2 * i + PIN_EINK_BIT0, val);
  }
}

// checks the homeSw state and shiftType, prevents downshifting if homeSw detects target
bool checkShiftTypeAndHomeSw(int shiftTypeReq)
{
  if (false && (shiftTypeReq == Shifter::ShiftTypes::UP))
  {
    return false;
  }
  return true;
}

// void processRelPosCmd()
// {
//   // int motorId = iSerial.idChr - '0';
//   if (!iSerial.parseLong(tempLong))
//   {
//     if (tempLong > 0)
//     {
//       serialShiftReqType = Shifter::ShiftTypes::UP;
//     }
//     else if (tempLong < 0)
//     {
//       serialShiftReqType = Shifter::ShiftTypes::DOWN;
//     }

//     iSerial.writeCmdChrIdChr();
//     iSerial.writeLong(tempLong);
//     iSerial.writeNewline();
//   }
//   else
//   {
//     iSerial.writeCmdWarning("could not parse position data");
//   }
// }

// void processAbsPosCmd()
// {
//   // int motorId = iSerial.idChr - '0';
//   if (!iSerial.parseLong(tempLong))
//   {
//     serialShiftReqType = Shifter::ShiftTypes::ABS;
//     serialShiftTargetGearParam = tempLong;

//     iSerial.writeCmdChrIdChr();
//     iSerial.writeLong(tempLong);
//     iSerial.writeNewline();
//   }
//   else
//   {
//     iSerial.writeCmdWarning("could not parse position data");
//   }
// }

// // handles serial cmds that aren't already handled by ISerial (connect, mode, debug)
// void handleSerialCmds()
// {
//   // int idx = iSerial.idChr - '0';

//   iSerial.handleBasicSerialCmds();

//   switch (iSerial.cmdChr)
//   {
//   case Cmds::CLEAR_CMD:
//     clearErrors();
//     break;
//   case Cmds::ABSPOS_CMD:
//     // processAbsPosCmd();
//     break;

//   case Cmds::HOME_CMD:
//     clearErrors();
//     iSerial.writeCmdChrIdChr();
//     iSerial.writeNewline();
//     // iSerial.setNewMode(Modes::RESETTING);
//     break;

//   case Cmds::RELPOS_CMD:
//     // processRelPosCmd();
//     break;

//   case Cmds::SERVOPOSINFO_CMD:
//     // processServoPosInfoCmd();
//     break;

//   case Cmds::SERIAL_OUTPUT: // prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
//     sendDiagnosticData();
//     break;

//   case Cmds::PARAMS_SET: // set params
//     // processParamsCmd();
//     break;

//   default:
//     processUnrecognizedCmd();
//     break;
//   }
// }

// void sendDiagnosticData()
// {
//   iSerial.writeCmdChrIdChr();
//   printDiagnosticDataToJsonString();
//   // iSerial.writeString(jsonString);
//   iSerial.writeNewline();
//   // iSerial.debugPrint("iSerial.status.mode: ");
//   // SerialLogging::info(String(iSerial.status.mode));
// }

// void processUnrecognizedCmd()
// {
//   String msg1 = "didn't recognize cmdChr: ";
//   msg1.concat(char(iSerial.cmdChr));
//   iSerial.writeCmdWarning(msg1);
// }

void turnAllOff() // turn off all outputs
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].disable();
  }
}

bool enableAllMotors()
{
  bool allMotorsEnabled = true;
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].enable();
    if (motors[i].getState() != Motor::States::IDLE)
    {
      allMotorsEnabled = false;
    }
  }
  return allMotorsEnabled;
}

String getSensorStateString(bool state)
{
  return state ? "ON" : "OFF";
}

void readInputs()
{
  inputs.ShiftUpSw = !digitalRead(PIN_SHIFT_UP);
  inputs.ShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
  inputs.ClutchNegLimSw = !digitalRead(PIN_CLUTCH_NEG_LIM);
  inputs.ClutchPosLimSw = !digitalRead(PIN_CLUTCH_POS_LIM);
  // inputs.LinearNegLimSw = !digitalRead(PIN_LINEAR_LIM_SW) && dimitriCfg.homingDirection == -1;
}

// returns true if the target changed from previous
bool setTargetGear(int targetGearNum)
{
  SerialLogging::info("setTargetGear()");
  int prevTargetGear = shiftData.targetGear;
  if (targetGearNum > NUM_GEARS)
  {
    shiftData.targetGear = NUM_GEARS;
  }
  else if (targetGearNum < 1)
  {
    shiftData.targetGear = 1;
  }
  else
  {
    shiftData.targetGear = targetGearNum;
  }

  // INITIALIZING SHIFT DATA - if targetChanged
  bool targetChanged = shiftData.targetGear != prevTargetGear;
  if (targetChanged)
  {
    SerialLogging::info(String(shiftData.targetGear).c_str());
  }
  return targetChanged;
}

void initializeDiagnosticData()
{
  diagnosticData.numOfDataPoints = 0;
  diagnosticData.targetGear = shiftData.targetGear;
  // diagnosticData.targetPosition = shiftData.targetPosition;
  diagnosticData.actualGear = motionData.actualGear;
  // diagnosticData.actualPosition = motionData.actualPosition;
}

int getRandomNumber(int min, int max) // get reandom
{
  return random(min, max + 1);
}

bool prevResetReq = false;

uint8_t motorIdLoop = 0;
unsigned long lastUpdateScanUs = 0;
unsigned long lastLoopUpdateScanUs = 0;
void loop()
{
  // SerialLogging::info("Dimitri loop() started");

  unsigned long timeNowUs = micros();
  if ((timeNowUs - lastUpdateScanUs) >= UPDATE_TIME_US)
  {
    updateMotors();
    SerialLogging::process();
    //Serial.println(timeNowUs - lastUpdateScanUs);
    lastUpdateScanUs = timeNowUs;
  }
  else if (timeNowUs - lastLoopUpdateScanUs > (NUM_MOTORS - 1) * UPDATE_TIME_US && motorId == 0)
  {
    loopState.run();
    // lastUpdateUs = timeNowUs;
    //Serial.println(timeNowUs - lastLoopUpdateScanUs);
    lastLoopUpdateScanUs = timeNowUs;

    readInputs();
    int shiftTypeReq = checkForGearShiftRequests();
    // timeNow = millis();

    // detect resetRequest
    if (inputs.ShiftDownSw && inputs.ShiftUpSw)
    {
      resetReq = true && !prevResetReq;
      prevResetReq = true;
    }
    else
    {
      resetReq = false;
      prevResetReq = false;
    }

    // AUTO ABORT IF ERROR
    if (errors.present && loopState.Step >= int(Modes::RESETTING))
    {
      loopState.transitionToStep(Modes::ABORTING);
    }
    else if (resetReq && loopState.Step == int(Modes::IDLE))
    {
      // iSerial.setNewMode(Modes::RESETTING);
    }

    switch (loopState.Step)
    {
    case Modes::ABORTING:
      loopState.StepDescription("ABORTING - turning all motors off");
      turnAllOff();
      loopState.transitionToStep(Modes::KILLED);
      break;
    case Modes::KILLED:
      loopState.StepDescription("KILLED - all motors off");
      if (loopState.FirstScan)
      {
        turnAllOff();
      }

      if (errors.present)
      {
        loopState.transitionToStep(Modes::ERROR);
      }
      else
      {
        loopState.transitionToStep(Modes::INACTIVE);
      }

      break;
    case Modes::ERROR:
      loopState.StepDescription("ERROR - handling error state");
      shiftData.targetGear = 0;
      isHomed = false;
      if (loopState.FirstScan)
      {
        turnAllOff();
      }
      else
      {
        // auto clear the errors after 5 seconds
        if (resetReq)
        {
          clearErrors();
        }
        if (!errors.present)
        {
          // iSerial.setNewMode(Modes::INACTIVE);
          loopState.transitionToStep(Modes::INACTIVE);
        }
        if (loopState.getStepActiveTime() > 3000 && false)
        {
          SerialLogging::error("ERROR: %s", errors.list[0].c_str());
        }
      }

      break;
    case Modes::INACTIVE:
      loopState.StepDescription("INACTIVE - motors off, waiting for reset");
      // turnAllOff();
      if (OPERATING_MODE == OperatingModes::AUTO || OPERATING_MODE == OperatingModes::MANUAL_CLUTCH_ENGAGE)
      {
        if (!AUTO_RESET)
        {
          // iSerial.debugPrintln("Press both shift buttons to reset");
        }

        if (AUTO_RESET || (resetReq))
        {
          loopState.transitionToStep(Modes::RESETTING);
        }
      }
      else
      {
        loopState.transitionToStep(Modes::MANUAL);
      }
      break;

    case Modes::RESETTING:
      loopState.StepDescription("RESETTING - performing homing routines");

      // skip homing if already homed
      if (isHomed)
      {
        // SerialLogging::info("RESETTING: already homed, skipping homing routine");
        if (enableAllMotors())
        {
          loopState.transitionToStep(Modes::IDLE);
        }
      }
      else
      {
        // SerialLogging::info("RESETTING: running homing routines");
        //  runHomingRoutineClutchMotor(true);
        // motors[Motors::CLUTCH].setDebug(true);
        loopState.transitionToStep(51);
      }
      break;

    case 51:
      loopState.StepDescription("RESETTING - requesting clutch homing process");
      // Initialize any variables or states needed before starting homing
      // motors[Motors::CLUTCH].requestProcess(Motor::MotorProcesses::HOME);
      if (loopState.FirstScan)
      {
        motors[Motors::CLUTCH].requestProcess(Motor::MotorProcesses::HOME);
      }
      else if (motors[Motors::CLUTCH].ActiveProcess == Motor::MotorProcesses::HOME && !motors[Motors::CLUTCH].isHomed)
      {
        loopState.transitionToStep(53);
      }
      else
      {
        triggerError("RESETTING: request clutch motor homing was rejected");
      }
      break;

    case 53:
      loopState.StepDescription("Waiting for clutch motor to finish homing");
      // Wait for clutch motor to reach disengaged position
      // SerialLogging::info("Clutch Motor State: %d", motors[Motors::CLUTCH].getState());
      if (motors[Motors::CLUTCH].ActiveProcess == Motor::MotorProcesses::NONE_PROCESS && motors[Motors::CLUTCH].isHomed)
      {
        if (OPERATING_MODE == OperatingModes::MANUAL_CLUTCH_ENGAGE)
        {
          loopState.transitionToStep(Modes::MANUAL);
        }
        else
        {
          loopState.transitionToStep(55);
        }
      }
      break;

    case 55:
      loopState.StepDescription("RESETTING - moving clutch to engaged pedaling position");
      if (loopState.FirstScan)
      {
        // Start moving clutch to engaged position
        motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING);
      }

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        loopState.transitionToStep(56);
      }
      break;

    case 56:
      loopState.StepDescription("Waiting for clutch motor to reach pedaling position");
      // Wait for clutch motor to reach engaged position
      if (motors[Motors::CLUTCH].atPositionAndStill && motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        setClutchRolloverPosition();
        loopState.transitionToStep(60);
      }

      else if (loopState.getStepActiveTime() > 5000)
      {
        triggerError("RESETTING: Clutch motor did not reach pedaling position in time");
      }
      break;

    case 60:
      loopState.StepDescription("RESETTING - running homing routine for linear motors");
      if (loopState.FirstScan)
      {
        runHomingRoutineLinearMotors(true);
      }
      else if (runHomingRoutineLinearMotors())
      {
        isHomed = true;
        loopState.transitionToStep(Modes::IDLE);
      }

      break;

    case Modes::IDLE:
      loopState.StepDescription("IDLE - waiting for gear change request");
      if (gearChangeReq)
      {
        processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
        // shiftStartTimeMs = timeNow;
        loopState.transitionToStep(Modes::SHIFTING);
      }
      else if (!atTarget)
      {
        // triggerError(Errors::MOTOR_NOT_AT_TARGET_WHILE_IDLE);
        SerialLogging::info("linear motors not at target while in idle, adjust position now");
        loopState.transitionToStep(Modes::SHIFTING);
      }
      else if (resetReq)
      {
        loopState.transitionToStep(Modes::RESETTING);
      }

      break;

    case Modes::SHIFTING:
      loopState.StepDescription("SHIFTING - processing gear shift");
      if (loopState.FirstScan)
      {
        disengageClutch(true);
      }
      else
      {
        if (disengageClutch())
        {
          moveLinearMotorsToGear(shiftData.targetGear, true);
          loopState.transitionToStep(210);
        }
      }
      break;

    case 210:
      loopState.StepDescription("SHIFTING - moving linear motors to target gear");
      disengageClutch();

      if (moveLinearMotorsToGear(shiftData.targetGear) && atTarget)
      {
        loopState.transitionToStep(220);
      }
      else if (gearChangeReq)
      {
        processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
        // iSerial.resetModeTime();
        // loopState.transitionToStep(1);
      }

      break;

    case 220:
      loopState.StepDescription("SHIFTING - engaging clutch");
      if (loopState.FirstScan)
      {
        // digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
        engageClutch(true);
        // reset mode time
      }
      else if (engageClutch())
      {
        // shiftTimeMs = timeNow - shiftStartTimeMs;
        loopState.transitionToStep(Modes::IDLE);
      }
      else if (loopState.getStepActiveTime() > 4000)
      {
        triggerError("SHIFTING: TOGGLE_MOTOR_ENGAGE_MOVE_TIMED_OUT");
      }

      break;

    case Modes::MANUAL:
      if (loopState.FirstScan)
      {
        disengageClutch(true);
      }

      switch (OPERATING_MODE)
      {
      case OperatingModes::MANUAL_CLUTCH_JOGGING:
        loopState.StepDescription("MANUAL - clutch jogging - use up/down");
        runMotorManualMode(Motors::CLUTCH);
        break;
      case OperatingModes::MANUAL_CLUTCH_ENGAGE:
        if (inputs.ShiftUpSw)
        {
          disengageClutch(true);
          loopState.transitionToStep(1150);
        }
        break;
      case OperatingModes::MANUAL_LINEAR_P:
        loopState.StepDescription("MANUAL - linear P jogging - use up/down");
        runMotorManualMode(Motors::LINEAR_P);
        break;
      case OperatingModes::MANUAL_LINEAR_S:
        loopState.StepDescription("MANUAL - linear S jogging - use up/down");
        runMotorManualMode(Motors::LINEAR_S);
        break;
      }

      break;
    case 1150:
      loopState.StepDescription("MANUAL - waiting for clutch to disengage");
      if (disengageClutch() && inputs.ShiftUpSw)
      {
        SerialLogging::info("Clutch motor disengaged (shift position)");
        engageClutch(true);
        loopState.transitionToStep(1155);
      }

      break;

    case 1155:
      loopState.StepDescription("MANUAL - waiting for clutch to engage after shift up");
      if (engageClutch())
      {
        SerialLogging::info("Clutch motor engaged (pedal position)");
        disengageClutch(true);
        loopState.transitionToStep(Modes::MANUAL);
      }

      break;

    default:
      SerialLogging::info(("DEBUG: UNRECOGNIZED Loop Step: " + String(loopState.Step)).c_str());
      break;
    }

    checkActualGear();
    updateGearNumberDigitalOutputs(shiftData.targetGear);
    gearChangeReq = false;

    if (DIAGNOSTIC_MODE == DiagnosticModes::UI)
    {
      updatePublishedDataChunk();
      SerialLogging::publishData(publishedData, MOTOR_DATA_SIZE * 3, "UI");
    }
    SerialLogging::process();

  }
  else
  {
    SerialLogging::process();
    
  }
}
