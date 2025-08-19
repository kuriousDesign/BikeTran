#define SCAN_TIME_US 5000  // how frequently the loop updates
#define UPDATE_TIME_US 350 // time that the motor velocities are updated, motor run() are called at half this rate
#define NUM_MOTORS 3

enum Motors
{
  CLUTCH = 0,
  LINEAR_P = 1,
  LINEAR_S = 2,
};

#include <Arduino.h>
#include "Arduino.h"
#include "ISerial.h"
#include "Encoder.h"
#include "Shifter.h"
#include "StateManager.h"
#include "CustomDataTypes.h"
#include "GearMap.h"
#include "Motor.h"
#include "TimerOne.h"

const OperatingModes OPERATING_MODE = OperatingModes::MANUAL_CLUTCH; // set to OperatingModes::AUTO to run the system in debug mode

const double LinearHomingPwr = 100.0;
const int LinearNudgeTimeMsDuringHomingJog = 50;
const double ClutchHoldingPwr = 25.0;
const int clutchSolenoidJostleTimeMs = 20;
const double LinearKp = 500.0; //@11.05VDC
const double LinearKd = 0.0;
uint16_t LinearNudgeTimeMs = 4; // 15
const double LinearNudgePower = 100.0;
const double POSITION_CLUTCH_PEDALING = 0.0;   // deg
const double POSITION_CLUTCH_SHIFTING = 228.0; // deg
const double LINEAR_P_HOME_OFFSET = 6.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear
const double LINEAR_S_HOME_OFFSET = 0.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear

const unsigned long DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS = 200; // ms, time to wait after clutch reaches shifting position before linear motor motion is allowed
const bool AUTO_RESET = false;                                                        // if this is true, the system will automatically reset when inactive (no errors)
const bool SHIFT_BUTTONS_DISABLED = false;                                            // if this is true, the shift buttons will be disabled (ignored)
const bool SIM_MODE = false;                                                          // set to true to simulate motor behavior (encoders positions for now, TODO: simulate lim switches)

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

// CLUTCH MOTOR - WIRED TO STANDALONE MOTOR BOARD A
#define PIN_CLUTCH_PWM 44     // --> wires to ENA pin of L298N driver board
#define PIN_CLUTCH_DIR_IN1 45 // --> wires to IN1 of L298N driver board
#define PIN_CLUTCH_DIR_IN2 46 // --> wires to IN2 of L298N driver board

#define PIN_LINEAR_P_ENC_A 18 // yellow wire of enc
#define PIN_LINEAR_P_ENC_B 19 // green wire of enc

#define PIN_LINEAR_S_ENC_A 20 // yellow wire of enc
#define PIN_LINEAR_S_ENC_B 52 // green wire of enc

#define PIN_CLUTCH_ENC_A 21 // yellow wire of enc??
#define PIN_CLUTCH_ENC_B 54 // green wire of enc??

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

const double CLUTCH_PULSES_PER_UNIT = 780.0 / 180.0;
const double LINEAR_PULSES_PER_UNIT = 14.0 / 11.0 * 2.0 * 8600.0 / (double(NUM_GEARS) - 1.0); // 9200 is the max position, 488 is the min position

Motor::Cfg clutchMotorCfg = {
  name : "clutch",                             // name
  homingDir : Motor::HomingDir::POSITIVE,      // homing direction
  homingType : Motor::HomingType::HOME_SWITCH, // homing type
  homeOffsetFromZero : 0.0,                    // home offset from zero
  unit : "deg",
  pulsesPerUnit : CLUTCH_PULSES_PER_UNIT, // unit and pulses per unit
  maxVelocity : 1000.0,                   // max velocity
  softLimitPositive : 721.0,              // soft limit positive
  softLimitNegative : -721.0,             // soft limit negative
  invertEncoderDir : false,               // invert encoder
  encoderRollover : false,                // enable encoder rollover
  invertMotorDir : false,                 // invert motor direction
  positionTol : 3.0,                      // position tolerance
  zeroVelocityTol : 5.0,                  // zero velocity tolerance, units/sec
  kP : 10.0,                              // kP
  kD : 1.0,                               // kD
  nudgeTimeMs : 0,                        // nudge time
  nudgePower : 100.0                      // nudge power
};

Motor::Cfg linearPrimaryMotorCfg = {
  name : "linear_primary",
  homingDir : Motor::HomingDir::POSITIVE,
  homingType : Motor::HomingType::HARDSTOP,
  homeOffsetFromZero : LINEAR_P_HOME_OFFSET, // units
  unit : "gear",
  pulsesPerUnit : LINEAR_PULSES_PER_UNIT,
  maxVelocity : 20.0,
  softLimitPositive : (6 + 0.5),
  softLimitNegative : 0.5,
  invertEncoderDir : false,
  encoderRollover : false,
  invertMotorDir : false,
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
  unit : "gear",
  pulsesPerUnit : LINEAR_PULSES_PER_UNIT,
  maxVelocity : 20.0,
  softLimitPositive : (3 + 0.5),
  softLimitNegative : 0.5,
  invertEncoderDir : false,
  encoderRollover : false,
  invertMotorDir : false,
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
ISerial iSerial;
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
StateManager clutchState;

void setup()
{
  iSerial.init();
  iSerial.THIS_DEVICE_ID = BIKE_MEGA_ID;
  iSerial.setAutoSendStatus(false); // status updates sent when mode changes
  iSerial.debug = true;
  clutchState.SetDebug(iSerial.debug);

  iSerial.setNewMode(Modes::ABORTING);

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

  // if (dimitriCfg.hasClutchSolenoid)
  // {
  //   pinMode(PIN_CLUTCH_SOL, OUTPUT);
  //   digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
  // }

  motors[Motors::CLUTCH].setDebug(false);
  motors[Motors::LINEAR_P].setDebug(false);
  motors[Motors::LINEAR_S].setDebug(false);

  shiftData.targetGear = 0;
  lastUpdateUs = micros();

  unsigned long timeNowMs = millis();
  while (OPERATING_MODE == OperatingModes::IO_CHECKOUT)
  {
    Serial.println("OPERATING MODE: IO_CHECKOUT");
    iSerial.debug = true;
    readInputs();
    iSerial.debugPrint("state of clutch lim sw at the pedaling position: ");
    iSerial.debugPrintln(inputs.ClutchNegLimSw ? "ON" : "OFF");
    iSerial.debugPrint("state of clutch positive lim sw at the shifting position: ");
    iSerial.debugPrintln(inputs.ClutchPosLimSw ? "ON" : "OFF");
    iSerial.debugPrint("state of shift up switch: ");
    iSerial.debugPrintln(inputs.ShiftUpSw ? "ON" : "OFF");
    iSerial.debugPrint("state of shift down switch: ");
    iSerial.debugPrintln(inputs.ShiftDownSw ? "ON" : "OFF");
    delay(1000);
  }
  Timer1.initialize(UPDATE_TIME_US); // Initialize timer to trigger every X microseconds
  Timer1.attachInterrupt(updateMotors, UPDATE_TIME_US);
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
    clutchState.transitionToStep(0);
  }
  else
  {
    clutchState.run();

    switch (clutchState.Step)
    {
    case 0:
      clutchState.transitionToStep(1);
      break;

    case 1: // ENABLE CLUTCH MOTOR
      motors[Motors::CLUTCH].enable();
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchState.transitionToStep(10);
      }
      break;

    case 10: // MOVE MOTOR TO SHIFTING POSITION (CFG PARAM)
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_SHIFTING);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchState.transitionToStep(11);
      }
      break;

    case 11: // WAIT FOR MOTOR TO REACH POSITION
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        motors[Motors::CLUTCH].hold_position();
        clutchState.transitionToStep(20);
      }
      break;

    case 20: // ACTIVELY HOLD MOTOR POSITION AND WAIT DELAY
      motors[Motors::CLUTCH].hold_position();
      if (clutchState.getStepActiveTime() > DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS)
      {
        clutchState.transitionToStep(1000);
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
    clutchState.transitionToStep(0);
  }
  else
  {
    clutchState.run();

    switch (clutchState.Step)
    {
    case 0:
      if (abs(motors[Motors::CLUTCH].actualPosition - POSITION_CLUTCH_SHIFTING) < 5.0)
      {
        clutchState.transitionToStep(1);
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
        clutchState.transitionToStep(10);
      }
      break;

    case 10: // MOVE MOTOR TO PEDALING POSITION (CFG PARAM)
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING + 360.0);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchState.transitionToStep(11);
      }
      break;

    case 11: // WAIT FOR MOTOR TO REACH POSITION
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchState.transitionToStep(20);
      }
      break;

    case 20:
      clutchState.StepDescription("ENGAGE_CLUTCH: wait before resetting position");
      if (clutchState.getStepActiveTime() > 5)
      {
        clutchState.transitionToStep(80);
      }
    case 80:
      clutchState.StepDescription("ENGAGE_CLUTCH: reset the position near zero");
      // at this point the motor position will be near 0 or 360 or 720 or -360 or -720, so i need reset the position so that its closer to zero
      setClutchRolloverPosition();
      clutchState.transitionToStep(90);
      break;

    case 90:
      clutchState.StepDescription("ENGAGE_CLUTCH: check neg lim sw");
      if (inputs.ClutchNegLimSw)
      {
        clutchState.transitionToStep(1000);
      }
      else
      {
        // if the neg lim sw is not pressed, then we need to hold the clutch motor in position
        triggerError("ENGAGE_CLUTCH: negative limit switch not pressed");
      }
      clutchState.transitionToStep(1000);
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
unsigned long timeNow = millis();

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
    iSerial.debugPrintln("SUPER UP SHIFT");
    targetChanged = setTargetGear(NUM_GEARS);
    break;
  case Shifter::ShiftTypes::UP:
    iSerial.debugPrintln("UP SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear + 1);
    break;
  case Shifter::ShiftTypes::SUPER_DOWN:
    iSerial.debugPrintln("SUPER DOWN SHIFT");
    targetChanged = setTargetGear(1);
    break;
  case Shifter::ShiftTypes::DOWN:
    iSerial.debugPrintln("DOWN SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear - 1);
    break;
  case Shifter::ShiftTypes::ABS:
    iSerial.debugPrint("ABS SHIFT TO: ");
    iSerial.debugPrintln(String(targetGearParam));
    targetChanged = setTargetGear(targetGearParam);
    break;
  }
  return targetChanged;
}

void printMotionData()
{
  iSerial.debugPrint("Target Gear: ");
  iSerial.debugPrint(String(shiftData.targetGear)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("");

  iSerial.debugPrint("Actual Gear: ");
  iSerial.debugPrint(String(motionData.actualGear)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("");
  /*
  iSerial.debugPrint("Target Position: ");
  iSerial.debugPrint(String(shiftData.targetPosition)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg");

  iSerial.debugPrint("Actual Position: ");
  iSerial.debugPrint(String(motionData.actualPosition)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg");

  iSerial.debugPrint("Actual Velocity: ");
  iSerial.debugPrint(String(motionData.actualVelocity)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg/sec");
  */

  // iSerial.debugPrintln(String(motionData.actualPosition));

  // iSerial.debugPrint("Encoder Raw Position: ");
  // iSerial.debugPrint(String(encoder.position)); // Use 1 decimal places for floating-point numbers
  // iSerial.debugPrintln("deg");
}

void sendInfoDataHeader(InfoTypes infoType)
{
  String headerString = char(Cmds::INFO_CMD) + String(int(infoType));
  iSerial.writeString(headerString);
}

void sendShiftData()
{
  if (iSerial.isConnected)
  {
    sendInfoDataHeader(InfoTypes::SHIFT_DATA); // MODIFY THIS PER INFO TYPE
    char data[SHIFTDATAPACKETSIZE];
    serializeShiftData(&shiftData, data); // MODIFY THIS PER INFO TYPE
    iSerial.taskPrintData(data, SHIFTDATAPACKETSIZE);
    iSerial.writeNewline();
  }
}

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

void sendMotionData()
{
  if (iSerial.isConnected)
  {
    sendInfoDataHeader(InfoTypes::MOTION_DATA); // MODIFY THIS PER INFO TYPE
    char data[MOTIONDATAPACKETSIZE];
    serializeMotionData(&motionData, data); // MODIFY THIS PER INFO TYPE
    iSerial.taskPrintData(data, MOTIONDATAPACKETSIZE);
    iSerial.writeNewline();
    // iSerial.debugPrint("motionData.actualGear: ");
    // iSerial.debugPrintln(String(motionData.actualGear));
  }
}

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

void sendErrorData()
{
  if (iSerial.isConnected)
  {
    sendInfoDataHeader(InfoTypes::ERROR_DATA); // MODIFY THIS PER INFO TYPE
    char data[FAULTDATAPACKETSIZE];
    serializeFaultData(&errors, data); // MODIFY THIS PER INFO TYPE
    iSerial.taskPrintData(data, FAULTDATAPACKETSIZE);
    iSerial.writeNewline();
  }
}

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
void runClutchMotorManualMode()
{
  static double lastPosition = 0.0;

  if (inputs.ShiftUpSw)
  {
    // disengageClutch();
    motors[Motors::CLUTCH].enable();
    motors[Motors::CLUTCH].jogUsingPower(20.0);
    if (iSerial.modeTime() > 500 && motors[Motors::CLUTCH].actualPosition < lastPosition)
    {
      Serial.println("error: clutch motor position decreased when it was expected to increase");
    }
    else
    {
      Serial.print("jogging clutch motor positively - position: ");
      Serial.print(motors[Motors::CLUTCH].actualPosition);
      Serial.println(" deg");
    }
  }
  else if (inputs.ShiftDownSw)
  {
    // disengageClutch();
    motors[Motors::CLUTCH].enable();
    motors[Motors::CLUTCH].jogUsingPower(-20.0);
    if (iSerial.modeTime() > 500 && motors[Motors::CLUTCH].actualPosition > lastPosition)
    {
      Serial.println("error: clutch motor position increased when it was expected to decrease");
    }
    else
    {
      Serial.print("jogging clutch motor negatively - position: ");
      Serial.print(motors[Motors::CLUTCH].actualPosition);
      Serial.println(" deg");
    }
  }
  else
  {
    motors[Motors::CLUTCH].disable();
    // disengageClutch(true);
    iSerial.resetModeTime();
    lastPosition = motors[Motors::CLUTCH].actualPosition;
    // if(dimitriCfg.hasClutchSolenoid){
    //   digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
    // }
    Serial.println("CLUTCH MOTOR MANUAL MODE: press and hold up or down shift to move");
  }
}

StateManager stateGearMove;
bool moveLinearMotorsToGear(int8_t targetGear, bool reset = false)
{
  if (reset)
  {
    stateGearMove.transitionToStep(0);
  }
  else
  {
    switch (stateGearMove.Step)
    {
    case 0:
      stateGearMove.StepDescription("Moving linear motors to target gear position");
      int8_t *gearPositions = gearMap.getGearPositions(shiftData.targetGear);
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
        iSerial.debugPrintln("moveLinearMotorsToGear() - Linear motors did not reach target gear position in time");
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

void runLinearMotorManualMode(uint8_t motorId)
{

  bool clutchIsDisengaged = disengageClutch();

  if (inputs.ShiftUpSw && clutchIsDisengaged)
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, !motorCfgs[Motors::LINEAR].invertDir);
    motors[motorId].enable();
    motors[motorId].jogUsingPower(20.0);
    Serial.print("position: ");
    Serial.println(motors[motorId].actualPosition);
    // motors[Motors::LINEAR].moveAbs(11.0);
  }
  else if (inputs.ShiftDownSw && clutchIsDisengaged)
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, motorCfgs[Motors::LINEAR].invertDir);
    motors[motorId].enable();
    motors[motorId].jogUsingPower(-20.0);
    Serial.print("position: ");
    Serial.println(motors[motorId].actualPosition);
    // motors[Motors::LINEAR].moveAbs(6.0);
  }
  else
  {
    // digitalWrite(PIN_LINEAR_P_PWM, LOW);
    motors[motorId].stop();
    // motors[Motors::LINEAR].disable();
  }
}

bool runMotorsThisScan = false;
uint8_t motorId = 0;
void updateMotors()
{
  runMotorsThisScan = !runMotorsThisScan;
  motors[motorId].update();
  motors[motorId].run();
  motorId = (motorId + 1) % NUM_MOTORS;
  // for (int i = 0; i < NUM_MOTORS; i++)
  // {
  //   motors[i].update();
  //   if (runMotorsThisScan)
  //   {
  //     //motors[i].run();
  //   }
  // }
}

StateManager clutchMotorHoming; // state manager for homing routine clutch motor
// homes motor to neg lim switch and "referred to as home sw", ends in the pedaling position
bool runHomingRoutineClutchMotor(bool reset = false)
{
  if (reset)
  {
    clutchMotorHoming.transitionToStep(0);
  }
  else
  {
    switch (clutchMotorHoming.Step)
    {
    case 0:
      clutchMotorHoming.StepDescription("Initializing homing process");
      // Initialize homing process
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchMotorHoming.transitionToStep(10);
      }
      else
      {
        motors[Motors::CLUTCH].enable();
      }
      break;
    case 10:
      // Move clutch motor to disengaged position
      motors[Motors::CLUTCH].home();
      if (motors[Motors::CLUTCH].getState() == Motor::States::HOMING)
      {
        clutchMotorHoming.transitionToStep(11);
      }
      break;
    case 11:
      clutchMotorHoming.StepDescription("Waiting for clutch motor to finish homing");
      // Wait for clutch motor to reach disengaged position
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE && motors[Motors::CLUTCH].isHomed)
      {
        clutchMotorHoming.transitionToStep(20);
      }
      break;
    case 20:
      clutchMotorHoming.StepDescription("Requesting clutch motor move to pedaling position");
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING);
      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchMotorHoming.transitionToStep(21);
      }
      break;
    case 21:
      clutchMotorHoming.StepDescription("Waiting for clutch motor to reach pedaling position");
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchMotorHoming.transitionToStep(22);
      }
      break;
    case 22:
      setClutchRolloverPosition();
      clutchMotorHoming.transitionToStep(1000);
      break;
    case 1000:
      clutchMotorHoming.StepDescription("Clutch motor is now homed");
      return true; // Clutch motor is now homed
      break;
    }
  }
  return false;
}

StateManager stateManagerHRLM; // state manager for homing routine linear motors
bool runHomingRoutineLinearMotors(bool reset = false)
{
  if (reset)
  {
    stateManagerHRLM.transitionToStep(0);
  }
  else
  {
    switch (stateManagerHRLM.Step)
    {
    case 0:
      isHomed = false;
      stateManagerHRLM.transitionToStep(1);
      break;
    case 1:
      disengageClutch(true);
      iSerial.debugPrintln("HOMING LINEAR MOTORS - Moving clutch motor to disengaged Position");
      stateManagerHRLM.transitionToStep(10);
      break;

    case 10: // MOVING CLUTCH TO DISENGAGED POSITION

      if (disengageClutch())
      {
        stateManagerHRLM.transitionToStep(20);
      }
      break;

    case 20: // ENABLE LINEAR MOTOR

      motors[Motors::LINEAR_P].enable();
      motors[Motors::LINEAR_S].enable();
      disengageClutch();
      if (motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        iSerial.debugPrintln("HOMING - Moving primary motor to postive hardstop and secondary motor to negative hard stop");
        stateManagerHRLM.transitionToStep(50);
      }
      break;

    case 50:
      stateManagerHRLM.StepDescription("request homing for linear motors");

      disengageClutch();
      motors[Motors::LINEAR_P].home();
      motors[Motors::LINEAR_S].home();
      if (motors[Motors::LINEAR_P].getState() == Motor::States::HOMING && motors[Motors::LINEAR_S].getState() == Motor::States::HOMING)
      {
        stateManagerHRLM.transitionToStep(51);
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
        iSerial.debugPrintln("HOMING - Moving linear motors to 1st gear");
        stateManagerHRLM.transitionToStep(61);
      }
      break;
    case 61:
      stateManagerHRLM.StepDescription("waiting for linear motors to reach gear 1 positions");
      disengageClutch();
      if (motors[Motors::LINEAR_P].atPosition && motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].atPosition && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        iSerial.debugPrintln("HOMING - Linear motors are at gear 1 positions");
        engageClutch(true);
        stateManagerHRLM.transitionToStep(70);
      }
      else if (stateManagerHRLM.getStepActiveTime() > 5000)
      {
        triggerError("runHomingRoutineLinearMotors() - Linear motors did not reach gear 1 positions in time");
        iSerial.debugPrintln("HOMING - Linear motors did not reach gear 1 positions in time");
        iSerial.status.step = 911; // go to error step
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

void printDiagnosticDataToJsonString()
{

  iSerial.writeString("{\"diagnostic\": ");

  // innerJson
  iSerial.writeString("{\"error\": ");

  iSerial.writeString("[");
  int16_t DELAY_TIME_US = 200;
  if (diagnosticData.numOfDataPoints > 0)
  {
    for (int i = 0; i < diagnosticData.numOfDataPoints; i++)
    {
      while (Serial.availableForWrite() <= 8)
      {
        delayMicroseconds(DELAY_TIME_US);
      }

      iSerial.writeString(String(diagnosticData.error[i]));

      if (i < diagnosticData.numOfDataPoints - 1)
      {
        iSerial.writeString(", ");
      }
    }
  }

  delayMicroseconds(500); // allow buffer to build up
  iSerial.writeString("]");
  iSerial.writeString(", \"cmd\": ");
  iSerial.writeString("[");

  if (diagnosticData.numOfDataPoints > 0)
  {
    for (int i = 0; i < diagnosticData.numOfDataPoints; i++)
    {
      while (Serial.availableForWrite() <= 8)
      {
        delayMicroseconds(DELAY_TIME_US);
      }
      iSerial.writeString(String(diagnosticData.cmd[i]));
      if (i < diagnosticData.numOfDataPoints - 1)
      {
        iSerial.writeString(", ");
      }
    }
  }

  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString("]");

  iSerial.writeString(", \"targetGear\": " + String(diagnosticData.targetGear));
  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString(", \"targetPosition\": " + String(diagnosticData.targetPosition));
  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString(", \"actualGear\": " + String(diagnosticData.actualGear));
  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString(", \"actualPosition\": " + String(diagnosticData.actualPosition));

  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString(", \"numOfDataPoints\": ");
  delayMicroseconds(700); // allow buffer to build up
  iSerial.writeString(String(diagnosticData.numOfDataPoints));

  delayMicroseconds(700);   // allow buffer to build up
  iSerial.writeString("}"); // closing the innerDoc
  iSerial.writeString("}"); // closing the outerDoc

  // Create the inner object JSON string
  // String innerJson = "{\"error\": " + diagnosticData.errorString + ", \"cmd\": " + cmdArrayString + ", \"numOfDataPoints\": " + String(lenArray) + "}";

  // Create the outer object JSON string and nest the inner object inside it
  // String outerJson = "{\"diagnostic\": " + innerJson + "}";

  // Print the JSON string
  // Serial.println(outerJson);

  // return outerJson;
  delayMicroseconds(7000); // allow buffer to build up
}

void processRelPosCmd()
{
  // int motorId = iSerial.idChr - '0';
  if (!iSerial.parseLong(tempLong))
  {
    if (tempLong > 0)
    {
      serialShiftReqType = Shifter::ShiftTypes::UP;
    }
    else if (tempLong < 0)
    {
      serialShiftReqType = Shifter::ShiftTypes::DOWN;
    }

    iSerial.writeCmdChrIdChr();
    iSerial.writeLong(tempLong);
    iSerial.writeNewline();
  }
  else
  {
    iSerial.writeCmdWarning("could not parse position data");
  }
}

void processAbsPosCmd()
{
  // int motorId = iSerial.idChr - '0';
  if (!iSerial.parseLong(tempLong))
  {
    serialShiftReqType = Shifter::ShiftTypes::ABS;
    serialShiftTargetGearParam = tempLong;

    iSerial.writeCmdChrIdChr();
    iSerial.writeLong(tempLong);
    iSerial.writeNewline();
  }
  else
  {
    iSerial.writeCmdWarning("could not parse position data");
  }
}

// handles serial cmds that aren't already handled by ISerial (connect, mode, debug)
void handleSerialCmds()
{
  // int idx = iSerial.idChr - '0';

  iSerial.handleBasicSerialCmds();

  switch (iSerial.cmdChr)
  {
  case Cmds::CLEAR_CMD:
    clearErrors();
    break;
  case Cmds::ABSPOS_CMD:
    // processAbsPosCmd();
    break;

  case Cmds::HOME_CMD:
    clearErrors();
    iSerial.writeCmdChrIdChr();
    iSerial.writeNewline();
    // iSerial.setNewMode(Modes::RESETTING);
    break;

  case Cmds::RELPOS_CMD:
    // processRelPosCmd();
    break;

  case Cmds::SERVOPOSINFO_CMD:
    // processServoPosInfoCmd();
    break;

  case Cmds::SERIAL_OUTPUT: // prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
    sendDiagnosticData();
    break;

  case Cmds::PARAMS_SET: // set params
    // processParamsCmd();
    break;

  default:
    processUnrecognizedCmd();
    break;
  }
}

void sendDiagnosticData()
{
  iSerial.writeCmdChrIdChr();
  printDiagnosticDataToJsonString();
  // iSerial.writeString(jsonString);
  iSerial.writeNewline();
  // iSerial.debugPrint("iSerial.status.mode: ");
  // iSerial.debugPrintln(String(iSerial.status.mode));
}

void processUnrecognizedCmd()
{
  String msg1 = "didn't recognize cmdChr: ";
  msg1.concat(char(iSerial.cmdChr));
  iSerial.writeCmdWarning(msg1);
}

void turnAllOff() // turn off all outputs
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].disable();
  }
  // digitalWrite(PIN_CLUTCH_PWM, 0);
  // digitalWrite(PIN_LINEAR_P_PWM, 0);

  // if(dimitriCfg.hasClutchSolenoid){
  //   digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
  // }
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

// JSON HELPERS

// retrieve the subjson object
String subJsonString(String &dataStr, bool &success, String name)
{
  // ServoSpeed::Params params;
  success = true;
  String payload;
  dataStr.replace(" ", "");
  payload = dataStr;

  // the success bool is initially true, if any of the checkNotEmptyString finds an empty value, it will set that success to false
  String subJsonStr = ISerial::checkNotEmptyString(jsonExtract(payload, name), success);

  return subJsonStr;
}

void updateIo()
{
  int sensorNum = 0;
  iSerial.setIo(sensorNum, inputs.ClutchNegLimSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ClutchPosLimSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftUpSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftDownSw);
}

// returns true if the target changed from previous
bool setTargetGear(int targetGearNum)
{
  iSerial.debugPrintln("setTargetGear()");
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
    iSerial.debugPrintln("STORING NEW SHIFT DATA");
    iSerial.debugPrint("Target Gear: ");
    iSerial.debugPrintln(String(shiftData.targetGear));
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

void loop()
{
  // iSerial.debugPrintln("Dimitri loop() started");
  unsigned long timeNowUs = micros();
  if (timeNowUs - lastUpdateUs > SCAN_TIME_US)
  {
    if (timeNowUs - lastUpdateUs > 1.10 * SCAN_TIME_US)
    {
      // iSerial.debugPrintln("WARNING: long scan time detected: " + String(timeNowUs - lastUpdateUs) + "usec");
    }
    lastUpdateUs = timeNowUs;

    if (false)
    {
      Serial.print(motors[Motors::CLUTCH].actualPosition);
      Serial.print(", ");
      Serial.print(motors[Motors::LINEAR_P].actualPosition);
      Serial.print(", ");
      Serial.print(motors[Motors::LINEAR_S].actualPosition);
      Serial.print(", ");
      Serial.println(shiftTimeMs);
    }

    if (false && iSerial.status.step != prevStep)
    {
      prevStep = iSerial.status.step;

      Serial.print(iSerial.status.mode);
      Serial.print(", ");
      Serial.print(iSerial.status.step);
      Serial.print(", ");
      Serial.print(errors.list[0]);
      /*
      Serial.print(shiftData.targetGear);
      Serial.print(", ");
      Serial.println(motionData.actualGear);
      */
      Serial.print(", CLUTCH: ");
      Serial.print(motors[Motors::CLUTCH].getState());
      // Serial.print(", ");
      // Serial.print(motors[Motors::CLUTCH].actualPosition);
      // Serial.print(", ");
      /*
      Serial.println(motors[Motors::CLUTCH].actualVelocity);
      //Serial.print(", ");
      //Serial.println(100*int(motors[Motors::CLUTCH].isStill));
      */
      Serial.print(", LINEAR P: ");
      Serial.print(motors[Motors::LINEAR_P].getState());
      Serial.print(", ");
      Serial.println(motors[Motors::LINEAR_P].actualPosition);
      // Serial.print(", ");
      // Serial.print(motors[Motors::LINEAR].actualVelocity);
      // Serial.print(", ");
      // Serial.println(String(motors[Motors::LINEAR].getOutputPower()));
      Serial.print(", LINEAR S: ");
      Serial.print(motors[Motors::LINEAR_S].getState());
      Serial.print(", ");
      Serial.println(motors[Motors::LINEAR_S].actualPosition);
    }

    // for (int i = 0; i < NUM_MOTORS; i++)
    // {
    //   motors[i].run();
    // }
    readInputs();
    timeNow = millis();

    if (timeNow - lastDisplayed_ms >= 500)
    {
      lastDisplayed_ms = timeNow;
      if (false)
      {
        // iSerial.debugPrint("CLUTCH MOTOR Actual Position: ");
        // iSerial.debugPrintln(String(encoders[Motors::CLUTCH].read()));
      }

      if (errors.present)
      {
        sendErrorData();
      }
    }

    resetReq = inputs.ShiftDownSw && inputs.ShiftUpSw;
    // AUTO ABORT IF ERROR
    if (errors.present && iSerial.status.mode >= int(Modes::RESETTING))
    {
      iSerial.setNewMode(Modes::ABORTING);
    }
    else if (resetReq && iSerial.status.mode == int(Modes::IDLE))
    {
      // iSerial.setNewMode(Modes::RESETTING);
    }

    switch (iSerial.status.mode)
    {
    case Modes::ABORTING:
      turnAllOff();
      iSerial.setNewMode(Modes::KILLED); // TODO: this may break the raspi code, may want raspi to initiate the mode change
      break;
    case Modes::KILLED:
      turnAllOff();

      if (errors.present)
      {
        iSerial.setNewMode(Modes::ERROR);
      }
      else
      {
        iSerial.setNewMode(Modes::INACTIVE);
      }

      break;
    case Modes::ERROR:
      shiftData.targetGear = 0;
      isHomed = false;
      if (iSerial.status.step == 0)
      {
        turnAllOff();
        iSerial.status.step = 1;
      }
      else if (iSerial.status.step == 1)
      {
        // auto clear the errors after 5 seconds
        if (resetReq)
        {
          clearErrors();
        }
        if (!errors.present)
        {
          iSerial.setNewMode(Modes::INACTIVE);
        }
        if (iSerial.modeTime() > 3000 && false)
        {
          Serial.print("ERROR: ");
          Serial.println(errors.list[0]);
          iSerial.resetModeTime();
        }
      }

      break;
    case Modes::INACTIVE:
      turnAllOff();
      if (OPERATING_MODE == OperatingModes::AUTO)
      {
        if (AUTO_RESET || (resetReq))
        {
          iSerial.setNewMode(Modes::RESETTING);
        }
      }
      else
      {
        iSerial.setNewMode(Modes::MANUAL);
      }
      break;

    case Modes::RESETTING:
      if (iSerial.status.step == 0)
      {
        // skip homing if already homed
        if (isHomed)
        {
          iSerial.debugPrintln("RESETTING: already homed, skipping homing routine");
          if (enableAllMotors())
          {
            iSerial.setNewMode(Modes::IDLE);
          }
        }
        else
        {
          iSerial.debugPrintln("RESETTING: running homing routines");
          runHomingRoutineClutchMotor(true);

          iSerial.status.step = 51;
        }
      }
      else if (iSerial.status.step == 51)
      {

        if (runHomingRoutineClutchMotor())
        {
          runHomingRoutineLinearMotors(true); // this resets the homing routine
          iSerial.status.step = 55;
        }
      }
      else if (iSerial.status.step == 55)
      {
        if (runHomingRoutineLinearMotors())
        {
          iSerial.setNewMode(Modes::IDLE);
        }
      }
      break;

    case Modes::IDLE:
      if (iSerial.status.step == 0)
      {
        // digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
        if (gearChangeReq)
        {
          shiftStartTimeMs = timeNow;
          iSerial.setNewMode(Modes::SHIFTING);
        }
        else if (!atTarget)
        {
          // triggerError(Errors::MOTOR_NOT_AT_TARGET_WHILE_IDLE);
          iSerial.debugPrintln("linear motors not at target while in idle, adjust position now");
          iSerial.status.mode = Modes::SHIFTING;
        }
        else if (resetReq)
        {
          iSerial.setNewMode(Modes::RESETTING);
        }
      }
      break;

    case Modes::SHIFTING:
      if (iSerial.status.step == 0)
      {
        iSerial.resetModeTime();
        disengageClutch(true);
        iSerial.status.step = 1;
      }
      else if (iSerial.status.step == 1)
      {
        if (disengageClutch())
        {
          iSerial.resetModeTime();
          moveLinearMotorsToGear(shiftData.targetGear, true);
          iSerial.status.step = 10;
        }
      }

      else if (iSerial.status.step == 10) // moving linear motors
      {
        disengageClutch();

        if (moveLinearMotorsToGear(shiftData.targetGear) && atTarget)
        {
          iSerial.resetModeTime();
          iSerial.status.step = 20;
        }
        else if (gearChangeReq)
        {
          iSerial.resetModeTime();
          iSerial.status.step = 1;
        }
      }
      else if (iSerial.status.step == 20)
      {
        // digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
        engageClutch(true);
        // reset mode time
        iSerial.resetModeTime();
        iSerial.status.step = 25;
      }
      else if (iSerial.status.step == 25) // MOVE CLUTCH TO PEDALING POSITION: MOVE CLUTCH MOTOR TO PEDALING POSITION
      {
        if (engageClutch())
        {
          shiftTimeMs = timeNow - shiftStartTimeMs;
          iSerial.setNewMode(Modes::IDLE);
        }
        else if (iSerial.modeTime() > 4000)
        {
          triggerError("SHIFTING: TOGGLE_MOTOR_ENGAGE_MOVE_TIMED_OUT");
        }
      }
      break;

    case Modes::MANUAL:
      if (iSerial.status.step == 0)
      {
        iSerial.debugPrintln("MANUAL MODE");
        disengageClutch(true);
        iSerial.status.step = 1;
      }
      else if (iSerial.status.step == 1)
      {
        switch (OPERATING_MODE)
        {
        case OperatingModes::MANUAL_CLUTCH:
          runClutchMotorManualMode();
          break;
        case OperatingModes::MANUAL_LINEAR_P:
          runLinearMotorManualMode(Motors::LINEAR_P);
          break;
        case OperatingModes::MANUAL_LINEAR_S:
          runLinearMotorManualMode(Motors::LINEAR_S);
          break;
        }
      }
      break;

    default:
      break;
    }

    if (iSerial.taskProcessUserInput())
    {
      handleSerialCmds(); // inside this command, the serialShiftReqType is assigned (if received)
    }

    checkActualGear();
    updateGearNumberDigitalOutputs(shiftData.targetGear);
    gearChangeReq = false;

    // GETS TEH GEAR CHANGE REQUEST
    int shiftTypeReq = checkForGearShiftRequests();

    if (shiftTypeReq > 0)
    {
      if (!checkShiftTypeAndHomeSw(shiftTypeReq))
      {
        iSerial.debugPrintln("WARNING: downshift request not accept accepted because of homeSw state");
      }
      else if (iSerial.status.mode == Modes::IDLE || iSerial.status.mode == Modes::SHIFTING)
      {
        gearChangeReq = processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
        if (gearChangeReq)
        {
          iSerial.debugPrint("shiftType: ");
          iSerial.debugPrintln(String(shiftTypeReq));
          sendShiftData();
          initializeDiagnosticData();
        }
      }
    }

    // RESET VALUES EACH LOOP - reset values at end of each loop, like OTEs and Events
    iSerial.event = 0;

    // DEBUG PASSTHROUGHS - comment out as needed
    // solenoids[0].setDebug(iSerial.debug);
  }
}
