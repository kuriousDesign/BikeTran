#define VERSION_NUMBER 53
constexpr int LOOP_UPDATE_RATE_US = 1200; // main loop update rate (1065us is about the fastest it can go with current code)
#define UI_UPDATE_RATE_MS 100
#define CSV_UPDATE_RATE_CYCLES 2 //

#include <Arduino.h>
#include "Arduino.h"
#include "Encoder.h"
#include "Shifter.h"
#include "StateManager.h"
#include "CustomDataTypes.h"
#include "GearMap.h"
#include "Motor.h"
// #include "SerialLogging.h"
#include "FastLogger.h"
#include "Packets.h"
#include <DueTimer.h>

// Motor control update frequency
constexpr float CONTROL_FREQ_HZ = 1e6 * NUM_MOTORS / LOOP_UPDATE_RATE_US; // motor control update frequency
constexpr float CONTROL_PERIOD_US = 1e6 / CONTROL_FREQ_HZ;

// OPERATING MODES: IO_CHECKOUT, MANUAL_CLUTCH_JOGGING, MANUAL_CLUTCH_ENGAGE, MANUAL_LINEAR_P, MANUAL_LINEAR_S, AUTO
const OperatingModes OPERATING_MODE = OperatingModes::MANUAL_CLUTCH_ENGAGE; // set to OperatingModes::AUTO to run the system in debug mode
const DiagnosticModes DIAGNOSTIC_MODE = DiagnosticModes::UI;                // set to DiagnosticModes::SERIAL_OUTPUT to output diagnostic data over serial, otherwise UI mode

const unsigned long DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS = 200; // ms, time to wait after clutch reaches shifting position before linear motor motion is allowed

const bool AUTO_RESET = false;             // if this is true, the system will automatically reset when inactive (no errors)
const bool SHIFT_BUTTONS_DISABLED = false; // if this is true, the shift buttons will be disabled (ignored)

uint8_t motorId = 0;
bool inputs[sizeof(byte)];

////////////////////////////////////////////////////
// OUTPUTS
////////////////////////////////////////////////////

struct Outputs
{
  bool DisplayBits[NUM_BITS] = {false, false, false, false};
};
Outputs outputs;

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

bool atTarget = false;

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
unsigned long lastUiUpdateMs = 0;

const bool SOL_ON = true;
StateManager loopState = StateManager("mainLoop");
StateManager clutchDeviceState = StateManager("clutchEngageState");

void setup()
{
  // Logger::init();
  Serial.begin(115200);
  while (!Serial)
  {
  }
  delay(500);
  if (DIAGNOSTIC_MODE == DiagnosticModes::SERIAL_OUTPUT)
  {
    Logger::setDebug(true);
    Serial.println("");
    String versionNumberString = "VERSION NUMBER ---> " + String(VERSION_NUMBER);
    Logger::info(versionNumberString.c_str());
    String operatingModeString = "OPERATING MODE ---> " + getOperatingModeToString(OPERATING_MODE);
    Logger::info(operatingModeString.c_str());
    loopState.setDebug(true);
    clutchDeviceState.setDebug(true);
    motors[Motors::CLUTCH].setDebug(false);
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
    Serial.println("state of clutch neg lim sw at the pedaling position: " + String(inputs[Inputs::ClutchNegLimSw] ? "ON" : "OFF"));
    Serial.println("state of clutch positive lim sw at the shifting position: " + String(inputs[Inputs::ClutchPosLimSw] ? "ON" : "OFF"));
    Serial.println("state of shift up switch: " + String(inputs[Inputs::ShiftUpSw] ? "ON" : "OFF"));
    Serial.println("state of shift down switch: " + String(inputs[Inputs::ShiftDownSw] ? "ON" : "OFF"));
    delay(1000);
  }
  // Timer1.initialize(UPDATE_TIME_US); // Initialize timer to trigger every X microseconds
  // Timer1.attachInterrupt(updateMotors, UPDATE_TIME_US);
  // Attach ISR to Timer1 (can use Timer0..5)

  // if using DUE board
  Timer1.attachInterrupt(updateISR)
      .start(CONTROL_PERIOD_US); // period in microseconds
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

    case 1:
      clutchDeviceState.StepDescription("enable clutch");
      motors[Motors::CLUTCH].enable();
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(10);
      }
      break;

    case 10:
      clutchDeviceState.StepDescription("move to shift pos");
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_SHIFTING,true);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchDeviceState.transitionToStep(11);
      }
      break;

    case 11:
      clutchDeviceState.StepDescription("wait to reach pos");
      if (motors[Motors::CLUTCH].getState() == Motor::States::HOLD_POSITION)
      {
        motors[Motors::CLUTCH].hold_position();
        clutchDeviceState.transitionToStep(20);
      }
      break;

    case 20:
      clutchDeviceState.StepDescription("hold position");
      motors[Motors::CLUTCH].hold_position();
      if (clutchDeviceState.getStepActiveTime() > DELAY_LINEAR_MOTION_AFTER_CLUTCH_REACHES_SHIFT_POSITION_MS)
      {
        clutchDeviceState.transitionToStep(1000);
      }
      break;
    case 1000: // DONE: HOLD CLUTCH AND HOLDING PWR TO KEEP IT IN DISENGAGED POSITION
      motors[Motors::CLUTCH].hold_position();
      return true;
      break;

    default:
      break;
    }
  }
  return false;
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
      clutchDeviceState.StepDescription("check start pos");
      if (abs(motors[Motors::CLUTCH].actualPosition - POSITION_CLUTCH_SHIFTING) < 5.0)
      {
        motors[Motors::CLUTCH].stop();
        clutchDeviceState.transitionToStep(1);
      }
      else
      {
        triggerError("position not within tolerance");
      }
      break;

    case 1:
      clutchDeviceState.StepDescription("enable clutch");
      motors[Motors::CLUTCH].enable();
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(10);
      }
      break;

    case 10:
      clutchDeviceState.StepDescription("move to pedaling pos");
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING + 360.0);

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        clutchDeviceState.transitionToStep(11);
      }
      break;

    case 11:
      clutchDeviceState.StepDescription("wait to reach pos");
      if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        clutchDeviceState.transitionToStep(20);
      }
      break;

    case 20:
      clutchDeviceState.StepDescription("wait before resetting position");
      if (clutchDeviceState.getStepActiveTime() > 5)
      {
        clutchDeviceState.transitionToStep(80);
      }
      break;
    case 80:
      clutchDeviceState.StepDescription("pseudo-zero the position");
      // at this point the motor position will be near 0 or 360 or 720 or -360 or -720, so i need reset the position so that its closer to zero
      setClutchRolloverPosition();
      clutchDeviceState.transitionToStep(90);
      break;

    case 90:
      clutchDeviceState.StepDescription("check neg lim sw");
      if (inputs[Inputs::ClutchNegLimSw])
      {
        clutchDeviceState.transitionToStep(1000);
      }
      else
      {
        // if the neg lim sw is not pressed, then we need to hold the clutch motor in position
        triggerError("neg lim sw not detected");
      }
      clutchDeviceState.transitionToStep(1000);
      break;

    case 1000:
      clutchDeviceState.StepDescription("done");
      //motors[Motors::CLUTCH].hold_position();
      return true;
      break;

    default:
      break;
    }
  }
  return false;
}

// Sets the clutch motor position to the nearest rollover position around 0
bool setClutchRolloverPosition()
{
  double newPosition = fmod(motors[Motors::CLUTCH].actualPosition - 360.0, 360.0);
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
    Logger::info("SUPER UP SHIFT");
    targetChanged = setTargetGear(NUM_GEARS);
    break;
  case Shifter::ShiftTypes::UP:
    Logger::info("UP SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear + 1);
    break;
  case Shifter::ShiftTypes::SUPER_DOWN:
    Logger::info("SUPER DOWN SHIFT");
    targetChanged = setTargetGear(1);
    break;
  case Shifter::ShiftTypes::DOWN:
    Logger::info("DOWN SHIFT");
    targetChanged = setTargetGear(shiftData.targetGear - 1);
    break;
  case Shifter::ShiftTypes::ABS:
    Logger::info("ABS SHIFT TO: {}", targetGearParam);
    targetChanged = setTargetGear(targetGearParam);
    break;
  }
  return targetChanged;
}

void printMotionData()
{
  Logger::info("Target Gear: {}", shiftData.targetGear);
  Logger::info("Actual Gear: {}", motionData.actualGear);
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
      Logger::error(code.c_str());
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

  if (inputs[Inputs::ShiftUpSw])
  {
    // disengageClutch();
    motors[motor_id].enable();
    if (motors[motor_id].getState() == Motor::States::IDLE || motors[motor_id].getState() == Motor::States::JOGGING)
    {
      motors[motor_id].jogUsingPower(MANUAL_CLUTCH_JOG_PWR);
      String infoMsg = String(motors[motor_id].actualPosition) + " deg";
      Logger::info(infoMsg.c_str());
    }
    if (motors[motor_id].actualPosition < lastPosition)
    {
      Logger::error("motor position decreased when it was expected to increase");
      loopState.transitionToStep(Modes::ERROR);
    }
  }
  else if (inputs[Inputs::ShiftDownSw])
  {
    // disengageClutch();
    motors[motor_id].enable();
    if (motors[motor_id].getState() == Motor::States::IDLE || motors[motor_id].getState() == Motor::States::JOGGING)
    {
      motors[motor_id].jogUsingPower(-MANUAL_CLUTCH_JOG_PWR);
      String infoMsgDown = String(motors[motor_id].actualPosition) + " deg";
      Logger::info(infoMsgDown.c_str());
    }

    if (motors[motor_id].actualPosition > lastPosition)
    {
      Logger::error("motor position increased when it was expected to decrease");
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
        Logger::info("moveLinearMotorsToGear() - Linear motors did not reach target gear position in time");
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
  else if (inputs[Inputs::ShiftUpSw && clutchIsDisengaged])
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, !motorCfgs[Motors::LINEAR].invertDir);
    String infoMsgUp = String(motors[motor_id].actualPosition) + " deg";
    Logger::info(infoMsgUp.c_str());
    motors[motor_id].jogUsingPower(MANUAL_LINEAR_JOG_PWR);
    // SerialLogging::info("jogging motor %d positively - position: %f deg", motor_id, motors[motor_id].actualPosition);

    // motors[Motors::LINEAR].moveAbs(11.0);
  }
  else if (inputs[Inputs::ShiftDownSw] && clutchIsDisengaged)
  {
    // analogWrite(PIN_LINEAR_P_PWM, 255);
    // digitalWrite(PIN_LINEAR_P_DIR, motorCfgs[Motors::LINEAR].invertDir);
    String infoMsgDown = String(motors[motor_id].actualPosition) + " deg";
    Logger::info(infoMsgDown.c_str());
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

unsigned long lastMotorUpdateUs = 0;
void updateMotors()
{
  motors[motorId].run();
  motorId = (motorId + 1) % NUM_MOTORS;
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
        Logger::info("HOMING - Moving primary motor to postive hardstop and secondary motor to negative hard stop");
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
        Logger::info("HOMING - Moving linear motors to 1st gear");
        stateManagerHRLM.transitionToStep(61);
      }
      break;
    case 61:
      stateManagerHRLM.StepDescription("waiting for linear motors to reach gear 1 positions");
      disengageClutch();
      if (motors[Motors::LINEAR_P].atPosition && motors[Motors::LINEAR_P].getState() == Motor::States::IDLE && motors[Motors::LINEAR_S].atPosition && motors[Motors::LINEAR_S].getState() == Motor::States::IDLE)
      {
        Logger::info("HOMING - Linear motors are at gear 1 positions");
        engageClutch(true);
        stateManagerHRLM.transitionToStep(70);
      }
      else if (stateManagerHRLM.getStepActiveTime() > 5000)
      {
        triggerError("runHomingRoutineLinearMotors() - Linear motors did not reach gear 1 positions in time");
        Logger::error("HOMING - Linear motors did not reach gear 1 positions in time");
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

byte *publishedData = new byte[MAX_PACKET_SIZE];
void updateDimitriPacket()
{
  int size = 0;
  // static int prevStep = -1;

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    byte *motorData = motors[i].getMotorData();
    memcpy(publishedData + size, motorData, MOTOR_DATA_SIZE);
    size += MOTOR_DATA_SIZE;
    delete[] motorData;
  }

  // convert loopState.Step to bytes (little-endian)
  int16_t loopStepInt16 = static_cast<int16_t>(loopState.Step);
  publishedData[size++] = static_cast<byte>(loopStepInt16 & 0xFF);        // LSB
  publishedData[size++] = static_cast<byte>((loopStepInt16 >> 8) & 0xFF); // MSB

  // convert loopState.OperationMode to bytes (little-endian)
  publishedData[size++] = static_cast<uint8_t>(OPERATING_MODE);

  // inputs - 1 byte
  uint8_t inputByte = 0;
  for (int i = 0; i < 8; i++)
  {
    inputByte |= (inputs[i] ? 1 : 0) << i;
  }
  publishedData[size++] = inputByte;

  loopStepInt16 = static_cast<int16_t>(clutchDeviceState.Step);
  publishedData[size++] = static_cast<byte>(loopStepInt16 & 0xFF);        // LSB
  publishedData[size++] = static_cast<byte>((loopStepInt16 >> 8) & 0xFF); // MSB
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
  inputs[Inputs::ShiftDownSw] = !digitalRead(PIN_SHIFT_DOWN);
  inputs[Inputs::ShiftUpSw] = !digitalRead(PIN_SHIFT_UP);
  inputs[Inputs::ClutchNegLimSw] = !digitalRead(PIN_CLUTCH_NEG_LIM);
  inputs[Inputs::ClutchPosLimSw] = !digitalRead(PIN_CLUTCH_POS_LIM);
  // inputs[Inputs::LinearNegLimSw = !digitalRead(PIN_LINEAR_LIM_SW) && dimitriCfg.homingDirection == -1;
}

// returns true if the target changed from previous
bool setTargetGear(int targetGearNum)
{
  Logger::info("setTargetGear()");
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
    Logger::info(String(shiftData.targetGear).c_str());
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
unsigned long timeNowUs = 0;
// unsigned long lastUpdateScanUs = 0;
unsigned long lastLoopUpdateScanUs = 0;
unsigned long loopScanTimeUs = 0;

unsigned long motorUpdateScanTimeUs = 0;
unsigned long csvUpdateCycleCount = 0;
bool csvHeaderWritten = false;
void runDimitri()
{
  // SCAN TIME MONITOR

  loopScanTimeUs = timeNowUs - lastLoopUpdateScanUs;
  lastLoopUpdateScanUs = timeNowUs;

  loopState.run();
  readInputs();
  int shiftTypeReq = checkForGearShiftRequests();
  // timeNow = millis();

  // detect resetRequest
  if (inputs[Inputs::ShiftDownSw] && inputs[Inputs::ShiftUpSw])
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
        Logger::error("ERROR: %s", errors.list[0].c_str());
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
      loopState.transitionToStep(54);
    }
    break;
  case 54:
    loopState.StepDescription("moving to clutch motor to engaged position");
    // Wait for clutch motor to reach disengaged position
    // SerialLogging::info("Clutch Motor State: %d", motors[Motors::CLUTCH].getState());
    if (loopState.FirstScan)
    {
      // Start moving clutch to disengaged position
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING);
    }
    else if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
    {
      loopState.transitionToStep(55);
    }
    break;

  case 55:
    loopState.StepDescription("waiting for clutch motor to finish moving to engaged");
    if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
    {
      loopState.transitionToStep(56);
    }
    break;

  case 56:
    loopState.StepDescription("Waiting for clutch motor to move to 0deg");
    // Wait for clutch motor to reach disengaged position
    // SerialLogging::info("Clutch Motor State: %d", motors[Motors::CLUTCH].getState());
    if (motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
    {
      if (OPERATING_MODE == OperatingModes::MANUAL_CLUTCH_ENGAGE)
      {
        loopState.transitionToStep(Modes::MANUAL);
      }
      else
      {
        loopState.transitionToStep(57);
      }
    }
    break;

  case 57:
    loopState.StepDescription("RESETTING - moving clutch to engaged pedaling position");
    if (loopState.FirstScan)
    {
      // Start moving clutch to engaged position
      motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING);
    }

    if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
    {
      loopState.transitionToStep(58);
    }
    break;

  case 58:
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
      Logger::info("linear motors not at target while in idle, adjust position now");
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
      if (inputs[Inputs::ShiftUpSw])
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
    loopState.StepDescription("waiting for clutch disengage");
    if (disengageClutch())
    {
      loopState.transitionToStep(1151);
    }
    break;

  case 1151:
    loopState.StepDescription("holding at disengage");
    if (loopState.getStepActiveTime() > 2000)
    {
      engageClutch(true);
      loopState.transitionToStep(1155);
    }

    break;

  case 1155:
    loopState.StepDescription("waiting for clutch engage");
    if (engageClutch())
    {
      loopState.transitionToStep(1156);
    }
    break;

  case 1156:
    loopState.StepDescription("re-engaged");
    if (loopState.getStepActiveTime() > 2000)
    {
      disengageClutch(true);
      loopState.transitionToStep(Modes::MANUAL);
    }
    break;

  default:
    Logger::info(("DEBUG: UNRECOGNIZED Loop Step: " + String(loopState.Step)).c_str());
    break;
  }

  checkActualGear();
  updateGearNumberDigitalOutputs(shiftData.targetGear);
  gearChangeReq = false;
}

bool updateEveryOtherScan = false;
void updateISR()
{
  // Serial.println("update");
  timeNowUs = micros();
  updateMotors();
  // SCAN TIME MONITOR
  if (timeNowUs - lastMotorUpdateUs > motorUpdateScanTimeUs)
  {
    motorUpdateScanTimeUs = timeNowUs - lastMotorUpdateUs;
  }

  lastMotorUpdateUs = timeNowUs;
  if (motorId == NUM_MOTORS - 1)
  {

    runDimitri();
    switch (DIAGNOSTIC_MODE)
    {
    case DiagnosticModes::UI:
      if (timeNowUs / 1000 - lastUiUpdateMs > UI_UPDATE_RATE_MS)
      {
        updateDimitriPacket();
        lastUiUpdateMs = timeNowUs / 1000;
        Logger::publishPacket(publishedData, getPacketSize(Packets::DIMITRI), Packets::DIMITRI);
      }
      break;
    case DiagnosticModes::CSV:
      // build clutch motor csv using motor position, velocity, power
      if (!csvHeaderWritten)
      {
        String csvHeader = "Time, Clutch Position, Velocity, Power";
        Logger::csv(csvHeader.c_str());
        csvHeaderWritten = true;
      }
      csvUpdateCycleCount++;

      if (csvUpdateCycleCount >= CSV_UPDATE_RATE_CYCLES)
      {
        csvUpdateCycleCount = 0;
        String csvLine = "";
        csvLine += String(motorUpdateScanTimeUs) + ",";
        motorUpdateScanTimeUs = 0;
        csvLine += String(motors[Motors::CLUTCH].actualPosition) + ",";
        csvLine += String(motors[Motors::CLUTCH].actualVelocity) + ",";
        csvLine += String(motors[Motors::CLUTCH].getOutputPower());
        Logger::csv(csvLine.c_str());
      }
      break;
    }
  }
}

void loop()
{
  // NOTE: updateISR() runs every 400us via timer interrupt
  Logger::process();
}