bool FLIP_POSITIVE = false;          // This should normally be false, just used for testing directional differences in motor cmd
bool SHIFT_BUTTONS_DISABLED = false; // if this is true, the shift buttons will be disabled (ignored)
bool SKIP_HOMING = true;
bool OUTPUTS_DISABLED = false; // used for testing encoder and other stuff
bool HAS_SOLENOID = false;    // if this is true, the solenoid will be used to hold the cam in place

#include "Arduino.h"
#include "ISerial.h"
#include "Encoder.h"
#include "Motor.h"
#include "Shifter.h"
#include "CustomDataTypes.h"

////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////

// TOGLGLE MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the engaged direction
#define PIN_TOGGLE_EN 3
#define PIN_TOGGLE_PWM 4     
#define PIN_TOGGLE_FWD 5      // 
#define PIN_TOGGLE_REV 6      // 


// LINEAR MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the down shift direction
#define PIN_LINEAR_EN 7
#define PIN_LINEAR_PWM 8
#define PIN_LINEAR_FWD 9       
#define PIN_LINEAR_REV 10      

#define PIN_LINEAR_ENC_A 18   //
#define PIN_LINEAR_ENC_B 19   //
#define PIN_TOGGLE_ENC_A 20   //
#define PIN_TOGGLE_ENC_B 21   //
#define PIN_EINK_BIT0 22      // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER
#define NUM_BITS 4
#define PIN_POS_LIM 41        // 
#define PIN_CAM 43            // 
#define PIN_SHIFT_UP 51       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 53     // RED WIRE ON SHIFTER

struct Inputs
{
  bool ShiftDownSw = false;
  bool ShiftUpSw = false;
  //bool HomeSw = false; // Wired N.C., so that is is OFF when target is Detected
  bool CamSw = false;
  bool PosLimSw = false;
};
Inputs inputs;

////////////////////////////////////////////////////
// OUTPUTS
////////////////////////////////////////////////////

struct Outputs
{
  //float MotorSpeed = 0.0; // range -100.0% to 100.0%, this will control the direction pin as well
  //float SolPwr = 0.0;     // range 0.0 % to 100 %
  bool DisplayBits[NUM_BITS] = {false, false, false, false};
};
Outputs outputs;

// MOTOR CONTROL BOARD
const int PWM_FREQUENCY_HZ = 2000; // Desired PWM frequency in Hz

// SOLENOID STOPPER
const float NOMINAL_SOL_PWR_PERC = 100.0;
const float NUDGE_SOL_PWR_PERC = 100.0;

#define NUM_MOTORS 1
enum Motors {
  TOGGLE=0,
  LINEAR=1,
};

// ENCODERS

Encoder encoders[NUM_MOTORS] = {
  //Encoder(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B),
  Encoder(PIN_LINEAR_ENC_A, PIN_LINEAR_ENC_B)
};

Encoder encoderToggle = Encoder(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B);

Motor motors[NUM_MOTORS] = {
  //Motor(PIN_TOGGLE_FWD, PIN_TOGGLE_REV, PIN_TOGGLE_PWM, &encoders[0]), //TOGGLE
  Motor(PIN_LINEAR_FWD, PIN_LINEAR_REV, PIN_LINEAR_PWM, &encoders[1])  //LINEAR
};

// MOTION PROFILE
MotionData motionData;

bool sw = false;
int time_now;
long last_time = millis();

// SHIFTER

Shifter shifter(PIN_SHIFT_UP, PIN_SHIFT_DOWN);
bool gearChangeReq = false;
int shiftTargetGearParam = 0;

// SERIAL SHIFTER DATA
int serialShiftReqType = 0;
int serialShiftTargetGearParam = 0;
int atTargetCnt = 0;

// SHIFT DATA
const float TARGET_TOLERANCE = 5.0;
ShiftData shiftData;

// MOTOR CONTROL LAW CONTROLLER

const double MIN_SPEED_REF_PERC = 25.0;   // formerly 22.0
const double MAX_SPEEDREF_PD_PERC = 30.0; // max speedRef during PD control mode (perc)
const double NUDGE_POWER_PERC = 100.0;    // speedRef used during intial impulse (nudge) to get the motor to move (perc)
const int NUDGE_TIME_MS = 150;            // duration of intial impulse cmd to get the motor to move (ms)
const int HOMING_NUDGE_TIME_MS = NUDGE_TIME_MS;
const double NOMINAL_SPEEDREF_PERC = 30.0;           // nominal speedRef for runControler() when far from target (perc)
const double NEAR_TARGET_DIST = 60.0;                // distance to be considered near the target, where mode switches to PD control
//PDController controller = PDController(1.20, 0.000); // NOTE: modify these parameters to improve the control, start with pd set to zero
#define NUM_GEARS 12
#define MIN_POSITION 0.0
#define MAX_POSITION (MIN_POSITION + float(NUM_GEARS - 1) * 360.0)
// // int targetGear = 1;                       // range is from 1 to NUM_GEARS, does not start at 0
// int actualGear = 1;                       // range is from 1 to NUM_GEARS, does not start at 0
bool controllerOn = false;                // set this true to activate outputs related to the controller, set to false to kill those outputs
unsigned long activationStartTime_ms = 0; // the time the controller first became activated
const int SOLENOID_RETRACT_TIME_MS = 100;
const int CONTROLLER_TIME_LIMIT_MS = 800; // max time the controller is allowed to be active

bool atTarget = false;
bool atTargetAndStill = false;


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

void setup()
{
  iSerial.init();
  iSerial.THIS_DEVICE_ID = BIKE_MEGA_ID;
  iSerial.setAutoSendStatus(true); // status updates sent when mode changes

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].init();
  }

  iSerial.debug = true;
  iSerial.setNewMode(Modes::ABORTING);

  // encoder.setDebug(true);
}

// int deleteMe = 0;
// int deleteMeSign = 1;
StopWatch stopWatch;
void loop()
{
  // updateIo();
  unsigned long timeNow = millis();
  if (timeNow - 500 >= lastDisplayed_ms && (iSerial.isConnected || true))
  {
    lastDisplayed_ms = timeNow;
    Serial.print("TOGGLE actualPosition: ");
    Serial.println(encoderToggle.read());

    if (iSerial.status.mode != Modes::SHIFTING)
    {
      //iSerial.sendStatus(true);
      //sendMotionData();
      //sendShiftData();
    }
    if (errors.present)
    {
      //sendErrorData();
    }
  }

  switch (iSerial.status.mode)
  {
  case Modes::ABORTING:
    turnAllOff();
    iSerial.setNewMode(Modes::KILLED); // TODO: this may break the raspi code, may want raspi to initiate the mode change
    break;
  case Modes::KILLED:
    //turnAllOff();
    //sendShiftData();
    // solenoids[Solenoids::STOPPER].changeMode(Solenoid::ON, 255);
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
    if (iSerial.status.step == 0)
    {
      //turnAllOff();
      //sendErrorData();
      //sendMotionData();
      //sendShiftData();
      iSerial.status.step = 1;
    }
    else if (iSerial.status.step == 1)
    {
      // auto clear the errors after 10 seconds
      if (iSerial.modeTime() > 5000)
      {
        //clearErrors();
      }
      if (!errors.present)
      {
        iSerial.setNewMode(Modes::INACTIVE);
      }
    }

    break;
  case Modes::INACTIVE:
    turnAllOff();

    if (SKIP_HOMING)
    {
      iSerial.setNewMode(Modes::IDLE);
    }
    else
    {
      iSerial.setNewMode(Modes::RESETTING);
    }
    break;

  case Modes::RESETTING:
    // TODO: insert homing logic here, home switch should be wired n.c.

    if (iSerial.status.step == 0)
    {
      //turnAllOff();
      

      //runHomingRoutine(); // this resets the homing routine
      iSerial.status.step = 1;
    }
    else if (iSerial.status.step == 1)
    {
      int homingStatus = 1;

      if (100 == homingStatus) // HOMING DONE
      {
        iSerial.setNewMode(Modes::IDLE);
        //setTargetGear(motionData.actualGear);
      }
      else if (911 == homingStatus) // HOMING ERROR
      {
        iSerial.setNewMode(Modes::ABORTING);
      }
    }
    break;

  case Modes::IDLE:
    if (iSerial.status.step == 0)
    {
      //turnAllOff();
      //sendShiftData();
      //printMotionData();
      // sendDiagnosticData();
      iSerial.status.step = 1;
    } else if (iSerial.status.step == 1)
    {
      
    }

    if (errors.present)
    {
      iSerial.setNewMode(Modes::ABORTING);
    }
    else if (gearChangeReq)
    {
      iSerial.setNewMode(Modes::SHIFTING);
      // printMotionData();
    }

    break;
  case Modes::SHIFTING:

    if (iSerial.status.step == 0)
    {
      //activateController();

      iSerial.status.step = 1;
      stopWatch.startTime = millis();
      stopWatch.loopCnt = 0;
      stopWatch.maxLoopTime = 0;
      atTargetCnt = 0;
    }
    else if (iSerial.status.step == 1)
    {
      if (errors.present)
      {
        stopWatch.stopTime = millis();
        iSerial.debugPrint("stopWatch time: ");
        iSerial.debugPrintln(String(stopWatch.stopTime - stopWatch.startTime));
        iSerial.debugPrint("stopWatch loopCnt: ");
        iSerial.debugPrintln(String(stopWatch.loopCnt));
        iSerial.debugPrint("stopWatch avgLoopTime us: ");
        iSerial.debugPrintln(String(1000.0 * float(stopWatch.stopTime - stopWatch.startTime) / float(stopWatch.loopCnt)));
        iSerial.debugPrint("stopWatch maxLoopTime: ");
        iSerial.debugPrintln(String(stopWatch.maxLoopTime));

        // sendDiagnosticData();
        //turnOffController();
        iSerial.setNewMode(Modes::ERROR);
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

  int shiftTypeReq = checkForGearShiftRequests();
  gearChangeReq = false;
  if (shiftTypeReq > 0)
  {
    if (!checkShiftTypeAndHomeSw(shiftTypeReq))
    {
      iSerial.debugPrintln("WARNING: downshift request not accept accepted because of homeSw state");
    }
    else if (iSerial.status.mode != Modes::IDLE && iSerial.status.mode != Modes::SHIFTING)
    {
      iSerial.debugPrintln("WARNING: shift request not accpeted because motor state not idle or already shifting");
    }
    else // NORMAL CASE
    {
      gearChangeReq = processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
      if (gearChangeReq)
      {
        iSerial.debugPrint("shiftType: ");
        iSerial.debugPrintln(String(shiftTypeReq));
        //sendShiftData();
        atTarget = false;
        atTargetAndStill = false;
        //initializeDiagnosticData();
      }
    }
  }

  //runAll(); // runs all things that have a run() method that need to be called each loop

  // RESET VALUES EACH LOOP - reset values at end of each loop, like OTEs and Events
  iSerial.event = 0;

  // DEBUG PASSTHROUGHS - comment out as needed
  // solenoids[0].setDebug(iSerial.debug);
}

// checks the homeSw state and shiftType, prevents downshifting if homeSw detects target
bool checkShiftTypeAndHomeSw(int shiftTypeReq)
{
  if (!inputs.PosLimSw && (shiftTypeReq == Shifter::ShiftTypes::DOWN || shiftTypeReq == Shifter::ShiftTypes::DOWN))
  {
    return false;
  }
  return true;
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
    //clearErrors();
    break;
  case Cmds::ABSPOS_CMD:
    processAbsPosCmd();
    break;

  case Cmds::HOME_CMD:
    //clearErrors();
    iSerial.writeCmdChrIdChr();
    iSerial.writeNewline();
    // iSerial.setNewMode(Modes::RESETTING);
    break;

  case Cmds::RELPOS_CMD:
    //processRelPosCmd();
    break;

  case Cmds::SERVOPOSINFO_CMD:
    // processServoPosInfoCmd();
    break;

  case Cmds::SERIAL_OUTPUT: // prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
    //sendDiagnosticData();
    break;

  case Cmds::PARAMS_SET: // set params
    // processParamsCmd();
    break;

  default:
    //processUnrecognizedCmd();
    break;
  }
}

void turnAllOff() // turn off all outputs
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].disable();
    }
}




void readInputs()
{
  inputs.ShiftUpSw = digitalRead(PIN_SHIFT_UP);
  inputs.ShiftDownSw = digitalRead(PIN_SHIFT_DOWN);
  inputs.CamSw = digitalRead(PIN_CAM);
  inputs.PosLimSw = digitalRead(PIN_POS_LIM);
}

void updateIo()
{
  int sensorNum = 0;
  iSerial.setIo(sensorNum, inputs.ShiftDownSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftUpSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.CamSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.PosLimSw);
}




int getRandomNumber(int min, int max) // get reandom
{
  return random(min, max + 1);
}

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
    //targetChanged = setTargetGear(NUM_GEARS);
    break;
  case Shifter::ShiftTypes::UP:
    iSerial.debugPrintln("UP SHIFT");
    //targetChanged = setTargetGear(shiftData.targetGear + 1);
    break;
  case Shifter::ShiftTypes::SUPER_DOWN:
    iSerial.debugPrintln("SUPER DOWN SHIFT");
    //targetChanged = setTargetGear(1);
    break;
  case Shifter::ShiftTypes::DOWN:
    iSerial.debugPrintln("DOWN SHIFT");
    //targetChanged = setTargetGear(shiftData.targetGear - 1);
    break;
  case Shifter::ShiftTypes::ABS:
    iSerial.debugPrint("ABS SHIFT TO: ");
    iSerial.debugPrintln(String(targetGearParam));
    //targetChanged = setTargetGear(targetGearParam);
    break;
  }
  return targetChanged;
}