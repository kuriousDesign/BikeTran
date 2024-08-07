bool FLIP_POSITIVE = false;          // This should normally be false, just used for testing directional differences in motor cmd
bool SHIFT_BUTTONS_DISABLED = false; // if this is true, the shift buttons will be disabled (ignored)
bool SKIP_HOMING = true;
bool OUTPUTS_DISABLED = false; // used for testing encoder and other stuff
bool HAS_SOLENOID = false;    // if this is true, the solenoid will be used to hold the cam in place

#include "Arduino.h"
#include "ISerial.h"
#include "Encoder.h"
#include "PDController.h"
#include "EncoderTracker.h"
#include "Shifter.h"
#include "CustomDataTypes.h"

////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////

#define PIN_MOTOR_PWM 4     
#define PIN_MOTOR_DIR 7       // LOW FOR POSITIVE, HIGH FOR NEGATIVE, output for an input to the motor driver board
#define PIN_SOL 6             // must be a pwm pin
#define PIN_ENABLE 8
#define PIN_SHIFT_UP 51       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 53     // RED WIRE ON SHIFTER
#define PIN_POS_LIM 11        // 
#define PIN_CAM 12            // 
#define PIN_LINEAR_ENC_A 21   //
#define PIN_LINEAR_ENC_B 23   //
#define PIN_TOGGLE_ENC_A 25   //
#define PIN_TOGGLE_ENC_B 27   //
#define PIN_EINK_BIT0 22      // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER
#define NUM_BITS 4

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
  float MotorSpeed = 0.0; // range -100.0% to 100.0%, this will control the direction pin as well
  float SolPwr = 0.0;     // range 0.0 % to 100 %
  bool DisplayBits[NUM_BITS] = {false, false, false, false};
};
Outputs outputs;

// MOTOR CONTROL BOARD
const int PWM_FREQUENCY_HZ = 2000; // Desired PWM frequency in Hz

// SOLENOID STOPPER
const float NOMINAL_SOL_PWR_PERC = 100.0;
const float NUDGE_SOL_PWR_PERC = 100.0;




enum Motors {
  TOGGLE=0,
  LINEAR=1,
}

// ENCODERS
Encoder encoders[2] = {
  Encoder(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B),
  Encoder(PIN_LINEAR_ENC_A, PIN_LINEAR_ENC_B)
};

Motor motors[2] = {
  Motor(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B, &encoders[TOGGLE]),
  Motor(PIN_LINEAR_ENC_A, PIN_LINEAR_ENC_B, &encoders[LINEAR])
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
PDController controller = PDController(1.20, 0.000); // NOTE: modify these parameters to improve the control, start with pd set to zero
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

  // initialize solenoid stopper
  pinMode(PIN_SOL, OUTPUT);
  // solenoids[Solenoids::STOPPER].init(PIN_SOL, SolenoidTypes::PWM, SolenoidSafety::CONTINUOUS_OK);

 

  // 
  pinMode(PIN_HOME_SW, INPUT);

  // initialize motor controller outputs
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  setDirPinPositive();
  setupPWM(PIN_MOTOR_PWM, PWM_FREQUENCY_HZ); // initialize motor control pin

  // initialize encoder
  initializeEncoderSystem();
  // iSerial.debug = true;
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
  if (timeNow - 500 >= lastDisplayed_ms && iSerial.isConnected && true)
  {
    lastDisplayed_ms = timeNow;

    if (iSerial.status.mode != Modes::SHIFTING)
    {
      iSerial.sendStatus(true);
      sendMotionData();
      sendShiftData();
    }
    if (errors.present)
    {
      sendErrorData();
    }
  }

  switch (iSerial.status.mode)
  {
  case Modes::ABORTING:
    turnAllOff();
    iSerial.setNewMode(Modes::KILLED); // TODO: this may break the raspi code, may want raspi to initiate the mode change
    break;
  case Modes::KILLED:
    turnAllOff();
    sendShiftData();
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
      turnAllOff();
      sendErrorData();
      sendMotionData();
      sendShiftData();
      iSerial.status.step = 1;
    }
    else if (iSerial.status.step == 1)
    {
      // auto clear the errors after 10 seconds
      if (iSerial.modeTime() > 5000)
      {
        clearErrors();
      }
      if (!errors.present)
      {
        iSerial.setNewMode(Modes::INACTIVE);
      }
    }

    break;
  case Modes::INACTIVE:
    turnAllOff();
    sendShiftData();
    sendErrorData();

    /*
    int time = iSerial.modeTime();
    if (time > 3000 && false)
    {
      setTargetGear((shiftData.targetGear + deleteMeSign) % 10);
      sendShiftData();
      iSerial.setNewMode(Modes::KILLED); // TODO: delete this
    }
    else if (time > 1000)
    {
      deleteMe = shiftData.targetPosition;
    }
    else
    {
      deleteMe = time;
    }
    */
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
      turnAllOff();
      

      runHomingRoutine(); // this resets the homing routine
      iSerial.status.step = 1;
    }
    else if (iSerial.status.step == 1)
    {
      int homingStatus = runHomingRoutine(false);

      if (100 == homingStatus) // HOMING DONE
      {
        iSerial.setNewMode(Modes::IDLE);
        setTargetGear(motionData.actualGear);
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
      turnAllOff();
      sendShiftData();
      printMotionData();
      // sendDiagnosticData();
      iSerial.status.step = 1;
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
      activateController();

      iSerial.status.step = 1;
      stopWatch.startTime = millis();
      stopWatch.loopCnt = 0;
      stopWatch.maxLoopTime = 0;
      atTargetCnt = 0;
    }
    else if (iSerial.status.step == 1)
    {

      if (gearChangeReq)
      {
        // sendShiftData();
        // activateController(); // this will restart the activation time
        iSerial.debugPrintln("gearChangeReq...");
        atTargetCnt = 0;
      }
      else if (checkShiftCompleted()) // TODO: switch logic back to just use atTargetAndStill
      {
        iSerial.debugPrintln("at target!");
        iSerial.debugPrint("shiftTime(ms): ");
        iSerial.debugPrintln(String(iSerial.modeTime()));

        // printMotionData();
        turnOffController();
        iSerial.setNewMode(Modes::IDLE);
      }

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
        turnOffController();
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
        sendShiftData();
        atTarget = false;
        atTargetAndStill = false;
        initializeDiagnosticData();
      }
    }
  }

  runAll(); // runs all things that have a run() method that need to be called each loop

  // RESET VALUES EACH LOOP - reset values at end of each loop, like OTEs and Events
  iSerial.event = 0;

  // DEBUG PASSTHROUGHS - comment out as needed
  // solenoids[0].setDebug(iSerial.debug);
}

// checks the homeSw state and shiftType, prevents downshifting if homeSw detects target
bool checkShiftTypeAndHomeSw(int shiftTypeReq)
{
  if (!inputs.HomeSw && (shiftTypeReq == Shifter::ShiftTypes::DOWN || shiftTypeReq == Shifter::ShiftTypes::DOWN))
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
    processAbsPosCmd();
    break;

  case Cmds::HOME_CMD:
    clearErrors();
    iSerial.writeCmdChrIdChr();
    iSerial.writeNewline();
    // iSerial.setNewMode(Modes::RESETTING);
    break;

  case Cmds::RELPOS_CMD:
    processRelPosCmd();
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

// returns true if current gear is at target position, updates actualGear and atTarget and atTargetAndStill
bool checkPositionAtTarget()
{

  motionData.actualGear = round(motionData.actualPosition / 360.0) + 1;

  // if targetError < 5.0

  if (abs(shiftData.targetPosition - motionData.actualPosition) < TARGET_TOLERANCE)
  {
    atTargetCnt++;

    motionData.actualGear = shiftData.targetGear;
    if (!atTarget)
    {
      // iSerial.debugPrintln("atTarget!");
    }
    atTarget = true;

    if (!tracker.isMoving || atTargetCnt > 7)
    {
      if (!atTargetAndStill)
      {
        iSerial.debugPrint("atTargetAndStill! - ");
        iSerial.debugPrint(String(shiftData.targetPosition));
        iSerial.debugPrint(",");
        iSerial.debugPrintln(String(motionData.actualPosition));
      }
      atTargetAndStill = true;
    }
    else
      atTargetAndStill = false;

    return true;
  }
  else
  {
    atTargetCnt = 0;
    atTarget = false;
    atTargetAndStill = false;
    return false;
  }
}

void processUnrecognizedCmd()
{
  String msg1 = "didn't recognize cmdChr: ";
  msg1.concat(char(iSerial.cmdChr));
  iSerial.writeCmdWarning(msg1);
}

void turnAllOff() // turn off all outputs
{
  disableMotor();
  turnOffController();
  outputs.MotorSpeed = 0.0;
  outputs.SolPwr = 0.0;
}

void turnOffController()
{
  controllerOn = false;
}

void activateController() // just call this once to activate
{
  activationStartTime_ms = millis();
  controllerOn = true;
}

void runAll()
{
  static uint16_t encoderMisreadCnt = 0;
  static bool errorTriggered = false;
  // 1. RUN THE ENCODER - THIS SETS THE FREQUENCY OF UPDATES TO FOLLOW
  bool attemptedReading = encoder.run();

  // 2. UPDATE THE MOTION DATA
  if (encoder.newReadingFlag)
  {
    stopWatch.loopCnt++;
    motionData.actualPosition = tracker.calculatePosition(encoder.position);
    motionData.actualVelocity = tracker.calculateFilteredVelocity();

    if (encoderMisreadCnt > 0)
    {
      iSerial.debugPrintln("encoder misread count reset to zero");
      encoder.setDebug(false);
    }
    encoderMisreadCnt = 0;
  }
  else if (attemptedReading)
  {
    encoderMisreadCnt++;
    if (encoderMisreadCnt % 10000 == 3)
    {
      iSerial.debugPrintln("WARNING: encoder misread count exceeded");
      if (iSerial.debug)
      {
        encoder.setDebug(true);
      }
      triggerError(Errors::ENCODER_MISREAD_COUNT_EXCEEDED);
    }
    else
    {
      encoder.setDebug(false);
    }
  }
  checkPositionAtTarget();

  if (attemptedReading)
  {
    unsigned long scanTime = micros();
    unsigned long loopTime = scanTime - stopWatch.prevScanTime;
    if (loopTime > stopWatch.maxLoopTime && stopWatch.prevScanTime != 0)
    {
      stopWatch.maxLoopTime = loopTime;
    }
    stopWatch.prevScanTime = scanTime;

    readInputs();
    recording = false;
    runController();
    writeOutputs();
    updateIo();
  }
  // runAllSolenoids();
}

bool checkShiftCompleted()
{
  return atTarget && (atTargetAndStill || atTargetCnt > 7) && !tracker.isMoving;
}

void readInputs()
{
  inputs.ShiftUpSw = digitalRead(PIN_SHIFT_UP);
  inputs.ShiftDownSw = digitalRead(PIN_SHIFT_DOWN);
  inputs.HomeSw = digitalRead(PIN_HOME_SW);
}

// responsible for setting the outputs, each time this function is called it will automatically reset all the control values for safety
void writeOutputs()
{

  if (OUTPUTS_DISABLED)
  {
    outputs.MotorSpeed = 0.0;
    outputs.SolPwr = 0.0;
  }

  // MOTOR SPEED - Controls direction pin and pwm pin for motor control board
  double sign = 0.0;
  if (outputs.MotorSpeed > 0.0)
  {
    setDirPinPositive();
    sign = 1.0;
  }
  else if (outputs.MotorSpeed < 0.0)
  {
    setDirPinNegative();
    sign = -1.0;
  }
  else
  {
    setDirPinPositive();
    sign = 0.0;
  }

  int outputValPwm = 0;
  // BOUND THE OUTPUT TO +/- 100.0%
  if (abs(outputs.MotorSpeed) > 100.0)
  {
    outputs.MotorSpeed = sign * 100.0;
    outputValPwm = 255;
  }
  else
  {
    outputValPwm = abs(round(outputs.MotorSpeed * 255.0 / 100.0));
  }

  analogWrite(PIN_MOTOR_PWM, outputValPwm);
  if (recording && diagnosticData.numOfDataPoints < NUM_DIAGNOSTICS_ARRAY)
  {
    // storing diagnostic data into arrays
    diagnosticData.error[diagnosticData.numOfDataPoints] = round(controller.error);
    diagnosticData.cmd[diagnosticData.numOfDataPoints] = round(outputs.MotorSpeed);
    diagnosticData.numOfDataPoints++;
  }
  outputs.MotorSpeed = 0.0; // ALWAYS RESET TO 0.0

  // SOLENOID STOPPERS POWER - controls both solenoids responsible for stopping the motor
  // BOUND THE OUTPUT TO 0.0 to 100.0%
  if (outputs.SolPwr >= 100.0)
  {
    outputs.SolPwr = 100.0;
    digitalWrite(PIN_SOL, HIGH);
  }
  else if (outputs.SolPwr <= 0.0)
  {
    outputs.SolPwr = 0.0;
    digitalWrite(PIN_SOL, LOW);
  }
  else
  {
    digitalWrite(PIN_SOL, LOW);
  }
  // analogWrite(PIN_SOL, round(outputs.SolPwr * 255.0));
  outputs.SolPwr = 0.0; // ALWAYS RESET TO 0.0
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
  iSerial.setIo(sensorNum, inputs.ShiftDownSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftUpSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.HomeSw);
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
    shiftData.targetPosition = 360.0 * (shiftData.targetGear - 1);
    shiftData.startingPosition = round(motionData.actualPosition); // store the actualPosition at the time of gear change request
    // printMotionData();
    checkPositionAtTarget();
    // atTarget = false;
    // atTargetAndStill = false;
    iSerial.debugPrintln("STORING NEW SHIFT DATA");
    iSerial.debugPrint("Target Gear: ");
    iSerial.debugPrintln(String(shiftData.targetGear));
    iSerial.debugPrint("Target Position: ");
    iSerial.debugPrintln(String(shiftData.targetPosition));
    iSerial.debugPrint("Starting Position: ");
    iSerial.debugPrintln(String(shiftData.startingPosition));
  }
  return targetChanged;
}

void initializeDiagnosticData()
{
  diagnosticData.numOfDataPoints = 0;
  diagnosticData.targetGear = shiftData.targetGear;
  diagnosticData.targetPosition = shiftData.targetPosition;
  diagnosticData.actualGear = motionData.actualGear;
  diagnosticData.actualPosition = motionData.actualPosition;
}

int getRandomNumber(int min, int max) // get reandom
{
  return random(min, max + 1);
}

// collects a few encoder values and then zeroes the initial position, make sure the system is powered up with the motor in locked position
void initializeEncoderSystem()
{
  encoder.init();
  // get a few encoder values
  int i = 0;
  while (i < 5)
  {
    i++;
    delayMicroseconds(600);
    encoder.run();
    delayMicroseconds(600);
    encoder.run();
    delayMicroseconds(600);
    encoder.run();
    motionData.actualPosition = tracker.calculatePosition(encoder.position);
    motionData.actualVelocity = tracker.calculateFilteredVelocity();
    // iSerial.debugPrintln(motionData.actualPosition);
    tracker.zeroPosition(encoder.position);
    delay(200);
  }

  setTargetGear(1);
}

void runController()
{
  static bool showedWarning = false;
  unsigned long activeTime_ms = millis() - activationStartTime_ms;
  double speedRef = 0.0;
  float solPwr = 0.0;
  controller.calculatePDSpeedControl(motionData.actualPosition, shiftData.targetPosition); // always allow this to run so that previous values stay valid
  if (controllerOn && activeTime_ms < CONTROLLER_TIME_LIMIT_MS)
  {
    double sign = 0.0;
    if (controller.error < 0.0)
    {
      sign = -1.0;
    }
    else if (controller.error > 0.0)
    {
      sign = 1.0;
    }
    // SOLENOID RETRACTING PHASE
    // Release solenoid initially with full power, than reduce to lower power as long as the absolute target error is greater than 180deg, otherwise turn it off
    if (activeTime_ms < SOLENOID_RETRACT_TIME_MS)
    {
      showedWarning = false;
      speedRef = 0.0;
      solPwr = NUDGE_SOL_PWR_PERC;
    }
    // NUDGE PHASE
    else if (activeTime_ms < SOLENOID_RETRACT_TIME_MS + NUDGE_TIME_MS)
    {
      speedRef = NUDGE_POWER_PERC * sign;
      solPwr = NOMINAL_SOL_PWR_PERC;
    }
    // FAR FROM TARGET PHASE - cruise at nominal power
    else if (abs(controller.error) > NEAR_TARGET_DIST)
    {
      speedRef = NOMINAL_SPEEDREF_PERC * sign;
      solPwr = NOMINAL_SOL_PWR_PERC;
    }
    // NEAR TO TARGET PHASE - pd control
    else if (abs(controller.error) <= NEAR_TARGET_DIST) // if error is within 180.0, switch to pd control and release solenoid
    {
      if (controller.speedControl == 0 || abs(controller.error) <= TARGET_TOLERANCE)
      {
        speedRef = 0.0;
      }
      else
      {
        speedRef = (MAX_SPEEDREF_PD_PERC - MIN_SPEED_REF_PERC) * controller.speedControl / 100.0 + MIN_SPEED_REF_PERC * controller.speedControl / abs(controller.speedControl);
      }
      solPwr = 0.0; // off
    }

    // SET THE PWM OUTPUT TO THE MOTOR BOARD IF ENOUGH TIME HAS BEEN ALLOTTED TO ALLOW SOLENOID TO RETRACT
    if (activeTime_ms >= SOLENOID_RETRACT_TIME_MS)
    {
      outputs.MotorSpeed = speedRef;
    }
    else
    {
      speedRef = 0.0;
      outputs.MotorSpeed = 0.0;
    }

    // SET THE SOLENOID PWR OUTPUT
    outputs.SolPwr = solPwr;
    // printing the diagnostic data as csv stream
    if (false)
    {
      // iSerial.debugPrint(String(round(controller.error)));
      iSerial.debugPrint(", ");
      iSerial.debugPrintln(String(speedRef));
    }

    if (diagnosticData.numOfDataPoints < NUM_DIAGNOSTICS_ARRAY)
    {
      recording = true;
      // storing diagnostic data into arrays
      // diagnosticData.error[diagnosticData.numOfDataPoints] = round(controller.error);
      // diagnosticData.cmd[diagnosticData.numOfDataPoints] = round(outputs.MotorSpeed);
      // diagnosticData.numOfDataPoints++;
    }
  }
  else // motor is not active
  {
    speedRef = 0.0;
    solPwr = 0.0;
    // solenoids[Solenoids::STOPPER].changeMode(Solenoid::OFF);
    if (controllerOn && activeTime_ms >= CONTROLLER_TIME_LIMIT_MS && !showedWarning)
    {
      triggerError(Errors::CONTROLLER_SHIFT_TIMED_OUT);
      showedWarning = true;
      iSerial.debugPrint("ERROR: Controller timed out after ");
      iSerial.debugPrint(String(CONTROLLER_TIME_LIMIT_MS));
      iSerial.debugPrintln(" ms");
      controllerOn = false;
    }
  }
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

// SETUP
void setupPWM(int pin, long frequency)
{
  uint8_t prescalerBits = 0;
  long pwmFrequency;

  // Calculate the prescaler and the closest achievable PWM frequency
  for (prescalerBits = 1; prescalerBits <= 7; prescalerBits++)
  {
    pwmFrequency = 1000000 / (long(pow(2, prescalerBits)) * 510);
    if (pwmFrequency < frequency)
    {
      break;
    }
  }

  // Set the prescaler
  switch (prescalerBits)
  {
  case 1: // 1
    TCCR3B = (TCCR3B & 0b11111000) | 0x01;
    break;
  case 2: // 8
    TCCR3B = (TCCR3B & 0b11111000) | 0x02;
    break;
  case 3: // 64
    TCCR3B = (TCCR3B & 0b11111000) | 0x03;
    break;
  case 4: // 256
    TCCR3B = (TCCR3B & 0b11111000) | 0x04;
    break;
  case 5: // 1024
    TCCR3B = (TCCR3B & 0b11111000) | 0x05;
    break;
  }

  // Set the PWM frequency
  pwmFrequency = 1000000 / (long(pow(2, prescalerBits)) * 510);
  long ocrValue = (long(F_CPU) / (long(pow(2, prescalerBits)) * frequency)) - 1;
  if (ocrValue < 65536)
  {
    ICR3 = ocrValue;
  }
  else
  {
    ICR3 = 65535;
  }

  // Set the PWM mode to Fast PWM
  TCCR3A = (TCCR3A & 0b00111111) | 0b10100000;

  // Set the PWM pin as an output
  pinMode(pin, OUTPUT);

  // Set the PWM to 0 initially
  analogWrite(pin, 0);
}

void printMotionData()
{
  iSerial.debugPrint("Target Gear: ");
  iSerial.debugPrint(String(shiftData.targetGear)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("");

  iSerial.debugPrint("Actual Gear: ");
  iSerial.debugPrint(String(motionData.actualGear)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("");

  iSerial.debugPrint("Target Position: ");
  iSerial.debugPrint(String(shiftData.targetPosition)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg");

  iSerial.debugPrint("Actual Position: ");
  iSerial.debugPrint(String(motionData.actualPosition)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg");

  iSerial.debugPrint("Actual Velocity: ");
  iSerial.debugPrint(String(motionData.actualVelocity)); // Use 1 decimal places for floating-point numbers
  iSerial.debugPrintln("deg/sec");

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
  *q16 = msgPacket->targetPosition;
  q16++;
  *q16 = msgPacket->startingPosition;
  q16++;

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
  *q16 = round(msgPacket->actualPosition);
  q16++;
  *q16 = round(msgPacket->actualVelocity);
  q16++;

  /*
  float *f = (float *)q;
  *f = msgPacket->actualPosition;
  f++;
  *f = msgPacket->actualVelocity;
  f++;
  */
}

void setDirPinPositive()
{
  if (!FLIP_POSITIVE)
  {
    digitalWrite(PIN_MOTOR_DIR, LOW);
  }
  else
  {
    digitalWrite(PIN_MOTOR_DIR, HIGH);
  }
}

void setDirPinNegative()
{
  if (!FLIP_POSITIVE)
  {
    digitalWrite(PIN_MOTOR_DIR, HIGH);
  }
  else
  {
    digitalWrite(PIN_MOTOR_DIR, LOW);
  }
}

int runHomingRoutineOld(bool resetCmd)
{
  // perform series of nudges with solenoids off until it is determined that the motor is in a locked position
  // nudge in a positive direction if the homeSw detects the target, nudge negitive if homeSw doesn't not detect the target

  static bool homingFirstScan = false;
  static int prevStep = 0;
  static int homingStep = 0;
  static long stepStartedTime = millis();
  static int initialPositionReading = 0;
  static int nudgeRetryCnt = 0;
  const int NUDGE_MAX_RETRIES = 5;

  float speedRefPerc = 0.0;
  float solPwr = 0.0;

  if (homingStep != prevStep)
  {
    homingFirstScan = true;
    stepStartedTime = millis();
    prevStep = homingStep;
    iSerial.debugPrint("homeStep: ");
    iSerial.debugPrintln(String(homingStep));
  }
  else
  {
    homingFirstScan = false;
  }
  // calculate the time since the step started
  long stepTime = millis() - stepStartedTime;

  if (resetCmd)
  {
    homingStep = 0;
    prevStep = 0;
    nudgeRetryCnt = 0;
    stepStartedTime = millis();
  }

  switch (homingStep)
  {
  case 0: // RESET: turn everything off
    turnAllOff();
    homingStep = 10;
    break;
  case 10: // determine the nudge direction based on homeSw
    if (!inputs.HomeSw)
    {
      homingStep = 20;
    }
    else
    {
      homingStep = 30;
    }
    break;
  case 20: // NUDGE POSITIVE - Get away from HomeSw
    // Record initial encoder position.
    // Then nudge motor in pos dir

    if (homingFirstScan)
    {
      nudgeRetryCnt++;
      initialPositionReading = motionData.actualPosition;
    }
    else if (stepTime > HOMING_NUDGE_TIME_MS)
    {
      speedRefPerc = 0.0;
      homingStep = 21;
    }
    else
    {
      speedRefPerc = NUDGE_POWER_PERC;
    }
    break;
  case 21: // Check if nudge moved less than 5 degrees
    if (abs(initialPositionReading - motionData.actualPosition) < 5.0)
    {
      // jump to final steps
      speedRefPerc = 0.0;
      homingStep = 50;
    }
    else
    {
      if (inputs.HomeSw)
      {
        nudgeRetryCnt = 0;
        homingStep = 30;
      }
      else if (nudgeRetryCnt < NUDGE_MAX_RETRIES)
      {
        homingStep = 20;
      }
      else // retry count exceeded
      {
        homingStep = 911;
      }
    }
    break;

  case 30: // NUDGE NEGATIVE
    // Record initial encoder position.
    // Then nudge motor in pos dir
    // setDirPinNegative();

    if (homingFirstScan)
    {
      nudgeRetryCnt++;
      initialPositionReading = motionData.actualPosition;
    }
    else if (stepTime > HOMING_NUDGE_TIME_MS)
    {
      speedRefPerc = 0.0;
      homingStep = 31;
    }
    else
    {
      speedRefPerc = -NUDGE_POWER_PERC;
    }
    break;

  case 31: // Check if nudge moved
    if (abs(initialPositionReading - motionData.actualPosition) < 5.0)
    {
      // jump to final steps
      speedRefPerc = 0.0;
      homingStep = 50;
    }
    else
    {
      if (nudgeRetryCnt < NUDGE_MAX_RETRIES)
      {
        homingStep = 30;
      }
      else
      {
        triggerError(Errors::HOMING_NUDGE_RETRIES_EXCEEDED);
        homingStep = 911;
      }
    }
    break;

  case 40: // Zero the encoder
    if (homingFirstScan)
    {
      turnAllOff();
      tracker.zeroPosition(encoder.position);
    }
    else
    {
      homingStep = 50;
    }
    break;

  case 50: // Check homeSw Status
    if (!inputs.HomeSw)
    {
      homingStep = 80;
    }
    else
    {
      homingStep = 60;
    }
    break;
  case 60: // down shift
    if (homingFirstScan)
    {
      shiftData.targetPosition = motionData.actualPosition - 360.0;
      activateController();
      // initialize the runController, signal downshift request
    }
    else if (checkShiftCompleted()) // TODO: switch logic back to just use atTargetAndStill
    {
      // printMotionData();
      turnOffController();
      homingStep = 50;
    }

    if (errors.present)
    {
      triggerError(Errors::CONTROLLER_FAULT_DURING_HOMING);
      printMotionData();
      turnOffController();
      homingStep = 911;
    }
    break;
  case 80: // zero and set isHomed
    if (homingFirstScan)
    {
      tracker.zeroPosition(encoder.position);
      isHomed = true;
      homingStep = 100;
    }
    break;
  case 100: // DONE
    if (homingFirstScan)
    {
      // DO NOTHING
    }
    break;
  case 911: // ERROR
    if (homingFirstScan)
    {
      speedRefPerc = 0.0;
      solPwr = 0.0;
      turnAllOff();
      // DO NOTHING
    }
    break;
  }
  if (!controllerOn)
  {
    outputs.MotorSpeed = speedRefPerc;
    outputs.SolPwr = solPwr;
    speedRefPerc = 0.0;
    solPwr = 0.0;
  }
  return homingStep;
}
void clearErrors()
{
  errors.present = false;
  for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  {
    errors.list[i] = Errors::NONE_ERROR;
  }
}
void triggerError(Errors code)
{
  bool isNewCode = true;
  errors.present = true;
  for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  {
    if (errors.list[i] == code)
    {
      isNewCode = false;
    }
    if (errors.list[i] == Errors::NONE_ERROR && isNewCode)
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
  // sending uint8_t vals
  uint8_t *q = (uint8_t *)data;
  *q = static_cast<uint8_t>(msgPacket->present);
  q++;
  for (int i = 0; i < FAULT_DATA_LIST_LENGTH; i++)
  {
    *q = msgPacket->list[i];
    q++;
  }
}

void runHomingRoutineStepper()
{
  static int homingStartDir = 0; //1: positive, -1: negative
  switch (iSerial.status.step)
  {
  case 0: // START HOMING
      iSerial.debugPrintln("HOMING STARTED: press both shifters switches at the same time to proceed");
      
      setTargetGear(motionData.actualGear);
      sendShiftData();
      turnAllOff();
      isHomed = false;
      iSerial.status.step = 10;
      break;

  case 10: // WAIT FOR DOWN SHIFT SW
      if (iShiftDownSw && iShiftUpSw)
      {
          enableMotor();
          if (iCamSw){
              iSerial.debugPrintln("Cam switch detected at homing start: moving to fine adjust");
              iSerial.status.step = 30;
          }
          else if (iPosLimSw)
          {
              iSerial.debugPrintln("Pos Lim Switch detected at homing start: moving backwards to find cam switch");
              homingStartDir = -1;
              iSerial.status.step = 20;
          }
          else
          {
              iSerial.debugPrintln("Pos Lim Switch not detected at homing start: moving forwards to find cam switch");
              homingStartDir = 1;
              iSerial.status.step = 20;
          }

      }
      break;

  case 20: // MOVING AN INDEX UNTIL CAM SWITCH TURNS ON
      // Serial.println("Homing: step 5");
      if (stepper.distanceToGo() == 0)
      {
          stepper.setMaxSpeed(MAX_VELOCITY / 3);
          stepper.move(homingStartDir*STEPS_PER_INDEX*1.1);
          iSerial.status.step = 21;
      }
      break;
  case 21:
      if (iCamSw)
      {
          iSerial.debugPrintln("Cam switch detected: stopping.");
          stepper.stop();
          time_start_ms = millis();
          iSerial.status.step = 22;
      }
      else if (stepper.distanceToGo() == 0)
      {
          iSerial.debugPrintln("ERROR! Expected the Cam switch to turn on before move finished");
          iSerial.status.step = 911;
      }
      break;
  case 22: // WAITING FOR STOP, WAITING ON CAM SWITCH AND THEN DECIDING
      if (stepper.distanceToGo() == 0 && millis() - time_start_ms > 500)
      {
          if(iCamSw){
              iSerial.debugPrintln("Cam switch is still on after the stop: now performing fine adjust");
              stepper.setCurrentPosition(0);
              iSerial.status.step = 30;
          }
          else{
              iSerial.debugPrintln("Cam switch turned off while stopping: reversing direction find it again ");
              homingStartDir*= -1;
              iSerial.status.step = 20;
          }
      }
      break;
  case 30: // HOME FINE ADJUST: moving forwards until cam switch turns off
      if (stepper.distanceToGo() == 0)
      {
          stepper.setMaxSpeed(MAX_VELOCITY / 10);
          stepper.move(STEPS_PER_INDEX / 20);
          iSerial.status.step = 31;
      }
      break;
  case 31: // HOME FINE ADJUST: waiting for cam switch to turn off then stopping
      if(!iCamSw){
          iSerial.debugPrintln("Cam switch off: stopping then moving back");
          stepper.stop();
          time_start_ms = millis();
          iSerial.status.step = 32;
      }
      else if (stepper.distanceToGo() == 0)
      {
          iSerial.debugPrintln("ERROR! Expected the Cam switch to turn off before move finished");
          iSerial.status.step = 911;
      }
      break;
  case 32: //HOME FINE ADJUST: moving backwards to center cam magnet with cam switch
      if (stepper.distanceToGo() == 0 && millis() - time_start_ms > 500)
      {
          stepper.setMaxSpeed(MAX_VELOCITY / 10);
          stepper.move(-STEPS_PER_INDEX / 50); //JOE NOTE: this value may need to be adjusted with each iteration of rig
          iSerial.status.step = 33;
      }
      break;
  case 33: //VERIFY CAM SWITCH IS ON
      if (stepper.distanceToGo() == 0){
          if(iCamSw){
              iSerial.status.step = 50;
          }
          else 
          {
              iSerial.debugPrintln("ERROR! Expected the Cam switch to turn on after cam sw centering move finished");
              iSerial.status.step = 911;
          }
      }
      break;
  case 50: // CHECKING POS LIM SW: INDEXING FORWARD IF NOT AT POS LIM SWITCH
      if (iPosLimSw)
      {
          iSerial.debugPrintln("Pos Lim switch detected!: setting position");
          iSerial.status.step = 80;
      }
      else {
          updateGearNumber();
          if(currentGear < NUM_GEARS){
              stepper.setMaxSpeed(MAX_VELOCITY);
              stepper.move(STEPS_PER_INDEX);
              iSerial.status.step = 51;
          } else {
              iSerial.debugPrintln("ERROR! Pos Lim switch not after 12 indexes: positive lim switch is not working");
              iSerial.status.step = 911;
          
          }
      }
      break;
  case 51: // WAITING FOR INDEX TO FINISH

      if (stepper.distanceToGo() == 0)
      {
          iSerial.status.step = 50;
      }
      break;
  case 80: // SET CURRENT GEAR AND TARGET GEAR NUMBER
      stepper.setCurrentPosition(STEPS_PER_INDEX * NUM_GEARS);
      targetGear = NUM_GEARS;
      updateGearNumber();
      iSerial.debugPrintln("HOMING IS FINISHED! Release the down shift switch to continue");
      iSerial.status.step = 90;
      break;
  case 90:
      if (!iShiftDownSw)
      {
          stepper.setMaxSpeed(MAX_VELOCITY);
          iSerial.status.step = 100;
      }
      break;

  case 100: // DONE
      // stepper.setMaxSpeed(MAX_VELOCITY);
      // isHomed = true;
      break;

  case 110: // DEPRECATED: CALIBRATE HOME
      int dist = 2;
      if (iShiftDownSw)
      {
          stepper.move(-dist);
          iSerial.status.step = 111;
      }
      else if (iShiftUpSw)
      {
          stepper.move(dist);
          iSerial.status.step = 111;
      }
      break;
  case 111:
      if (stepper.distanceToGo() == 0)
      {
          iSerial.status.step = 110;
          iSerial.debugPrint("Current Position: ");
          iSerial.debugPrintln(String(stepper.currentPosition()));
          // textDisplay(String(stepper.currentPosition()).c_str());
      }
      break;

  case 911: // ERROR
      disableMotor();
      // isHomed = false;
      break;
  default:
      break;
  }

  if (iSerial.status.step == 100)
  {
      iSerial.setNewMode(Dimitri::Modes::IDLE);
  }
  else if (iSerial.status.step == 911)
  {
      errorNumber = Dimitri::Errors::HOMING_FAILED;
      iSerial.debugPrintln("ERROR! Homing failed, aborting");
      iSerial.setNewMode(Dimitri::Modes::ABORTING);
  }
}