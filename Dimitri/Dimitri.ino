bool SHIFT_BUTTONS_DISABLED = false; // if this is true, the shift buttons will be disabled (ignored)
bool START_IN_MANUAL= true;
const bool SIM_MODE = false;

#define SCAN_TIME_US 5000  //how frequently the loop updates
#define UPDATE_TIME_US 1000 //time that the motor velocities are updated

#include "Arduino.h"
#include "ISerial.h"
#include "Encoder.h"
#include "Shifter.h"
#include "CustomDataTypes.h"
#include "Motor.h"
#include "TimerOne.h"


////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////

// TOGLGLE MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the engaged direction
#define PIN_TOGGLE_DIR 12   
#define PIN_TOGGLE_PWM 3
#define PIN_TOGGLE_CURRENT A0     
   
// LINEAR MOTOR - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
// when motor has positive power, it moves the motor in the down shift direction
#define PIN_LINEAR_PWM 11
#define PIN_LINEAR_DIR 13     
#define PIN_LINEAR_CURRENT A1   
  
#define PIN_LINEAR_ENC_A 18   //
#define PIN_LINEAR_ENC_B 19   //
#define PIN_TOGGLE_ENC_A 20   //
#define PIN_TOGGLE_ENC_B 21   //
#define PIN_EINK_BIT0 22      // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER DISPLAY ON E-INK
#define NUM_BITS 4
#define PIN_LINEAR_POS_LIM 41 
#define PIN_TOGGLE_NEG_LIM 43          
#define PIN_SHIFT_UP 51       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 53     // RED WIRE ON SHIFTER

struct Inputs
{
  bool ShiftDownSw = false;
  bool ShiftUpSw = false;
  bool ToggleNegLimSw = false;
  bool LinearPosLimSw = false;
  uint16_t ToggleCurrent = 0;
  uint16_t LinearCurrent = 0;
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


#define NUM_MOTORS 2

enum Motors {
  TOGGLE=0,
  LINEAR=1,
};

// ENCODERS
Encoder encoders[NUM_MOTORS] = {
  Encoder(PIN_TOGGLE_ENC_A, PIN_TOGGLE_ENC_B),
  Encoder(PIN_LINEAR_ENC_A, PIN_LINEAR_ENC_B) //14PPR Of the motor and gear ratio of 1:50 and 1/2 rev per index, but empiracally found 767 counts per index
};

Motor::Cfg motorCfgs[NUM_MOTORS] = {
  //name, homeDir, homeType, unit, pulsesPerUnit, maxVelocity, softLimitPositive, softLimitNegative, invertDir, positionTol, zeroVelocityTol, kP, kD, nudgeTimeMs

  {"toggle",-1, 2, "deg", 900.0, 1000.0, 180.0, 0.0, true, 5.0, 5.0, 1.0, 0.0, 0, 100.0}, // TOGGLE name, homeDir, homeType, unit, pulsesPerUnit, maxVelocity, softLimitPositive, softLimitNegative, invertDir, positionTol, zeroVelocityTol, kP, kD
  {"linear",1, 2, "gear", 767.0, 20.0, 12.0, 1.0, true, 0.015, 0.1, 500.0, 15.0, 15, 100.0} // LINEAR
};


Motor motors[NUM_MOTORS] = {
  Motor(PIN_TOGGLE_DIR, PIN_TOGGLE_PWM, &encoders[Motors::TOGGLE], &motorCfgs[Motors::TOGGLE],SIM_MODE),
  Motor(PIN_LINEAR_DIR, PIN_LINEAR_PWM, &encoders[Motors::LINEAR], &motorCfgs[Motors::LINEAR],SIM_MODE)
};


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
ShiftData shiftData;

//PDController controller = PDController(1.20, 0.000); // NOTE: modify these parameters to improve the control, start with pd set to zero

#define NUM_GEARS 12
#define MIN_POSITION 0.0
#define COUNTS_PER_GEAR 750.0
#define MAX_POSITION (MIN_POSITION + float(NUM_GEARS - 1) * COUNTS_PER_GEAR)

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

// LOOP SCAN RATE
unsigned long lastUpdateUs = 0;

void setup()
{
  iSerial.init();
  iSerial.THIS_DEVICE_ID = BIKE_MEGA_ID;
  iSerial.setAutoSendStatus(false); // status updates sent when mode changes
  iSerial.debug = false;

  iSerial.setNewMode(Modes::ABORTING);
  
  pinMode(PIN_TOGGLE_CURRENT, INPUT);
  pinMode(PIN_LINEAR_CURRENT, INPUT);
  pinMode(PIN_LINEAR_POS_LIM, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_NEG_LIM, INPUT_PULLUP);
  pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
  pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_DIR, OUTPUT);
  pinMode(PIN_TOGGLE_PWM, OUTPUT);
  pinMode(PIN_LINEAR_PWM, OUTPUT);
  pinMode(PIN_LINEAR_DIR, OUTPUT);

  //motors[Motors::TOGGLE].setDebug(true);
  motors[Motors::LINEAR].setDebug(false);

  lastUpdateUs = micros();

  Timer1.initialize(UPDATE_TIME_US); // Initialize timer to trigger every 1000 microseconds
  Timer1.attachInterrupt(updateMotors);

}

// int deleteMe = 0;
// int deleteMeSign = 1;
StopWatch stopWatch;
unsigned long timeNow = millis();


int tempInt = 0;

void loop()
{
  unsigned long timeNowUs = micros();
  if (timeNowUs - lastUpdateUs > SCAN_TIME_US)
  {
    if(timeNowUs - lastUpdateUs > 1.10*SCAN_TIME_US){
      iSerial.debugPrintln("WARNING: long scan time detected: " + String(timeNowUs - lastUpdateUs) + "usec");
    }
    lastUpdateUs = timeNowUs;

    //used for plotting velocity
    if(true){
      Serial.print(motors[Motors::LINEAR].actualPosition);
      Serial.print(", ");
      Serial.println(String(motors[Motors::LINEAR].atPosition));
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].run();
    }
    readInputs();
    timeNow = millis();
  
    if (timeNow - lastDisplayed_ms >= 500)
    {
      lastDisplayed_ms = timeNow;
      
      //iSerial.debugPrint("toggle motor encoder pulses:");
      //iSerial.debugPrintln(String(motors[Motors::TOGGLE].actualEncoderPulses));
      //iSerial.debugPrint("Toggle Neg Lim Sw: ");
      //iSerial.debugPrintln(String(inputs.ToggleNegLimSw));

      if (false)
      {
        iSerial.debugPrint("TOGGLE MOTOR Actual Position: ");
        iSerial.debugPrintln(String(encoders[Motors::TOGGLE].read()));
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
          iSerial.status.step = 1;
        }
        else if (iSerial.status.step == 1)
        {
          // auto clear the errors after 5 seconds
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
        if (START_IN_MANUAL)
        {
          //motors[Motors::TOGGLE].enable();
          iSerial.setNewMode(Modes::MANUAL);

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
          //runHomingRoutine(); // this resets the homing routine
          iSerial.status.step = 1;
        }

        break;

      case Modes::IDLE:
        if (iSerial.status.step == 0)
        {
          iSerial.status.step = 1;
        }

        if (errors.present)
        {
          iSerial.setNewMode(Modes::ABORTING);
        }
        else if (gearChangeReq)
        {
          iSerial.setNewMode(Modes::SHIFTING);
        }

        break;
      case Modes::SHIFTING:
        break;

      case Modes::MANUAL:

        if (iSerial.status.step == 0)
        {
          motors[Motors::TOGGLE].enable();
          if(motors[Motors::TOGGLE].getState() == Motor::States::IDLE){
            iSerial.debugPrintln("HOMING - Moving toggle motor to Engaged Position");
            iSerial.resetModeTime();
            iSerial.status.step = 5;
          }
        }
        else if (iSerial.status.step == 5){
          motors[Motors::TOGGLE].jogUsingPower(-100.0);
          //digitalWrite(PIN_TOGGLE_DIR, motorCfgs[Motors::TOGGLE].invertDir);
          //analogWrite(PIN_TOGGLE_PWM, 255);

          if (inputs.ToggleNegLimSw || iSerial.modeTime() > 1500){ 
            //motors[Motors::TOGGLE].zero();
            //analogWrite(PIN_TOGGLE_PWM, 0);
            motors[Motors::TOGGLE].stop();
            iSerial.resetModeTime();
            iSerial.debugPrintln("HOMING - Moving toggle motor to Disengaged Position");
            iSerial.status.step = 6;
          } else if (iSerial.modeTime() > 2000){
            iSerial.debugPrintln("HOMING - Error finding Toggle Neg Lim Sw");
            iSerial.status.step = 911; //error
          }
        }
        else if (iSerial.status.step == 6){
          
          if(iSerial.modeTime() > 1000){

            iSerial.debugPrint("TOGGLE MOTOR Actual Position Before zero(): ");
            iSerial.debugPrintln(String(encoders[Motors::TOGGLE].read()));
            motors[Motors::TOGGLE].zero();
            iSerial.debugPrint("TOGGLE MOTOR Actual Position After zero(): ");
            iSerial.debugPrintln(String(encoders[Motors::TOGGLE].read()));
            iSerial.resetModeTime();
            iSerial.status.step = 10;
          }
          else if (iSerial.modeTime() > 200){
            motors[Motors::TOGGLE].stop();
          }
        }
        else if (iSerial.status.step == 10){
          motors[Motors::TOGGLE].jogUsingPower(100.0);
          if(encoders[Motors::TOGGLE].read() > 700){
            iSerial.resetModeTime();
            iSerial.status.step = 20;
          }else if (iSerial.modeTime() > 2000){
            iSerial.debugPrintln("TOGGLE MOTOR Error While Moving to Disengaged Position");
            iSerial.debugPrint("TOGGLE MOTOR Actual Position: ");
            iSerial.debugPrintln(String(encoders[Motors::TOGGLE].read()));
            iSerial.status.step = 911; //error
          }
        }
        else if (iSerial.status.step == 20){
          //analogWrite(PIN_TOGGLE_PWM, 0);
          motors[Motors::TOGGLE].stop();
          motors[Motors::LINEAR].enable();
          if(motors[Motors::LINEAR].getState() == Motor::States::IDLE && motors[Motors::TOGGLE].getState() == Motor::States::IDLE){
            iSerial.debugPrintln("HOMING - Moving linear motor to positive lim sw");
            iSerial.resetModeTime();
            iSerial.status.step = 21;
          }
        }
        else if (iSerial.status.step == 21){
          motors[Motors::LINEAR].jogUsingPower(100.0);
          if(inputs.LinearPosLimSw){
            motors[Motors::LINEAR].stop();
            iSerial.debugPrintln("HOMING - LINEAR MOTOR found limit switch");
            iSerial.resetModeTime();
            iSerial.status.step = 25;
          } else if (iSerial.modeTime() > 1500){
            iSerial.debugPrintln("ERROR: LINEAR MOTOR timed out while trying to reach limit switch");
            iSerial.debugPrint("LINEAR MOTOR Actual Position: ");
            iSerial.debugPrintln(String(encoders[Motors::LINEAR].read()));
            //iSerial.resetModeTime();
            iSerial.status.step = 911;
          }
        }
        else if (iSerial.status.step == 25){
          
          if(iSerial.modeTime() > 500){
            //encoders[Motors::LINEAR].write(0);
            iSerial.debugPrint("LINEAR MOTOR Actual Position before zero(): ");
            iSerial.debugPrintln(String(encoders[Motors::LINEAR].read()));
            motors[Motors::LINEAR].zero();
            tempInt = 0;
            iSerial.debugPrint("LINEAR MOTOR Actual Position after zero(): ");
            iSerial.debugPrintln(String(encoders[Motors::LINEAR].read()));
            iSerial.resetModeTime();
            iSerial.status.step = 26;
          }
        }
        else if (iSerial.status.step == 26){
          if(iSerial.modeTime() > 25){
            motors[Motors::LINEAR].stop();
            iSerial.status.step = 27;
          } else {
            motors[Motors::LINEAR].jogUsingPower(-100.0);
          }
        }
        else if (iSerial.status.step == 27){
          if(!inputs.LinearPosLimSw){
            iSerial.status.step = 28;
          } else if(tempInt > 7){
            iSerial.debugPrintln("ERROR: LINEAR MOTOR exceeded move count");
            iSerial.debugPrint("LINEAR MOTOR Actual Position: ");
            iSerial.debugPrintln(String(encoders[Motors::LINEAR].read()));
            iSerial.status.step = 911;
          } else {
            tempInt++;
            iSerial.resetModeTime();
            iSerial.status.step = 26;
          }
        }
        else if (iSerial.status.step == 28){
          motors[Motors::LINEAR].zero();
          iSerial.status.step = 30;
          //iSerial.debugPrint("LINEAR MOTOR Actual Position: ");
          //iSerial.debugPrintln(String(encoders[Motors::LINEAR].read()));
          
        }
        else if (iSerial.status.step == 30){
          //analogWrite(PIN_TOGGLE_PWM, 0);
          motors[Motors::LINEAR].moveAbs(-0.03);
          if(motors[Motors::LINEAR].getState() != Motor::States::IDLE){
            iSerial.debugPrintln("HOMING - Moving linear motor to -0.05");
            iSerial.resetModeTime();
            iSerial.status.step = 31;
          }
    
        }
        else if (iSerial.status.step == 31){
          if(motors[Motors::LINEAR].getState() == Motor::States::IDLE){
            iSerial.debugPrintln("HOMING - setting linear motor position to be 12");
            motors[Motors::LINEAR].setPosition(12.0);
            iSerial.resetModeTime();
            iSerial.status.step = 90;
          }
        }
        else if (iSerial.status.step == 90){
          //iSerial.debugPrint("Linear Motor isStill: ");
          //iSerial.debugPrintln(String(motors[Motors::LINEAR].isStill));
          runLinearMotorManualMode();
          iSerial.debugPrint("LINEAR MOTOR Actual Position: ");
          iSerial.debugPrintln(String(motors[Motors::LINEAR].actualPosition));
        }
        else if (iSerial.status.step == 100){
          runToggleMotorManualMode();
        }
        else if (iSerial.status.step == 911){
          //analogWrite(PIN_TOGGLE_PWM, 0);
          motors[Motors::TOGGLE].disable();
          motors[Motors::LINEAR].disable();
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
      else if (iSerial.status.mode != Modes::IDLE && iSerial.status.mode != Modes::SHIFTING && iSerial.status.mode != Modes::MANUAL)
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

    //runAll(); // runs all things that have a run() method that need to be called each loop

    // RESET VALUES EACH LOOP - reset values at end of each loop, like OTEs and Events
    iSerial.event = 0;

    // DEBUG PASSTHROUGHS - comment out as needed
    // solenoids[0].setDebug(iSerial.debug);
  }
}

// checks the homeSw state and shiftType, prevents downshifting if homeSw detects target
bool checkShiftTypeAndHomeSw(int shiftTypeReq)
{
  if (inputs.LinearPosLimSw && (shiftTypeReq == Shifter::ShiftTypes::UP))
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
    //processAbsPosCmd();
    break;

  case Cmds::HOME_CMD:
    clearErrors();
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
  //digitalWrite(PIN_TOGGLE_PWM, 0);
  //digitalWrite(PIN_LINEAR_PWM, 0);
}

bool checkShiftCompleted()
{
  return atTarget && (atTargetAndStill || atTargetCnt > 7);
}

void readInputs()
{
  inputs.ShiftUpSw = !digitalRead(PIN_SHIFT_UP);
  inputs.ShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
  inputs.ToggleNegLimSw = !digitalRead(PIN_TOGGLE_NEG_LIM);
  inputs.LinearPosLimSw = !digitalRead(PIN_LINEAR_POS_LIM);
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
  iSerial.setIo(sensorNum, inputs.ToggleNegLimSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftUpSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.ShiftDownSw);

  sensorNum++;
  iSerial.setIo(sensorNum, inputs.LinearPosLimSw);

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
    //checkPositionAtTarget();
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


// USED FOR MANUAL MODE - jogs motor at 100% pwr with shift switches
void runToggleMotorManualMode(){
    if(inputs.ShiftUpSw){
        //analogWrite(PIN_TOGGLE_PWM, 255);
        //digitalWrite(PIN_TOGGLE_DIR, !motorCfgs[Motors::TOGGLE].invertDir);
        motors[Motors::TOGGLE].enable();
        motors[Motors::TOGGLE].jogUsingPower(100);
        //iSerial.debugPrintln("TOGGLE MOTOR jogging positive");
    } else if(inputs.ShiftDownSw){
        //analogWrite(PIN_TOGGLE_PWM, 255);
        //digitalWrite(PIN_TOGGLE_DIR, motorCfgs[Motors::TOGGLE].invertDir);
        motors[Motors::TOGGLE].enable();
        motors[Motors::TOGGLE].jogUsingPower(-100);
    } else {
        //analogWrite(PIN_TOGGLE_PWM, 0);
        //motors[Motors::TOGGLE].stop();
        motors[Motors::TOGGLE].disable();
    }
}

void runLinearMotorManualMode(){
    
    if(inputs.ShiftUpSw){
        //analogWrite(PIN_LINEAR_PWM, 255);
        //digitalWrite(PIN_LINEAR_DIR, !motorCfgs[Motors::LINEAR].invertDir);
        motors[Motors::LINEAR].enable();
        //motors[Motors::LINEAR].jogUsingPower(100);
        motors[Motors::LINEAR].moveAbs(11.0);
    } else if(inputs.ShiftDownSw){
        //analogWrite(PIN_LINEAR_PWM, 255);
        //digitalWrite(PIN_LINEAR_DIR, motorCfgs[Motors::LINEAR].invertDir);
        motors[Motors::LINEAR].enable();
        //motors[Motors::LINEAR].jogUsingPower(-100);
        motors[Motors::LINEAR].moveAbs(6.0);
    } else {
        //digitalWrite(PIN_LINEAR_PWM, LOW);
        //motors[Motors::LINEAR].stop();
        //motors[Motors::LINEAR].disable();
    }
}

void updateMotors()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].update();
  }
}