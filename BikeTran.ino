bool FLIP_POSITIVE = true; // This should normally be false, just used for testing directional differences in motor cmd

#include "ISerial.h"
#include "Encoder.h"
#include "PDController.h"
#include "EncoderTracker.h"
#include "Shifter.h"
#include "Solenoid.h"

// PWM SIGNAL TO MOTOR DRIVER
// 50Hz-20kHz, Amp 2.5V-5V

// MOTOR INFO
//  HALL SENSORS     DRIVER BOARD
//  RED: +5VDC       RED
//  BLK: GND         BLK
//  GRN: HALL A      GRN
//  BLU: HALL B      WHT
//  YEL: HALL C      YEL

// MOTOR PHASES
//  RED: mA
//  YEL: mB
//  BLK: mC

////////////////////////////////////////////////////
// INPUTS
////////////////////////////////////////////////////
#define PIN_HOME_SW 8
////////////////////////////////////////////////////
// OUTPUTS
////////////////////////////////////////////////////

// MOTOR CONTROL BOARD
#define PWM_PIN 11                 // Used for controlling the motor driver board
const int PWM_FREQUENCY_HZ = 2000; // Desired PWM frequency in Hz
#define DIR_PIN 3                  // LOW FOR POSITIVE, HIGH FOR NEGATIVE, output for an input to the motor driver board

// PWM DRIVERS FOR SOLENOID STOPPER
#define PIN_SOL_STOPPER 6

// ids, used for array so start with 0!
enum Solenoids : int
{
  STOPPER = 0,
};
#define NUM_SOLENOIDS 1
Solenoid solenoids[NUM_SOLENOIDS];

// ENCODER - note that the rs485 needs to be connected to the serial1 rx and tx pins
Encoder encoder;
EncoderTracker tracker;
unsigned long lastRead_us = 0;

unsigned long previousMillis = 0; // Initialize previousMillis to 0

// MOTION PROFILE
unsigned long startTime = 0; // The start time of the motion profile
float currentPosition = 0.0; // Current position in revolutions//
float targetPosition = 1.0;  // Target position in revolutions
float currentVelocity = 0.0; // Initialize the stored position
float maxVelocity = 1.0;     // Maximum velocity in revolutions per second
float acceleration = 0.1;    // Acceleration in revolutions per second squared

bool sw = false;
int time_now;
long last_time = millis();

// SHIFTER
#define PIN_SHIFT_UP 15
#define PIN_SHIFT_DOWN 16
Shifter shifter(PIN_SHIFT_UP, PIN_SHIFT_DOWN);

// MOTOR CONTROL LAW CONTROLLER
const double MAX_OUTPUT = 60.0;
PDController controller = PDController(1.2, 0.2500); // NOTE: modify these parameters to improve the control, start with pd set to zero
#define NUM_GEARS 12
#define MIN_POSITION 0.0
#define MAX_POSITION (MIN_POSITION + float(NUM_GEARS - 1) * 360.0)
int targetGear = 1;
int actualGear = 1;
bool controllerOn = false;                // set this true to activate outputs related to the controller, set to false to kill those outputs
unsigned long activationStartTime_ms = 0; // the time the controller first became activated
const int SOLENOID_RETRACT_TIME_MS = 100;
const int CONTROLLER_TIME_LIMIT_MS = 1200; // max time the controller is allowed to be active
int controllerError = 0;                   // 1: timed out

ISerial iSerial;
long tempLong;
float tempFloat;

// DIAGNOSTICS
unsigned long lastDisplayed_ms = 0;

class RadGear
{
public:
  enum Modes : int32_t //
  {
    ABORTING = -3,
    KILLED = -2,
    ERROR = -1,
    INACTIVE = 0,
    RESETTING = 50,
    IDLE = 100,
    SHIFTING = 200, // while shifting, the controller is active
    // SUPER_SHIFT_UP = 300,
    // SHIFT_DOWN = 400,
    // SUPER_SHIFT_DOWN = 500,
  };

  enum Events : int
  {
    NONE = 0,
    UP_SHIFT_REQ = 1,
    SUPER_UP_SHIFT_REQ = 2,
    DOWN_SHIFT_REQ = 3,
    SUPER_DOWN_SHIFT_REQ = 4,
  };
};

void setup()
{
  iSerial.init();

  // initialize solenoid stopper
  pinMode(PIN_SOL_STOPPER, OUTPUT);
  // solenoids[Solenoids::STOPPER].init(PIN_SOL_STOPPER, SolenoidTypes::PWM, SolenoidSafety::CONTINUOUS_OK);

  // initialize encoder
  encoder.init();

  // initialize shifter
  pinMode(PIN_HOME_SW, INPUT);

  // initialize motor controller outputs
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);          //
  setupPWM(PWM_PIN, PWM_FREQUENCY_HZ); // initialize motor control pin

  initializeEncoderSystem();
  iSerial.debug = true;
  iSerial.setNewMode(RadGear::Modes::ABORTING);
}

void loop()
{
  // updateIo();
  if (millis() - 250 >= lastDisplayed_ms)
  {
    lastDisplayed_ms = millis();
    printCurrentPosition();
  }

  /*
    if (iSerial.taskProcessUserInput())
    {
      handleSerialCmds();
    }
  */

  bool gearChangeReq = false;
  int shiftTypeReq = checkForGearShift();
  bool atTarget = checkPosition();
  if (shiftTypeReq > 0)
  {
    if (iSerial.status.mode == RadGear::Modes::IDLE || iSerial.status.mode == RadGear::Modes::SHIFTING)
    {
      gearChangeReq = processShiftReqNum(shiftTypeReq);

      if (shiftTypeReq > 0 && gearChangeReq)
      {
        atTarget = false;
        Serial.print("shiftType: ");
        Serial.println(shiftTypeReq);
      }
    }
  }

  switch (iSerial.status.mode)
  {
  case RadGear::Modes::ABORTING:
    turnAllOff();
    iSerial.setNewMode(RadGear::Modes::KILLED); // TODO: this may break the raspi code, may want raspi to initiate the mode change
    break;
  case RadGear::Modes::KILLED:
    turnAllOff();
    printCurrentPosition();
    // solenoids[Solenoids::STOPPER].changeMode(Solenoid::ON, 255);
    iSerial.setNewMode(RadGear::Modes::INACTIVE);
    break;
  case RadGear::Modes::ERROR:
    turnAllOff();
    iSerial.setNewMode(RadGear::Modes::INACTIVE);
    break;
  case RadGear::Modes::INACTIVE:
    turnAllOff();
    iSerial.setNewMode(RadGear::Modes::RESETTING);
    break;

  case RadGear::Modes::RESETTING:
    // TODO: insert homing logic here, home switch should be wired n.c.

    // 1. Check home sw, if ON go to 2. else go to 1B

    // 1B. Record initial encoder position. Rotate motor slowly in pos dir, if encoder position doesn't change, jump to end, otherwise wait for home sw to turn off.

    // 1C. Stop motor, go to step 4

    // 2. Retract solenoid, then hold solenoid retracted low power

    // 3. Rotate motor in slow speed in neg dir (constant speed ref output from pwm pin) until home switch turns off (actuated), record encoder position and release solenoid

    // 4. Reduce motor speed to turtle speed and keep turning in neg dir until encoder value stops changing (go to next step), other wise if encoder value is more than 1 rev from recorded value in prev step, then trigger error

    // 5. Stop motor and zero the encoder
    digitalWrite(PWM_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    delay(200);

    analogWrite(PIN_SOL_STOPPER, 254);

    delay(100);
    analogWrite(PIN_SOL_STOPPER, 229);
    analogWrite(PWM_PIN, 255);
    delay(70);
    digitalWrite(PIN_SOL_STOPPER, LOW);
    analogWrite(PWM_PIN, 75);
    delay(600);
    digitalWrite(PWM_PIN, LOW);
    // delay(1000);
    digitalWrite(PIN_SOL_STOPPER, LOW);
    iSerial.setNewMode(RadGear::Modes::IDLE);
    targetGear = actualGear;
    break;

  case RadGear::Modes::IDLE:
    if (iSerial.status.step == 0)
    {
      turnAllOff();
      iSerial.status.step = 1;
    }
    if (gearChangeReq)
    {
      iSerial.setNewMode(RadGear::Modes::SHIFTING);
    }

    break;
  case RadGear::Modes::SHIFTING:
    if (iSerial.status.step == 0)
    {
      activateController();
      iSerial.status.step = 1;
    }
    else if (iSerial.status.step == 1)
    {
      if (gearChangeReq)
      {
        activateController(); // this will restart the activation time
      }
      else if (atTarget)
      {
        printCurrentPosition();
        iSerial.setNewMode(RadGear::Modes::IDLE);
      }

      if (controllerError > 0)
      {
        printCurrentPosition();
        turnOffController();
        iSerial.setNewMode(RadGear::Modes::ERROR);
      }
    }

    break;

  default:
    break;
  }

  runAll(); // runs all things that have a run() method that need to be called each loop

  // RESET VALUES EACH LOOP - reset values at end of each loop, like OTEs and Events
  iSerial.event = 0;

  // DEBUG PASSTHROUGHS - comment out as needed
  // solenoids[0].setDebug(iSerial.debug);
}

// handles serial cmds that aren't already handled by ISerial (connect, mode, debug, maybe more?)
void handleSerialCmds()
{
  int idx = iSerial.idChr - '0';

  switch (iSerial.cmdChr)
  {
  case Cmds::ABSPOS_CMD:
    // processAbsPosCmd();
    break;

  case Cmds::SERVOPOSINFO_CMD:
    // processServoPosInfoCmd();
    break;

  case Cmds::SERIAL_OUTPUT: // prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
    iSerial.writeCmdChrIdChr();
    iSerial.writeNewline();
    Serial.print("iSerial.status.mode: ");
    Serial.println(iSerial.status.mode);
    // Serial.print("Servo Mode: ");
    // Serial.println(servos[idx].mode);
    // Serial.print("ActualPosition_pulse: ");
    // Serial.println(servos[idx].pos);
    //  Serial.print("RefPosition: ");
    //  Serial.println(servos[idx].referencePosition);
    //  Serial.print("ActualVelocity: ");
    //  Serial.println(ts[idx].iStepper.status.actualVelocity);
    break;

  case Cmds::PARAMS_SET: // set params
    // processParamsCmd();
    break;

  default:
    processUnrecognizedCmd();
    break;
  }
}

// returns true if current gear is at target position
bool checkPosition()
{
  actualGear = round(currentPosition / 360.0) + 1;

  if (abs(targetPosition - currentPosition) < 10.0)
  {
    actualGear = targetGear;
    return true;
  }
  else
  {
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
  turnAllSolenoidsOff();
  turnOffController();
}

void turnOffController()
{
  controllerOn = false;
  controllerError = 0;
  digitalWrite(PWM_PIN, LOW);
}

void activateController() // just call this once to activate
{
  activationStartTime_ms = millis();
  controllerOn = true;
}

void runAll()
{
  bool attemptedReading = encoder.run();
  if (encoder.newReadingFlag)
  {
    currentPosition = tracker.calculateDegrees(encoder.position);
  }
  if (attemptedReading)
  {
    runController();
  }
  // runAllSolenoids();
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
  /*
  int sensorNum = 0;
  iSerial.setIo(sensorNum, readPiInput(PIN_PI_TELEPORTER_INPUT));

  sensorNum++;
  iSerial.setIo(sensorNum, readPiInput(PIN_PI_LOOP_INPUT));

  sensorNum++;
  iSerial.setIo(sensorNum, readPiInput(PIN_PI_SPACE_INV_INPUT));

  sensorNum++;
  iSerial.setIo(sensorNum, readPiInput(PIN_PI_FLIPPER_INPUT));
  */
}

// returns true if the target changed from previous
bool setTargetGear(int targetGearNum)
{
  int prevTargetGear = targetGear;
  if (targetGearNum > NUM_GEARS)
  {
    targetGear = NUM_GEARS;
  }
  else if (targetGearNum < 1)
  {
    targetGear = 1;
  }
  else
  {
    targetGear = targetGearNum;
  }

  targetPosition = 360.0 * (targetGear - 1);
  Serial.print("Target Gear: ");
  Serial.println(targetGear);
  Serial.print("Target Position: ");
  Serial.println(targetPosition);
  Serial.print("Actual Gear: ");
  Serial.println(actualGear);
  Serial.print("Actual Position: ");
  Serial.println(currentPosition);
  return (targetGear != prevTargetGear);
}

int getRandomNumber(int min, int max) // get reandom
{
  return random(min, max + 1);
}

// collects a few encoder values and then zeroes the initial position, make sure the system is powered up with the motor in locked position
void initializeEncoderSystem()
{
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
    currentPosition = tracker.calculateDegrees(encoder.position);
    // Serial.println(currentPosition);
    tracker.zeroPosition();
    delay(200);
  }

  setTargetGear(1);
}

void turnAllSolenoidsOff()
{
  for (int i = 0; i < NUM_SOLENOIDS; i++)
  {
    solenoids[i].changeMode(Solenoid::Modes::OFF);
    solenoids[i].run();
  }
}

void runAllSolenoids()
{
  for (int i = 0; i < NUM_SOLENOIDS; i++)
  {
    solenoids[i].run();
  }
}

void runController()
{
  static bool showedWarning = false;
  unsigned long activeTime_ms = millis() - activationStartTime_ms;
  // controller.calculateSpeedControl(currentPosition, targetPosition); // always allow this to run so that previous values stay valid
  if (controllerOn && activeTime_ms < CONTROLLER_TIME_LIMIT_MS)
  {
    // Release solenoid initially with full power, than reduce to lower power as long as the absolute target error is greater than 180deg, otherwise turn it off
    if (activeTime_ms < 2 * SOLENOID_RETRACT_TIME_MS)
    {
      showedWarning = false;
      // solenoids[Solenoids::STOPPER].changeMode(Solenoid::ON, 255);
      analogWrite(PIN_SOL_STOPPER, 255);
    }
    else if (abs(controller.error) > 180.0)
    {
      // solenoids[Solenoids::STOPPER].changeMode(Solenoid::ON, 75);
      analogWrite(PIN_SOL_STOPPER, 144);
    }
    else
    {
      // solenoids[Solenoids::STOPPER].changeMode(Solenoid::OFF);
      digitalWrite(PIN_SOL_STOPPER, LOW);
    }

    // SET THE DIRECTION PIN
    if (controller.speedControl < 0.0 && !FLIP_POSITIVE)
    {
      digitalWrite(DIR_PIN, HIGH);
      // controller.speedControl = 0.0;
    }
    else if (controller.speedControl > 0.0 && FLIP_POSITIVE)
    {
      digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
      digitalWrite(DIR_PIN, LOW);
    }

    // SET THE PWM OUTPUT TO THE MOTOR BOARD IF ENOUGH TIME HAS BEEN ALLOTTED TO ALLOW SOLENOID TO RETRACT
    if (activeTime_ms >= SOLENOID_RETRACT_TIME_MS)
    {
      analogWrite(PWM_PIN, MAX_OUTPUT * controller.speedControl / 100.0);
    }
    else
    {
      digitalWrite(PWM_PIN, LOW);
    }
  }
  else
  {
    digitalWrite(PWM_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(PIN_SOL_STOPPER, LOW);
    // solenoids[Solenoids::STOPPER].changeMode(Solenoid::OFF);
    if (controllerOn && activeTime_ms >= CONTROLLER_TIME_LIMIT_MS && !showedWarning)
    {
      controllerError = 1;
      showedWarning = true;
      Serial.print("ERROR: Controller timed out after ");
      Serial.print(CONTROLLER_TIME_LIMIT_MS);
      Serial.println(" ms");
    }
  }
}

int checkForGearShift()
{
  int shiftReqType = shifter.checkShiftReq();
  if (shiftReqType == 0)
  {
    shiftReqType = checkForGearShiftSerial();
  }
  return shiftReqType;
}

// updates the target gear and corresponding target position
bool processShiftReqNum(int shiftType)
{
  bool targetChanged = false;
  switch (shiftType)
  {
  case Shifter::NONE:
    break;
  case Shifter::SUPER_UP:
    Serial.println("SUPER UP SHIFT");
    targetChanged = setTargetGear(NUM_GEARS);
    break;
  case Shifter::UP:
    Serial.println("UP SHIFT");
    targetChanged = setTargetGear(targetGear + 1);
    break;
  case Shifter::SUPER_DOWN:
    Serial.println("SUPER DOWN SHIFT");
    targetChanged = setTargetGear(1);
    break;
  case Shifter::DOWN:
    Serial.println("DOWN SHIFT");
    targetChanged = setTargetGear(targetGear - 1);
    break;
  }
  return targetChanged;
}

int checkForGearShiftSerial()
{
  if (Serial.available())
  {
    String receivedString = Serial.readStringUntil('\n');
    if (receivedString.equals("SUPERUP"))
    {
      return Shifter::SUPER_UP;
    }
    else if (receivedString.equals("UP"))
    {
      return Shifter::UP;
    }
    else if (receivedString.equals('SUPERDOWN'))
    {
      return Shifter::DOWN;
    }
    else if (receivedString.equals('DOWN'))
    {
      return Shifter::DOWN;
    }
    else
    {
      return Shifter::NONE;
    }
  }
  else
  {
    return Shifter::NONE;
  }
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

void printCurrentPosition()
{
  // Serial.print("Current Position: ");
  Serial.println(currentPosition, 1); // Use 1 decimal places for floating-point numbers
  // Serial.println(dataSize);
}

/////////////////////////////
// UNUSED CODE
///////////////////////////

// OPEN LOOP POSITION TRACKING

float updateOpenLoopPosition(float velocity, unsigned long currentTime)
{
  unsigned long elapsedTime = currentTime - previousMillis;
  currentPosition += velocity * (elapsedTime / 1000.0); // Integration formula
  previousMillis = currentTime;                         // Update previousMillis
  return currentPosition;
}

// Function to calculate the position based on time
float trapezoidalMotionProfile(unsigned long currentTime)
{
  // Calculate elapsed time since the motion profile started
  unsigned long elapsedTime = currentTime - startTime;

  // Calculate the time required to reach maximum velocity (acceleration phase)
  float timeToMaxVelocity = maxVelocity / acceleration;

  // Calculate the time required for the deceleration phase (same as acceleration time)
  float timeToDecelerate = timeToMaxVelocity;

  // Calculate the distance traveled during acceleration and deceleration phases
  float distanceAccelDecel = 0.5 * acceleration * timeToMaxVelocity * timeToMaxVelocity;

  // Calculate the remaining distance to travel at maximum velocity
  float remainingDistance = targetPosition - (2 * distanceAccelDecel);

  if (elapsedTime < timeToMaxVelocity)
  {
    // In the acceleration phase
    currentVelocity = acceleration * elapsedTime;
    currentPosition = 0.5 * acceleration * elapsedTime * elapsedTime;
  }
  else if (elapsedTime < (timeToMaxVelocity + timeToDecelerate))
  {
    // In the constant velocity phase
    float timeInMaxVelocity = elapsedTime - timeToMaxVelocity;
    currentVelocity = maxVelocity;
    currentPosition = distanceAccelDecel + maxVelocity * timeInMaxVelocity;
  }
  else if (currentPosition < targetPosition)
  {
    // In the deceleration phase
    float timeInDeceleration = elapsedTime - (timeToMaxVelocity + timeToDecelerate);
    currentVelocity = maxVelocity - (acceleration * timeInDeceleration);
    currentPosition = targetPosition - (0.5 * acceleration * timeInDeceleration * timeInDeceleration);
  }

  return currentVelocity;
}