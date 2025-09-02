#include "Motor.h"
#include <Encoder.h>
//#include <TimerOne.h>
#include <Arduino.h>
#include "StateManager.h"
// Constructor
Motor::Motor(uint8_t dirPin, uint8_t speedPin, Encoder *encoder, Cfg *cfg, bool simMode, uint8_t dir2Pin, uint8_t homeSwPin)
    : _dirPin(dirPin), _speedPin(speedPin), _encoder(encoder), _cfg(cfg), _simMode(simMode), _dir2Pin(dir2Pin), _homeSwPin(homeSwPin)
{
    init();
}

bool Motor::requestProcess(uint8_t processId)
{
    if (ActiveProcess == MotorProcesses::NONE_PROCESS && (_state == States::IDLE || _state == States::KILLED))
    {
        String message = "New Process requested: " + String(processId);
        debugPrintln(message);
        ActiveProcess = processId;
        return true;
    } else if ( ActiveProcess == processId){
        //do nothing
    }
    else {
        String message = "Process request denied: " + String(processId);
        debugPrintln(message);
    }
    return false;
}

void Motor::runProcess()
{
    bool processFirstScan = (ActiveProcess != prevActiveProcess);
    prevActiveProcess = ActiveProcess;
    //SerialLogging::info("Motor %s - process Step: %d", _cfg->name.c_str(), processState.Step);
    switch (ActiveProcess)
    {
    case MotorProcesses::HOME:
        if (processFirstScan)
        {
            homeToHardstop(_cfg->homingDir, true);
            homeToSwitch(_cfg->homingDir, Sensors::HOME_SW,true);
            //SerialLogging::info("Motor %s - process Step: %d", _cfg->name.c_str(), processState.Step);(_cfg->homingDir, Sensors::HOME_SW, true);
            _isHomed = false;
        }
        else
        {
            switch (_cfg->homingType)
            {
            case Motor::HomingType::NONE:
                // No homing required
                _isHomed = true;
                ActiveProcess = MotorProcesses::NONE_PROCESS;
                break;
            case Motor::HomingType::HOME_SWITCH:
                //debugPrintln("homing to home sw");
                if (homeToSwitch(_cfg->homingDir, Sensors::HOME_SW))
                {
                    _isHomed = true;
                    ActiveProcess = MotorProcesses::NONE_PROCESS;
                }
                break;
            case Motor::HomingType::LIMIT_SWITCH:
                if (homeToSwitch(_cfg->homingDir, Sensors::LIMIT_SW_POS) || homeToSwitch(_cfg->homingDir, Sensors::LIMIT_SW_NEG))
                {
                    _isHomed = true;
                    ActiveProcess = MotorProcesses::NONE_PROCESS;
                }
                break;
            case Motor::HomingType::HARDSTOP:
                if (homeToHardstop(_cfg->homingDir))
                {
                    _isHomed = true;
                    ActiveProcess = MotorProcesses::NONE_PROCESS;
                }
                break;
            default:
                debugPrintln("Invalid homing type");
                ActiveProcess = MotorProcesses::NONE_PROCESS;
                break;
            }
        }
        break;
    }
    
}

// run the state machine
void Motor::run()
{
    runProcess();
    // INSTANT KILL
    if (_disableReq)
    {
        _nextState = States::KILLED;
        //debugPrintln("MOTOR (" + _cfg->name + ") - Instant Kill");
    }

    if (_nextState != _state)
    {
        _state = _nextState;
        debugPrintln("State: " + stateToString());
        _firstScan = true;
    }
    else
    {
        _firstScan = false;
    }

    // Update these properties that are set by the update function
    isStill = _isStill;
    atPositionAndStill = _atPositionAndStill;
    atPosition = _atPosition;
    isHomed = _isHomed;

    switch (_state)
    {
    case States::KILLED:
        isEnabled = false;
        _outputPower = 0.0;
        targetPower = 0.0;
        targetPosition = actualPosition;
        if (_enableReq)
        {
            _nextState = States::IDLE;
        }
        break;
    case States::IDLE:

        isEnabled = true;
        _outputPower = 0.0;

        if (_moveAbsReq)
        {
            _nudgingStarted = false;
            _nextState = States::MOVING;
        }
        else if (_jogReq)
        {
            _nudgingStarted = false;
            _nextState = States::JOGGING;
        }
        else if (_holdReq)
        {
            _nextState = States::HOLD_POSITION;
        }

        break;
    case States::JOGGING:
        checkIsNudging(true);
        if (_isNudging)
        {
            _outputPower = targetPower / abs(targetPower) * _cfg->nudgePower;
        }
        else
        {
            _outputPower = targetPower;
        }

        // TODO: add sticky input to check if motor is stalled using isStill and checking prev and current encoder counts

        if (_stopReq)
        {
            _nextState = States::STOPPING;
        }
        break;
    case States::MOVING:
        if (_stopReq)
        {
            _nextState = States::STOPPING;
        }
        else if (atPositionAndStill)
        {
            _nextState = States::IDLE;
        }
        else
        {
            _outputPower = pdControl();
        }
        break;
    case States::HOLD_POSITION:
        _outputPower = pdControl();
        break;

    case States::STOPPING:
        targetPower = 0.0;
        _outputPower = 0.0; // TODO: add deceleration based on cfg params and current velocity
        if (_jogReq)
        {
            _nextState = States::JOGGING;
        }
        else if (isStill)
        {
            targetPosition = actualPosition;
            _nextState = States::IDLE;
        }
        break;

    default:
        break;
    }

    setOutputs();

    // Update the encoder position if in simulation mode
    if (_simMode)
    {

        // this allows for accumulation of sub pulses to gather on the deltaPulses variable
        if (_outputPower == 0.0)
        {
            _deltaPulses = 0.0;
        }
        else
        {
            _deltaPulses += _outputPower * _cfg->maxVelocity * _cfg->pulsesPerUnit * double(_scanTimeUs) / 1000000.0; // assumes instantate acceleration
        }

        int32_t update_pulses = round(_deltaPulses);
        if (update_pulses != 0.0)
        {
            _deltaPulses -= update_pulses;
        }
        int32_t newEncoderPulses = actualEncoderPulses + update_pulses;
        _encoder->write(newEncoderPulses);
    }

    // RESET ALL REQUESTS
    _stopReq = false;
    //_homeReq = false;
    _moveAbsReq = false;
    _jogReq = false;
    _holdReq = false;
    _zeroReq = false;
    _enableReq = false;
    _disableReq = false;
}

bool Motor::homeToHardstop(int8_t dir, bool reset)
{
    processState.run();
    static double max_position = 0.0;
    _outputPower = 0.0;
    if (reset)
    {
        _isHomed = false;
        if (dir != HomingDir::NEGATIVE && dir != HomingDir::POSITIVE)
        {
            //debugPrintln("Invalid homing direction");
            static String errorMsg = "Invalid homing direction: " + String(dir);
            processState.triggerError(errorMsg.c_str());
            processState.transitionToStep(911);
        }
        else
        {
            processState.transitionToStep(0);
        }
    }
    else
    {
        switch (processState.Step)
        {
        case 0:
            processState.StepDescription("Initializing homing sequence");
            _isHomed = false;
            zero();
            max_position = actualPosition;
            processState.transitionToStep(10);
            break;
        case 10:
            processState.StepDescription("Moving towards hard stop");
            _outputPower = dir * _cfg->homingPwr;
            if ((dir > 0 && actualPosition > max_position) || (dir < 0 && actualPosition < max_position))
            {
                max_position = actualPosition;
                processState.transitionToStep(11);
            }
            else if (processState.getStepActiveTime() > 1000)
            {
                debugPrintln("Hardstop found");
                processState.transitionToStep(20);
            }
            break;
        case 11:
            processState.StepDescription("Resetting step time");
            _outputPower = dir * _cfg->homingPwr;
            processState.transitionToStep(10);
            break;
        case 20:
            processState.StepDescription("Setting position");
            _outputPower = dir * _cfg->homingPwr;
            setPosition(_cfg->homeOffsetFromZero);
            processState.transitionToStep(30);
        case 30:
            processState.StepDescription("Removing power");
            _outputPower = 0.0;
            // debugPrintln("Homing complete");
            processState.transitionToStep(1000);
            break;
        case 1000:
            processState.StepDescription("Homing complete");
            _outputPower = 0.0;
            _isHomed = true;
            return true;
            break;
        case 911:
            processState.StepDescription("Error");
            _outputPower = 0.0;
            break;
        }
    }

    return false;
}

bool Motor::homeToSwitch(int8_t searchDir, Sensors sensorId, bool reset)
{
    processState.run();
    static double recorded_position = 0.0;
    //_outputPower = 0.0;
    if (reset)
    {
        _isHomed = false;
        if (searchDir != HomingDir::NEGATIVE && searchDir != HomingDir::POSITIVE)
        {
            debugPrintln("Invalid homing direction");
            static String errorMsg = "Invalid homing direction: " + String(searchDir);
            processState.triggerError(errorMsg.c_str());
            processState.transitionToStep(911);
        }
        else
        {
            processState.transitionToStep(0);
        }
    }
    else
    {
        //SerialLogging::info("Motor %s - process Step: %d", _cfg->name.c_str(), processState.Step);
        switch (processState.Step)
        {
        case 0:
            processState.StepDescription("Initializing homing sequence");
            _isHomed = false;
            zero();
            recorded_position = 0.0;
            if (_state == States::IDLE){
                processState.transitionToStep(10);
            } else if (_state == States::KILLED) {
                processState.transitionToStep(1);
            }

            break;

        case 1:
            processState.StepDescription("Enabling motor");
            enable();
            if (_state == States::IDLE)
            {
                processState.transitionToStep(10);
            }
            break;
        case 10:
            processState.StepDescription("Searching for sensor");
            jogUsingPower(searchDir * _cfg->homingPwr);

            if (sensors[sensorId])
            {
                debugPrintln("homeToSwitch(): Sensor found!");
                processState.transitionToStep(20);
            }
            break;

        case 20:
            processState.StepDescription("Moving off sensor");
            jogUsingPower(searchDir * _cfg->homingPwr * 0.8);
            if (sensors[sensorId])
            {
                processState.resetStepTime();
            }
            else if (!sensors[sensorId] && processState.getStepActiveTime() > 50)
            {
                debugPrintln("Sensor no longer triggered");
                stop();
                processState.transitionToStep(21);
            }
            break;
        case 21: 
            if(_state == Motor::States::IDLE){
                processState.transitionToStep(30);
            }
        case 30:
            processState.StepDescription("Going to trigger sensor and record position");
            jogUsingPower(-searchDir * _cfg->homingPwr * 0.8); // change this back to 0.5 TODO!!!!!!!!!
            if (sensors[sensorId])
            {
                debugPrintln("recorded position");
                recorded_position = actualPosition;
                stop();
                processState.transitionToStep(31);
            }
            break;
        case 31:
            if (processState.getStepActiveTime() > 1500 && _state == States::IDLE)
            {
                processState.StepDescription("Letting motor settle and then setting recorded position");
                double adjustedSetPosition = (actualPosition - recorded_position) + _cfg->homeOffsetFromZero;
                setPosition(adjustedSetPosition);
                processState.transitionToStep(1000);
            }
            break;
        case 1000:
            processState.StepDescription("Homing complete");
            _isHomed = true;
            debugPrintln("Homing complete");
            return true;
            break;
        }
    }
    return false;
}

void Motor::setOutputs()
{
    if (_outputPower > 0.0)
    {
        digitalWrite(_dirPin, !_cfg->invertMotorDir);
        if (_dir2Pin != DUMMY_PIN)
        {
            digitalWrite(_dir2Pin, _cfg->invertMotorDir);
        }
    }
    else if (_outputPower < 0.0)
    {
        digitalWrite(_dirPin, _cfg->invertMotorDir);
        if (_dir2Pin != DUMMY_PIN)
        {
            digitalWrite(_dir2Pin, !_cfg->invertMotorDir);
        }
    }
    else
    {
        digitalWrite(_dirPin, false);
        if (_dir2Pin != DUMMY_PIN)
        {
            digitalWrite(_dir2Pin, false);
        }
    }

    if (_outputPower < -100.0)
    {
        _outputPower = -100.0;
    }
    else if (_outputPower > 100.0)
    {
        _outputPower = 100.0;
    }
    uint8_t speed = abs(round(_outputPower * 255.0 / 100.0));
    if (_outputPower != 0.0 && false)
    {
        debugPrint("Speed: ");
        debugPrintln(String(speed).c_str());
    }
    analogWrite(_speedPin, speed);
}

bool Motor::zero()
{
    _encoder->write(0);
    updatePosition();
    return true;
}

// returns true if command accepted
bool Motor::home()
{
    return requestProcess(MotorProcesses::HOME);
}

bool Motor::setPosition(double position)
{
    double diff = position - actualPosition;
    int32_t pulses = round(position * _cfg->pulsesPerUnit);
    _encoder->write(pulses * (_cfg->invertEncoderDir ? -1 : 1));
    reconditionFilteredData(diff);
    return true;
}

void Motor::reconditionFilteredData(double diff)
{
    for (int i = 0; i < NUM_FILTER_POINTS; i++)
    {
        lastPositions[i] += diff;
    }
}

bool Motor::moveAbs(double position)
{
    if (_state == States::IDLE || _state == States::MOVING)
    {
        targetPosition = position;
        _moveAbsReq = true;
        return true;
    }
    return false;
}

bool Motor::jogUsingPower(double powerPercent)
{
    if (_state == States::IDLE || _state == States::JOGGING || _state == States::STOPPING)
    {
        targetPower = powerPercent;
        _jogReq = true;
    }
    else
    {
        debugPrintln("jogging only allowed while idle or jogging");
        return false;
    }
    return true;
}

bool Motor::stop()
{
    _stopReq = true;
    return true;
}

bool Motor::enable()
{
    if (_disableReq)
    {
        return false;
        debugPrintln("Can't enable while disable request is active");
    }
    _enableReq = true;
    return true;
}

bool Motor::disable()
{
    _disableReq = true;
    return true;
}

bool Motor::hold_position()
{
    if (_state != States::KILLED)
    {
        targetPosition = actualPosition;
        _holdReq = true;
        return true;
    }
    return false;
}

void Motor::init()
{
    pinMode(_dirPin, OUTPUT);
    pinMode(_speedPin, OUTPUT);
    if (_dir2Pin != DUMMY_PIN)
    {
        pinMode(_dir2Pin, OUTPUT);
    }
    if (_homeSwPin != DUMMY_PIN)
    {
        //pinMode(_homeSwPin, INPUT_PULLUP);
    }
    _motorPrefix = "MOTOR (" + _cfg->name + ") ";
    String processName = String(_motorPrefix) + "Process";
    processState.updateName(processName);

    // Timer1.initialize(_scanTimeUs); // Initialize timer to trigger every 1000 microseconds
    // Timer1.attachInterrupt(update);
}

bool *Motor::updateSensors()
{
    sensors[Sensors::HOME_SW] = !digitalRead(_homeSwPin);
    sensors[Sensors::LIMIT_SW_POS] = false; //! digitalRead(_limitSwPosPin);
    sensors[Sensors::LIMIT_SW_NEG] = false; //! digitalRead(_limitSwNegPin);
    return sensors;
}

double Motor::pdControl()
{
    // Calculate the error (position error)

    // Calculate the speed control parameter (output)

    double speedControl = _cfg->kP * error - _cfg->kD * actualVelocity;
    // Constrain the speed control parameter between -100% and 100%
    if (speedControl > 100.0)
    {
        speedControl = 100.0;
    }
    else if (speedControl < -100.0)
    {
        speedControl = -100.0;
    }

    checkIsNudging();
    // Modify speed control if nudging
    if (_isNudging)
    {
        // debugPrintln("Nudging");
        speedControl = error / abs(error) * _cfg->nudgePower;
    }

    return speedControl;
}

// this will set the _isNudging flag
void Motor::checkIsNudging(bool isJogging)
{
    // check nudging
    _isNudging = false;
    if (_isStill && (!_atPositionAndStill || isJogging) && _cfg->nudgeTimeMs > 0)
    {
        if (!_nudgingStarted)
        {
            _nudgeStartTime = millis();
            _nudgingStarted = true;
        }

        if (millis() - _nudgeStartTime < _cfg->nudgeTimeMs)
        {
            _isNudging = true;
        }
    }
    else
    {
        _isNudging = false;
        _nudgingStarted = false;
    }
}

/*
void Motor::updateNew() {
    unsigned long currentTime = micros();

    // Read the encoder and scale to units
    actualEncoderPulses = _encoder->read();
    actualPosition = double(actualEncoderPulses) / _cfg->pulsesPerUnit;

    // Shift position and time history arrays to make space for the new entry
    memmove(&lastPositions[1], &lastPositions[0], (NUM_FILTER_POINTS - 1) * sizeof(double));
    memmove(&lastTimes[1], &lastTimes[0], (NUM_FILTER_POINTS - 1) * sizeof(unsigned long));
    lastPositions[0] = actualPosition;
    lastTimes[0] = currentTime;

    // Initialize status flags
    _atPosition = true;
    _isStill = true;

    // Calculate velocity over the most recent interval
    double deltaPosition = lastPositions[0] - lastPositions[1];
    double deltaTime = double(lastTimes[0] - lastTimes[1]) / 1.0e6; // Convert to seconds
    double velocity = deltaPosition / deltaTime;


    if (abs(velocity) > _cfg->zeroVelocityTol) {
        _isStill = false;
    }
        // Check if the motor is at the target position and if it is still
    if (abs(lastPositions[0] - targetPosition) > _cfg->positionTol || !_isStill) {
        _atPosition = false;
    }

    // Calculate the overall velocity considering all history points
    if (lastTimes[NUM_FILTER_POINTS - 1] != 0) {
        deltaPosition = lastPositions[0] - lastPositions[NUM_FILTER_POINTS - 1];
        deltaTime = double(lastTimes[0] - lastTimes[NUM_FILTER_POINTS - 1]) / 1.0e6; // Convert to seconds
        actualVelocity = deltaPosition / deltaTime;
    }

    if (_state == States::MOVING) {
        _outputPower = pdControl();
        setOutputs();
    }
}
*/

void Motor::updatePosition()
{
    // Read the encoder and scale to units
    int32_t dir = _cfg->invertEncoderDir ? -1 : 1;
    actualEncoderPulses = _encoder->read() * dir;
    actualPosition = double(actualEncoderPulses) / _cfg->pulsesPerUnit;
    if (_cfg->encoderRollover)
    {
        double range = _cfg->softLimitPositive - _cfg->softLimitNegative;
        actualPosition = actualPosition - range * round(actualPosition / range);
        while (actualPosition < _cfg->softLimitNegative)
        {
            actualPosition += range;
        }
        while (actualPosition >= _cfg->softLimitPositive)
        {
            actualPosition -= range;
        }
    }
}

void Motor::update() {
    // debugPrintln(_cfg->name + "motor " + " - Update");
    unsigned long currentTime = micros();
    unsigned long delataTime = currentTime - lastTime;
    lastTime = currentTime;
    if (delataTime <= 0) {
        delataTime = 1000;  // Prevent div-by-zero; assume 1ms scan
    }

    updatePosition();
    updateSensors();

    error = targetPosition - actualPosition;
    double deltaPosition = actualPosition - _prevPosition;
    int32_t deltaEncoderPulses = actualEncoderPulses - _prevEncoderPulses;

    // Simplified velocity: Unfiltered, then IIR low-pass (alpha=0.2 for smoothing ~5 samples)
    double new_vel = 1000000.0 * deltaPosition / double(delataTime);
    actualVelocity = 0.8 * actualVelocity + 0.2 * new_vel;

    // Simplified average abs(error): Reset on target change, else IIR
    if (_prevTargetPosition != targetPosition) {
        _prevTargetPosition = targetPosition;
        avgAbsError = abs(error);
    } else {
        avgAbsError = 0.8 * avgAbsError + 0.2 * abs(error);
    }

    _isStill = abs(actualVelocity) <= _cfg->zeroVelocityTol;
    _atPositionAndStill = avgAbsError <= _cfg->positionTol && _isStill;

    if (abs(error) > _cfg->positionTol) {
        _atPosition = false;
    } else {
        _atPosition = true;
    }

    if (_state == States::MOVING) {
        _outputPower = pdControl();
        setOutputs();
    }

    _prevPosition = actualPosition;
    _prevEncoderPulses = actualEncoderPulses;
}

int16_t Motor::getState()
{
    return _state;
}

void Motor::setDebug(bool state)
{
    if (state && !_debug)
    {
        _debug = state;
        debugPrintln("Debugging Enabled");
    }
    _debug = state;
    processState.SetDebug(state);
}

// gather motor data and convert into byte chunk, total bytes = int16, float, float, float = 14
byte* Motor::getMotorData(){
    byte* data = new byte[MOTOR_DATA_SIZE];
    unsigned int i = 0;
    unsigned long size = 0;
    float tmpFloat = 0.0f;

    // motor state - int16 (2bytes)
    size = sizeof(_state);
    memcpy(data + i, &_state, size);
    i += size;

    // motor position - float (4bytes)
    size = sizeof(tmpFloat);
    tmpFloat = actualPosition;
    memcpy(data + i, &tmpFloat, size);
    i += size;

    // motor velocity - float (4bytes)
    size = sizeof(tmpFloat);
    tmpFloat = actualVelocity;
    memcpy(data + i, &tmpFloat, size);
    i += size;

    // motor target position - float (4bytes)
    size = sizeof(tmpFloat);
    tmpFloat = targetPosition;
    memcpy(data + i, &tmpFloat, size);
    return data;
}


void Motor::debugPrintln(String msg)
{
    if (_debug)
    {
        String fullMsg = _motorPrefix + msg;
        SerialLogging::info(fullMsg.c_str());
    }
}

// void Motor::debugPrint(String msg)
// {
//     if (_debug)
//     {
//         Serial.print(msg);
//     }
// }

String Motor::stateToString()
{
    switch (_state)
    {
    case States::KILLED:
        return "KILLED";
    case States::IDLE:
        return "IDLE";
    case States::JOGGING:
        return "JOGGING";
    case States::MOVING:
        return "MOVING";
    case States::HOLD_POSITION:
        return "HOLD_POSITION";
    case States::STOPPING:
        return "STOPPING";
    default:
        return String(_state);
    }
}

double Motor::getOutputPower()
{
    return _outputPower;
}