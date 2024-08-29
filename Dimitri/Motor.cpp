#include "Motor.h"
#include <Encoder.h>
#include <TimerOne.h>
#include <Arduino.h>
// Constructor
Motor::Motor(uint8_t dirPin, uint8_t speedPin, Encoder* encoder, Cfg* cfg, bool simMode)
    : _dirPin(dirPin), _speedPin(speedPin), _encoder(encoder), _cfg(cfg), _simMode(simMode) {
    init();
}

// run the state machine
void Motor::run() {

    // INSTANT KILL
    if (_disableReq) {
        _nextState = States::KILLED;
     } 

    if (_nextState != _state) {
        _state = _nextState;
        debugPrint("MOTOR (" + _cfg->name + ") - State: ");
        debugPrintln(stateToString());
        _firstScan = true;
    } else {
        _firstScan = false;
    }

        // Update these properties that are set by the update function
    isStill = _isStill;
    atPosition = _atPosition;

    switch (_state) {
        case States::KILLED:
            isEnabled = false;
            _outputPower = 0.0;
            targetPower = 0.0;
            targetPosition = actualPosition;
            if (_enableReq) {
                _nextState = States::IDLE;
            }
            break;
        case States::IDLE:

            isEnabled = true;
            _outputPower = 0.0;
  

            if (_moveAbsReq) {
                _nudgingStarted = false;
                _nextState = States::MOVING;
            } else if (_jogReq) {
                _nudgingStarted = false;
                _nextState = States::JOGGING;
            } else if (_holdReq) {
                _nextState = States::HOLD_POSITION;
            }
            break;
        case States::JOGGING:
            checkIsNudging(true);
            if(_isNudging) {
                _outputPower = targetPower / abs(targetPower) * _cfg->nudgePower;
            } else {
                _outputPower = targetPower;
            }

            //TODO: add sticky input to check if motor is stalled using isStill and checking prev and current encoder counts

            if (_stopReq) {
                _nextState = States::STOPPING;
            } 
            break;
        case States::MOVING:
            if (_stopReq) {
                _nextState = States::STOPPING;
            }
            else if (atPosition) {
                _nextState = States::IDLE;
            } else {
                _outputPower = pdControl();
            }
            break;
        case States::HOLD_POSITION:
            break;
        case States::STOPPING:
            targetPower = 0.0;
            _outputPower = 0.0; //TODO: add deceleration based on cfg params and current velocity
            if (isStill) {
                targetPosition = actualPosition;
                _nextState = States::IDLE;
            }
            break;
        
        default:
            break;
    }

    setOutputs();

    // Update the encoder position if in simulation mode
    if(_simMode) {

        // this allows for accumulation of sub pulses to gather on the deltaPulses variable
        if (_outputPower == 0.0) {
            _deltaPulses = 0.0;
        } else {
            _deltaPulses += _outputPower * _cfg->maxVelocity * _cfg->pulsesPerUnit * double(_scanTimeUs)/1000000.0; //assumes instantate acceleration
        }
        
        int32_t update_pulses =round(_deltaPulses);
        if ( update_pulses != 0.0) {
            _deltaPulses -= update_pulses;
        }
        int32_t newEncoderPulses = actualEncoderPulses + update_pulses;
        _encoder->write(newEncoderPulses);
    }

    // RESET ALL REQUESTS
    _stopReq = false;
    _homingReq = false;
    _moveAbsReq = false;
    _jogReq = false;
    _holdReq = false;
    _zeroReq = false;
    _enableReq = false;
    _disableReq = false;
}

void Motor::setOutputs(){
    if (_outputPower > 0.0) {
        digitalWrite(_dirPin, !_cfg->invertDir);
    }
    else if (_outputPower < 0.0) {
        digitalWrite(_dirPin, _cfg->invertDir);
    }
    else {
        digitalWrite(_dirPin, false);
    }

    if (_outputPower < -100.0) {
        _outputPower = -100.0;
    } else if (_outputPower > 100.0) {
        _outputPower = 100.0;
    }
    uint8_t speed = abs(round(_outputPower * 255.0 / 100.0));
    if (_outputPower != 0.0 && false) {
        debugPrint("Speed: ");
        debugPrintln(String(speed));
    }
    analogWrite(_speedPin, speed);
}

bool Motor::zero() {
    _encoder->write(0);
    return true;
}

bool Motor::setPosition(double position) {
    int32_t pulses = round(position*_cfg->pulsesPerUnit);
    _encoder->write(pulses);
    for(int i = 0; i < NUM_FILTER_POINTS; i++) {
        lastPositions[i] = position;
        lastErrors[i] = targetPosition - position;
        sumErrors = abs(targetPosition - position)*double(NUM_FILTER_POINTS);
    }
    return true;
}

bool Motor::moveAbs(double position) {
    if (_state == States::IDLE || _state == States::MOVING) {
        targetPosition = position;
        _moveAbsReq = true;
        return true;
    }
    return false;
}

bool Motor::jogUsingPower(double powerPercent) {
    if (_state != States::KILLED) {
        targetPower = powerPercent;
        _jogReq = true;
    } else {
        debugPrintln("can't jog while Motor is killed, must enable first");
        return false;
    }
    return true;
}

bool Motor::stop() {
    _stopReq = true;
    return true;
}

bool Motor::enable() {
    if(_disableReq) {
        return false;
        debugPrintln("Can't enable while disable request is active");
    }
    _enableReq = true;
    return true;
}

bool Motor::disable() {
    _disableReq = true;
    return true;
}

bool Motor::hold_position() {
    if (_state != States::KILLED) {
        targetPosition = actualPosition;
        _holdReq = true;
        return true;
    }
    return false;
}

void Motor::init() {
    pinMode(_dirPin, OUTPUT);
    pinMode(_speedPin, OUTPUT);
    
    //Timer1.initialize(_scanTimeUs); // Initialize timer to trigger every 1000 microseconds
    //Timer1.attachInterrupt(update);
}




double Motor::pdControl() {
    // Calculate the error (position error)
    
    // Calculate the speed control parameter (output)

    double speedControl = _cfg->kP * error - _cfg->kD * actualVelocity;
    // Constrain the speed control parameter between -100% and 100%
    if (speedControl > 100.0) {
        speedControl = 100.0;
    } else if (speedControl < -100.0) {
        speedControl = -100.0;
    }



    checkIsNudging();
    // Modify speed control if nudging
    if (_isNudging) {
        //debugPrintln("Nudging");
        speedControl = error/abs(error) * _cfg->nudgePower;
    }

    return speedControl;
}

// this will set the _isNudging flag
void Motor::checkIsNudging(bool isJogging = false) {
    // check nudging
    _isNudging = false;
    if(_isStill && (!_atPosition || isJogging) && _cfg->nudgeTimeMs > 0) {
        if(!_nudgingStarted ) {
            _nudgeStartTime = millis();
            _nudgingStarted = true;
        } 

        if (millis() - _nudgeStartTime < _cfg->nudgeTimeMs) {
            _isNudging = true;
        }  
    }
    else {
        _isNudging = false;
        _nudgingStarted = false;
    }

}

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


void Motor::update() {
    unsigned long currentTime = micros();
    unsigned long delataTime = currentTime - lastTimes[_prevIndexFilter];
    if (delataTime <= 0){
        delataTime = 1000; // just make it non-zero and non-negative
    }

    unsigned long dTime = currentTime - lastTimes[_indexFilter];
    lastTimes[_indexFilter] = currentTime;
    if (dTime <= 0){
        dTime = 1000*NUM_FILTER_POINTS; // just make it non-zero and non-negative
    }


    
    // Read the encoder and scale to units
    actualEncoderPulses = _encoder->read();
    actualPosition = double(actualEncoderPulses) / _cfg->pulsesPerUnit;
    double dPosition = actualPosition - lastPositions[_indexFilter];
    lastPositions[_indexFilter] = actualPosition;
    error = targetPosition - actualPosition;
    double deltaPosition = actualPosition - _prevPosition;
    int32_t deltaEncoderPulses = actualEncoderPulses - _prevEncoderPulses;

    sumDeltaPositions += deltaPosition - lastDeltaPositions[_indexFilter];
    lastDeltaPositions[_indexFilter] = deltaPosition;

    sumTimes += delataTime - lastDeltaTimes[_indexFilter];
    lastDeltaTimes[_indexFilter] = delataTime;

    //actualVelocity = 1.0e6 * sumDeltaPositions/double(sumTimes);
    actualVelocity = 1.0e6 * double(dPosition) / double(dTime) ;


    // Shift position and time history arrays to make space for the new entry
    //memmove(&lastPositions[1], &lastPositions[0], (NUM_FILTER_POINTS - 1) * sizeof(double));
    //memmove(&lastTimes[1], &lastTimes[0], (NUM_FILTER_POINTS - 1) * sizeof(unsigned long));
    //lastPositions[0] = actualPosition;
    //lastTimes[0] = delataTime;
    double unfiltered_velocity = 1000000.0 * double(deltaPosition) / double(delataTime) ;


    //lastVelocities[_indexFilter] = unfiltered_velocity;
    //sumVelocities += unfiltered_velocity - lastVelocities[_prevIndexFilter];
    //actualVelocity = sumVelocities/double(NUM_FILTER_POINTS);
    //lastPositions[_indexFilter] = actualPosition;
    
    sumSpeeds += abs(unfiltered_velocity) - abs(lastVelocities[_indexFilter]);
    lastVelocities[_indexFilter] = unfiltered_velocity;

    _prevIndexFilter = _indexFilter;
    _indexFilter = (_indexFilter + 1);
    if(_indexFilter >= NUM_FILTER_POINTS) {
        _indexFilter = 0;
    }
    
    // if the targetPosition changes
    if (_prevTargetPosition != targetPosition) {
        _prevTargetPosition = targetPosition;
        sumErrors = abs(error)*double(NUM_FILTER_POINTS);
        for (int i = 0; i < NUM_FILTER_POINTS; i++) {
            lastErrors[i] = error;
        }
    } else {
        sumErrors += abs(error) - abs(lastErrors[_indexFilter]);
    }
    lastErrors[_indexFilter] = error;
    
    //_isStill = sumSpeeds/double(NUM_FILTER_POINTS) <= _cfg->zeroVelocityTol;
    _isStill = abs(actualVelocity) <= _cfg->zeroVelocityTol;
    _atPosition = sumErrors/double(NUM_FILTER_POINTS) <= _cfg->positionTol && _isStill;

    if (_state == States::MOVING) {
        _outputPower = pdControl();
        setOutputs();
    }

    _prevPosition = actualPosition;
    _prevEncoderPulses = actualEncoderPulses;
}


int16_t Motor::getState() {
    return _state;
}

void Motor::setDebug(bool state) {
    _debug = state;
    debugPrintln("Motor " + _cfg->name + " - Debugging Enabled"); 
}

void Motor::debugPrintln(String msg) {
    if (_debug) {
        Serial.println(msg);
    }

}

void Motor::debugPrint(String msg) {
    if (_debug) {
        Serial.print(msg);
    }
}

String Motor::stateToString() {
    switch (_state) {
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

double Motor::getOutputPower() {
    return _outputPower;
}