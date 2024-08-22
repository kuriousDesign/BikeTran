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
                _nextState = States::MOVING;
            } else if (_jogReq) {
                _nextState = States::JOGGING;
            } else if (_holdReq) {
                _nextState = States::HOLD_POSITION;
            }
            break;
        case States::JOGGING:
            _outputPower = targetPower;
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
            _outputPower = 0.0; //TODO: add deceleration based on cfg params and current velocity
            if (isStill) {
                targetPosition = actualPosition;
                _nextState = States::IDLE;
            }
            break;
        
        default:
            break;
    }

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



bool Motor::zero() {
    _encoder->write(0);
    return true;
}

bool Motor::setPosition(double position) {
    int32_t pulses = round(position*_cfg->pulsesPerUnit);
    _encoder->write(pulses);
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
    error = targetPosition - actualPosition;
    // Calculate the speed control parameter (output)
    double speedControl = _kp * error - _kd * actualVelocity;
    // Constrain the speed control parameter between -100% and 100%
    if (speedControl > 100.0) {
        speedControl = 100.0;
    } else if (speedControl < -100.0) {
        speedControl = -100.0;
    }
    return speedControl;
}

void Motor::update() {
    unsigned long currentTime = micros();
    //read the encoder and scale to units
    actualEncoderPulses = _encoder->read();
    actualPosition = double(actualEncoderPulses)/_cfg->pulsesPerUnit;
    lastPositions[2] = lastPositions[1];
    lastPositions[1] = lastPositions[0];
    lastPositions[0] = actualPosition;

    lastTimes[2] = lastTimes[1];
    lastTimes[1] = lastTimes[0];
    lastTimes[0] = currentTime;

    // Check if the motor is at the target position
    atPosition = true;
    for (int i = 0; i < 3; i++) {
        if (abs(lastPositions[i] - targetPosition) > _cfg->positionTol) {
            atPosition = false;
        }
    }
    isStill = true;
    if (abs(actualVelocity) > _cfg->zeroVelocityTol) {
        atPosition = false;
        isStill = false;
    }

    if (lastTimes[2] != 0) {
        double deltaPosition = lastPositions[0] - lastPositions[2];
        double deltaTime = double(lastTimes[0] - lastTimes[2]) / 1.0e6; // Convert to seconds
        actualVelocity = deltaPosition / deltaTime;
    }
    run();
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