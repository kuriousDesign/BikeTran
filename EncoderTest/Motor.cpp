#include "Motor.h"

// Initialize static member
Motor* Motor::instance = nullptr;

Motor::Motor(uint8_t* fwdPin, uint8_t* revPin, uint8_t* speedPin, Encoder* encoder)
    : _fwdPin(fwdPin), _revPin(revPin), _speedPin(speedPin), _encoder(encoder) {
    instance = this;
}

bool Motor::moveAbs(double position) {
    if (state == States::IDLE || state == States::MOVING) {
        state = States::MOVING;
        targetPosition = position;
        return true;
    }
    return false;
}

bool Motor::jogUsingPower(int8_t dir, double powerPercent) {
    if (state != States::KILLED) {
        targetPower = powerPercent;
        state = States::IDLE;
    }
    return true;
}

bool Motor::stop() {
    if (state != States::KILLED) {
        state = States::IDLE;
    }
    return true;
}

bool Motor::disable() {
    targetPower = 0.0;
    _outputPower = 0.0;
    state = States::KILLED;
    return true;
}

bool Motor::hold_position() {
    if (state != States::KILLED) {
        targetPosition = actualPosition;
        state = States::HOLD_POSITION;
        return true;
    }
    return false;
}

void Motor::init() {
    pinMode(*_fwdPin, OUTPUT);
    pinMode(*_revPin, OUTPUT);
    pinMode(*_speedPin, OUTPUT);
    
    Timer1.initialize(1000); // Initialize timer to trigger every 1000 microseconds
    Timer1.attachInterrupt(timerISR);
}

void Motor::run() {
    switch (state) {
    case States::KILLED:
        _outputPower = 0.0;
        break;
    case States::HOMING:
        break;
    
    case States::IDLE:
        _outputPower = 0.0;
        break;
    case States::JOGGING:
        _outputPower = targetPower;
        break;
    case States::MOVING:
        if (actualPosition == targetPosition) {
            state = States::IDLE;
        } else {
            _outputPower = pdControl();
        }
        break;
    case States::HOLD_POSITION:
        break;
    
    default:
        break;
    }

    if (_outputPower > 0) {
        digitalWrite(*_fwdPin, HIGH);
        digitalWrite(*_revPin, LOW);
    }
    else if (_outputPower < 0) {
        digitalWrite(*_fwdPin, LOW);
        digitalWrite(*_revPin, HIGH);
    }
    else {
        digitalWrite(*_fwdPin, LOW);
        digitalWrite(*_revPin, LOW);
    }

    if (_outputPower < -100.0) {
        _outputPower = -100.0;
    } else if (_outputPower > 100.0) {
        _outputPower = 100.0;
    }
    analogWrite(*_speedPin, abs(round(_outputPower * 255.0 / 100.0)));
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

void Motor::timerISR() {
    if (instance != nullptr) {
        instance->handleTimerISR();
    }
}

void Motor::handleTimerISR() {
    unsigned long currentTime = micros();
    //read the encoder and scale to units
    actualPosition = double(_encoder->read())/_countsPerUnit;
    lastPositions[2] = lastPositions[1];
    lastPositions[1] = lastPositions[0];
    lastPositions[0] = actualPosition;

    lastTimes[2] = lastTimes[1];
    lastTimes[1] = lastTimes[0];
    lastTimes[0] = currentTime;

    if (lastTimes[2] != 0) {
        double deltaPosition = lastPositions[0] - lastPositions[2];
        double deltaTime = double(lastTimes[0] - lastTimes[2]) / 1.0e6; // Convert to seconds
        actualVelocity = deltaPosition / deltaTime;
    }
    run();
}
