#include "Motor.h"
#include <Encoder.h>
#include <TimerOne.h>

// Initialize static member
Motor* Motor::instance = nullptr;

Motor::Motor(uint8_t* dirPin, uint8_t* speedPin, Encoder* encoder, Cfg* cfg)
    : _dirPin(dirPin), _speedPin(speedPin), _encoder(encoder), _cfg(cfg) {
    instance = this;
}

bool Motor::zero() {
    _encoder->write(0);
    return true;
}

bool Motor::setPosition(double position) {
    _encoder->write(round(position*_cfg->pulsesPerUnit));
    return true;
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

bool Motor::enable() {
    if (state == States::KILLED) {
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
    pinMode(*_dirPin, OUTPUT);
    pinMode(*_speedPin, OUTPUT);
    
    Timer1.initialize(1000); // Initialize timer to trigger every 1000 microseconds
    Timer1.attachInterrupt(timerISR);
}

void Motor::run() {
    switch (state) {
    case States::KILLED:
        isEnabled = false;
        _outputPower = 0.0;
        break;
    case States::HOMING:
        break;
    
    case States::IDLE:
        isEnabled = true;
        _outputPower = 0.0;
        break;
    case States::JOGGING:
        _outputPower = targetPower;
        break;
    case States::MOVING:
        if (atPosition) {
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
        digitalWrite(*_dirPin, !_cfg->invertDir);
    }
    else if (_outputPower < 0) {
        digitalWrite(*_dirPin, _cfg->invertDir);
    }
    else {
        digitalWrite(*_dirPin, false);
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
