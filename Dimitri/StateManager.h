#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>

class StateManager {
private:
    int _step = -1;
    int _nextStep = 0;
    bool _debugEnabled = false;
    bool _errorsPresent = false;


    unsigned long _stepStartTime = 0;
    String _stepDescription = "";

public:
    // Read-only property
    int Step; 
    bool FirstScan;
    bool ErrorsPresent;

    void SetDebug(bool enable) {
        _debugEnabled = enable;
    }

    void StepDescription(String description) {
        _stepDescription = description;
    }

    // Set a new step and reset the step timer
    void transitionToStep(int newStep) {
        _nextStep = newStep;
        _stepStartTime = millis();
    }

    void triggerError(String errorMessage) {
        if (_debugEnabled) {
            Serial.print("Error: ");
            Serial.println(errorMessage);
        }
        _errorsPresent = true;
    }

    void clearErrors() {
        _errorsPresent = false;
    }

    void run() {
        ErrorsPresent = _errorsPresent;
        if (FirstScan) {
            if(_debugEnabled) {
                Serial.print("Step: ");
                Serial.print(_step);
                Serial.print(" - ");
                Serial.println(_stepDescription);
            }
        }
        if (_step != _nextStep) {
            _step = _nextStep;
            _stepStartTime = millis();
            FirstScan = true;
        } else {
            FirstScan = false;
        }
        Step = _step;
    }

    // ms, Get the time the current step has been active
    unsigned long getStepActiveTime() const {
        return millis() - _stepStartTime;
    }
};

#endif // STATE_MANAGER_H
