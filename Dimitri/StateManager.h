#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>
#include "SerialLogging.h"

class StateManager
{
private:
    String _name;
    int _step = -1;
    int _nextStep = 0;
    bool _debugEnabled = false;
    bool _errorsPresent = false;

    unsigned long _stepStartTime = 0;
    String _stepDescription = "";

public:
    // Constructor
    StateManager(String name) : _name(name) {}

    // Read-only property
    int Step;
    bool FirstScan;
    bool ErrorsPresent;

    void updateName(const String& name)
    {
        _name = name;
    }   

    void SetDebug(bool enable)
    {
        _debugEnabled = enable;
    }
    void StepDescription(const char* description)
    {
        if (FirstScan)
        {
            _stepDescription = description;
            if (_debugEnabled)
            {
                //Serial.println(_name + " Step: " + _step);// + " - " + _stepDescription);
                //SerialLogging::info("%s Step: %d - %s", _name.c_str(), _step, _stepDescription.c_str());
                SerialLogging::info("%s Step: %d", _name.c_str(), _step);
            }
        }
    }

    // Set a new step and reset the step timer
    void transitionToStep(int newStep)
    {
        _nextStep = newStep;
       
    }

    void triggerError(const char* msg)
    {
        if (_debugEnabled)
        {
            SerialLogging::error(msg);
        }
        _errorsPresent = true;
    }

    void clearErrors()
    {
        _errorsPresent = false;
    }

    void run()
    {
        ErrorsPresent = _errorsPresent;

        if (_step != _nextStep)
        {
            _step = _nextStep;
            _stepStartTime = millis();
            FirstScan = true;
        }
        else
        {
            FirstScan = false;
        }
        Step = _step;
    }

    // ms, Get the time the current step has been active
    unsigned long getStepActiveTime() const
    {
        return millis() - _stepStartTime;
    }
};

#endif // STATE_MANAGER_H
