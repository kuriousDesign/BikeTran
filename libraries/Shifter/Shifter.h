#ifndef SHIFTER_H
#define SHIFTER_H

#include <Arduino.h>
#include "StickyInput.h"

class Shifter
{
public:
    Shifter(int pin_up, int pin_down)
        : PIN_UP_SHIFT(pin_up),
          PIN_DOWN_SHIFT(pin_down),
          si_up(StickyInput(pin_up, 10, true, true)),
          si_down(StickyInput(pin_down, 10, true, true)),
          si_up_super(StickyInput(pin_up, 1000, true, true)),
          si_down_super(StickyInput(pin_down, 1000, true, true))
    {
        pinMode(PIN_UP_SHIFT, INPUT_PULLUP);
        pinMode(PIN_DOWN_SHIFT, INPUT_PULLUP);
        // Serial.print("pin_up");
        // Serial.println(PIN_UP_SHIFT);
    }

    enum ShiftTypes : int
    {
        NONE = 0,
        UP = 1,         // up one
        DOWN = 2,       // down one
        SUPER_UP = 3,   // all the way up
        SUPER_DOWN = 4, // all the way down
        ABS = 5,        // go to specific gear

    };

    int shiftReqNum = ShiftTypes::NONE;
    unsigned long previousMillisUp = 0;
    unsigned long previousMillisDown = 0;

    // checks the states of the shifter sticky inputs and returns the shift type
    int checkShiftReq()
    {
        int shiftNum = ShiftTypes::NONE;

        if (si_down_super.read())
        {
            shiftNum = ShiftTypes::SUPER_DOWN;
        }
        else if (si_down.read())
        {
            shiftNum = ShiftTypes::DOWN;
        }
        else if (si_up_super.read())
        {
            // Serial.println("super up");
            shiftNum = ShiftTypes::SUPER_UP;
        }
        else if (si_up.read())
        {
            // Serial.println("up");
            shiftNum = ShiftTypes::UP;
        }

        return shiftNum;
    }

private:
    const int SHORT_SHIFT_MS = 1;
    const int LONG_SHIFT_MS = 1000;
    int preShiftType = ShiftTypes::NONE;
    int PIN_UP_SHIFT;
    int PIN_DOWN_SHIFT;
    bool shortStateUp = false;
    bool shortStateDown = false;
    bool longStateUp = true;
    bool longStateDown = true;
    StickyInput si_down_super;
    StickyInput si_up_super;
    StickyInput si_down;
    StickyInput si_up;
};

#endif