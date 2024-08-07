#ifndef STICKYINPUT_H
#define STICKYINPUT_H

class StickyInput
{
private:
    bool prevState = false;
    int InputPin;
    unsigned int DelayOn_ms;
    unsigned long firstOn_ms = 0;
    bool IsOneShot;
    bool prevOutput = false;
    bool InvertPolarity = false;

public:
    // speed range: 10 is fasted and 1 is slowest
    StickyInput(int inputPin, unsigned int delayOn, bool isOneShot, bool invertPolarity = false)
        : InputPin(inputPin),
          DelayOn_ms(delayOn),
          IsOneShot(isOneShot),
          InvertPolarity(invertPolarity)
    {
        // pass
    }

    bool read()
    {
        bool output = false;
        bool currentState = digitalRead(InputPin);
        currentState = InvertPolarity ? !currentState : currentState;
        unsigned long currentMillis = millis();

        if (currentState && !prevState)
        {
            firstOn_ms = currentMillis;
            prevOutput = false;
        }

        if (currentState && currentMillis - firstOn_ms >= DelayOn_ms)
        {
            if (!IsOneShot)
            {

                output = true;
            }
            else if (!prevOutput)
            {
                output = true;
                prevOutput = true;
            }
        }

        prevState = currentState;
        return output;
    };
};

#endif