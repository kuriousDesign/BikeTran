#ifndef SOLENOID_H
#define SOLENOID_H

#include <Arduino.h>

enum SolenoidTypes : int
{
    DIGITAL = 0, // either on or off
    PWM = 1,     // pwm accepted, (0-255)
};

enum SolenoidSafety : int
{
    RESTRICTED = 0,    // restricted on time and enforced cooldown time
    CONTINUOUS_OK = 1, // continuous duty ok
};

class Solenoid
{
private:
    int pin;
    uint8_t brightness = 0; // 0 to 255
    bool isOn = false;
    int type = SolenoidTypes::DIGITAL;
    int safety = SolenoidSafety::RESTRICTED;

    int param1;
    int param2;
    int param3;
    unsigned long ellapsedTime_ms = 0;
    unsigned long lastModeChange_ms = 0;
    bool modeChange = true;
    bool cooldownActive = false;
    bool fireMethodActive = true; // this is set each time the method is being called
    bool sw = false;              // switch value
    void strobe();
    bool debug;

    void fade();
    void flash();
    uint8_t triangleWave(float minVal, float maxVal, float period, float elapsed_time);

public:
    Solenoid();
    void init(int pin, int type = SolenoidTypes::DIGITAL, int safety = SolenoidSafety::RESTRICTED, long max_on_ms = 0, long cooldown_ms = 1000);
    void fire(int pwm = 0);
    void run();
    bool outputIsOn();
    void changeMode(int next_mode, int param1 = 0, int param2 = 0, int param3 = 0);
    int mode;
    int cooldown_time_ms = 1000; // enforced time solenoid must be off before firing again, this will prevent mode change
    int max_on_time_ms = 0;      // max allowable on time for solenoid if not continuous type
    enum Modes : int
    {
        OFF = 0,
        ON = 1,     // param1: brightness (0-255) (if PWM type), DO NOT USE FOR
        STROBE = 2, // Triangle-Wave Oscillation betweeen two values, param1: period_ms, param2: minVal, param3: maxVal (0-255)
        FLASH = 3,  // Square-Wave Oscillation between OnVal and Off, param1: period_on_ms, param2: period_off_val, param3: onVal (0-255)
        FADE = 4,   // Set to OnVal -> hold on for time -> then fade to zero, param1: on_time_before_fade_ms, param2: transition_ms, param3: onVal (0-255)
    };
    void setDebug(bool setOn);
    void debugPrintln(String msg);
    void debugPrint(String msg);
};
#endif