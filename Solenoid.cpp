#include "Solenoid.h"
#include <Arduino.h>
#include <math.h>

Solenoid::Solenoid()
{
}

void Solenoid::init(int pin, int type = SolenoidTypes::DIGITAL, int safety = SolenoidSafety::RESTRICTED, long max_on_ms = 0, long cooldown_ms = 1000)
{
    pin = pin;
    type = type;
    safety = safety;
    max_on_time_ms = max_on_ms;
    cooldown_time_ms = cooldown_ms;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

// call this method all the time, it will update near 60 Hz
void Solenoid::run()
{
    unsigned long checkTime = millis();

    if (checkTime != ellapsedTime_ms + lastModeChange_ms)
    {
        ellapsedTime_ms = checkTime - lastModeChange_ms;

        switch (mode)
        {
        case Solenoid::Modes::OFF:
            brightness = 0;
            isOn = false;
            break;
        case Solenoid::Modes::ON:
            brightness = param1;
            isOn = true;
            break;
        case Solenoid::Modes::FADE:
            fade();
        case Solenoid::Modes::STROBE:
            strobe();
            // Serial.println(brightness);
            break;
        case Solenoid::Modes::FLASH:
            flash();
            break;
        default:
            // digitalWrite(pin, LOW);
            break;
        };
        analogWrite(pin, brightness);
        modeChange = false;
    }

    if (safety == SolenoidSafety::RESTRICTED)
    {
        if (brightness > 0 || isOn)
        {
            if (ellapsedTime_ms >= max_on_time_ms)
            {
                cooldownActive = true;
                lastModeChange_ms = checkTime;
                isOn = false;
                brightness = 0;
                debugPrint("run(): output switched off at time: ");
                debugPrintln(String(lastModeChange_ms));
            }
        }
        else if (cooldownActive)
        {
            isOn = false;
            brightness = 0;
            if (cooldownActive & !fireMethodActive & ellapsedTime_ms > cooldown_time_ms) // enforces caller to drop the fire call
            {
                cooldownActive = false;
                lastModeChange_ms = checkTime;
                debugPrint("run(): cooldown deactivated at time: ");
                debugPrintln(String(lastModeChange_ms));
            }
        }
    }

    if (type == SolenoidTypes::DIGITAL)
    {
        digitalWrite(pin, isOn);
    }
    else if (type == SolenoidTypes::PWM)
    {
        analogWrite(pin, brightness);
    }
    fireMethodActive = false;
}

// change the mode and pass in mode parameters
void Solenoid::changeMode(int next_mode, int param1 = 0, int param2 = 0, int param3 = 0)
{
    Solenoid::param1 = param1;
    Solenoid::param2 = param2;
    Solenoid::param3 = param3;
    if (mode != next_mode)
    {
        lastModeChange_ms = millis();
        mode = next_mode;
        modeChange = true;
        sw = true;
        lastModeChange_ms = 0;
    }
}

// Call this once, pulses the solenoid on for max_time_ms, must drop this request to be able to fire again
void Solenoid::fire(int pwm = 0)
{
    fireMethodActive = true;
    if (!cooldownActive & !isOn)
    {
        isOn = true;
        lastModeChange_ms = millis();
        brightness = pwm;
        debugPrint("fire(): output is on at time: ");
        debugPrintln(String(lastModeChange_ms));
    }
}

uint8_t Solenoid::triangleWave(float minVal, float maxVal, float period, float elapsed_time)
{
    float phase = fmod(elapsed_time, period) / period;

    float amplitude = (maxVal - minVal);

    if (phase <= 0.5)
    {
        return uint8_t(minVal + amplitude * (2.0 * phase));
    }
    else
    {
        return uint8_t(maxVal - amplitude * (2.0 * (phase - 0.5)));
    }
}

void Solenoid::strobe() // Triangle-Wave Oscillation betweeen two values, param1: period_ms, param2: minVal, param3: maxVal (0-255)

{
    brightness = triangleWave(param2, param3, param1, ellapsedTime_ms);
    // Serial.print("brightness: ");
    // Serial.println(brightness);
}

void Solenoid::flash() // Square-Wave Oscillation between OnVal and Off, param1: period_on_ms, param2: period_off_val, param3: onVal (0-255)

{
    long rem = ellapsedTime_ms % (param1 + param2);
    if (rem < param1)
    {
        brightness = param3;
    }
    else
    {
        brightness = 0;
    }

    // Serial.print("brightness: ");
    // Serial.println(brightness);
}

void Solenoid::fade() // Set to OnVal -> hold on for time -> then fade to zero, param1: on_time_before_fade_ms, param2: transition_ms, param3: onVal (0-255)
{
    long rem = ellapsedTime_ms % (param1 + param2);
    float t = ellapsedTime_ms - param1;
    if (t < 0)
    {
        brightness = param3;
    }
    else if (t < param2)
    {
        brightness = round((1.0 - t / float(param2)) * float(param3));
    }
    else
    {
        brightness = 0;
    }

    // Serial.print("brightness: ");
    // Serial.println(brightness);
}

bool Solenoid::outputIsOn() // read the solenoid output
{
    if (brightness > 0 || isOn)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Solenoid::debugPrint(String msg)
{
    if (debug)
    {
        Serial.print(msg);
    }
}

void Solenoid::debugPrintln(String msg)
{
    if (debug)
    {
        Serial.println(msg);
    }
}
void Solenoid::setDebug(bool setOn)
{
    debug = setOn;
}