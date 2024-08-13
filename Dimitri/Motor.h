#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>
#include <TimerOne.h>

class Motor {
public:
    struct Cfg {
        int8_t homingDir; //-1 for reverse, 1 for forward
        uint8_t homingType; //0 for none, 1 for home swith, 2 for limit switch, 3 for hardstop
        String unit; //'mm' or 'deg' for example
        double pulsesPerUnit;
        //double maxVelocity;
        //double maxAcceleration;
        double softLimitPositive; //units
        double softLimitNegative; //units
        bool invertDir;
        double positionTol; //units
        double zeroVelocityTol; //units
    };
    Motor(uint8_t* dirPin, uint8_t* speedPin, Encoder* encoder, Cfg* cfg);

    bool isHomed = false;
    bool isEnabled = false;
    bool atPosition;
    bool isStill;
    double actualPosition;
    int32_t actualEncoderPulses;
    double actualVelocity;
    double targetPosition;
    int8_t targetDir; //used during jogging
    double targetPower; //used during jogging
    enum States : int8_t {
        KILLED = -1,
        IDLE = 0,
        HOMING = 1,
        MOVING = 2,
        JOGGING = 3,
        HOLD_POSITION = 4
    };
    int16_t state = States::KILLED;
    bool zero();
    bool setPosition(double position);
    bool enable();
    bool moveAbs(double position);
    bool jogUsingPower(int8_t dir, double powerPercent); //dir: -1 for negative, 1 for positive
    bool stop();
    bool disable();
    bool hold_position();



    void init();
    double error;

private:
    double _outputPower; //between -100.0 and 100.0
    double _kp = 1.0; //proportional gain TODO: move these to the config struct
    double _kd = 0.0; //derivative gain: TODO: move these to the config struct
    double _countsPerUnit = 1.0;

    void run();
    double pdControl();

    // Timer interrupt service routine - reads encoder position and calculates actualVelocity
    static void timerISR();
    void handleTimerISR();

    Cfg* _cfg;
    uint8_t* _dirPin;
    uint8_t* _speedPin;
    Encoder* _encoder;
    int16_t _step;
    double lastPositions[3];
    unsigned long lastTimes[3];

    static Motor* instance;
};

#endif // MOTOR_H
