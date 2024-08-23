#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>
#include <TimerOne.h>

#define NUM_FILTER_POINTS 11

class Motor {
public:
    struct Cfg {
        String name;
        int8_t homingDir; //-1 for reverse, 1 for forward
        uint8_t homingType; //0 for none, 1 for home swith, 2 for limit switch, 3 for hardstop
        String unit; //'mm' or 'deg' for example
        double pulsesPerUnit;
        double maxVelocity; //units
        //double maxAcceleration; //units
        double softLimitPositive; //units
        double softLimitNegative; //units
        bool invertDir;
        double positionTol; //units
        double zeroVelocityTol; //units
        double kP; //proportional gain for PID, units
        double kD; //derivative gain for PID, units
        uint16_t nudgeTimeMs; //ms
        double nudgePower; //%
    };
    Motor(uint8_t dirPin, uint8_t speedPin, Encoder* encoder, Cfg* cfg, bool simMode = false);

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
        HOLD_POSITION = 4,
        STOPPING = 5
    };
    int16_t getState();
    
    bool zero();
    bool setPosition(double position);
    bool enable();
    bool moveAbs(double position);
    bool jogUsingPower(double powerPercent); //dir: -1 for negative, 1 for positive
    bool stop();
    bool disable();
    bool hold_position();



    void init();
    double error;
    void setDebug(bool state);
    String stateToString(); //current state to string
    void update(); //used to udpate motor velocities, should be 1000Hz or better
    void run(); //used to run the state machine

private:
    uint16_t _scanTimeUs = 4000; //TODO: update this inside the run() function

    bool _stopReq = false;
    bool _homingReq = false;
    bool _moveAbsReq = false;
    bool _jogReq = false;
    bool _holdReq = false;
    bool _zeroReq = false;
    bool _enableReq = false;
    bool _disableReq = false;

    bool _firstScan = false;
    int16_t _nextState = States::KILLED;    
    int16_t _state = States::IDLE;
    bool _simMode = false;
    bool _debug = false;
    double _outputPower; //between -100.0 and 100.0
    double _countsPerUnit = 1.0;

    bool _isStill = false;
    bool _atPosition = false;
    bool _isNudging = false;
    unsigned long _nudgeStartTime = 0;
    bool _nudgingStarted = false;

    double pdControl();

    void setOutputs();

    Cfg* _cfg;
    uint8_t _dirPin;
    uint8_t _speedPin;
    Encoder* _encoder;
    int16_t _step;
    
    double lastPositions[NUM_FILTER_POINTS];
    unsigned long lastTimes[NUM_FILTER_POINTS];

    void debugPrint(String msg);
    void debugPrintln(String msg);
    double _deltaPulses = 0;
};

#endif // MOTOR_H
