#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>
#include <TimerOne.h>
#include "StateManager.h"

#define NUM_FILTER_POINTS 5

class Motor
{
public:
    // HomingType enum
    enum HomingType : uint8_t
    {
        NONE = 0,
        HOME_SWITCH = 1,
        LIMIT_SWITCH = 2,
        HARDSTOP = 3
    };

    enum HomingDir : int8_t
    {
        NEGATIVE = -1,
        POSITIVE = 1
    };

    enum Sensors : uint8_t
    {
        HOME_SW = 0,
        LIMIT_SW_POS = 1,
        LIMIT_SW_NEG = 2
    };

    static const uint8_t DUMMY_PIN = 255;

    struct Cfg
    {
        String name;
        int8_t homingDir;   //-1 for reverse, 1 for forward
        uint8_t homingType; // 0 for none, 1 for home swith, 2 for limit switch, 3 for hardstop
        double homeOffsetFromZero; // units
        String unit;        //'mm' or 'deg' for example
        double pulsesPerUnit;
        double maxVelocity; // units
        // double maxAcceleration; //units
        double softLimitPositive; // units
        double softLimitNegative; // units
        bool invertEncoderDir;
        bool encoderRollover; // if true, uses soft limits values for rollover
        bool invertMotorDir;
        double positionTol;     // units
        double zeroVelocityTol; // units
        double kP;              // proportional gain for PID, units
        double kD;              // derivative gain for PID, units
        uint16_t nudgeTimeMs;   // ms
        double nudgePower;      //%
    };
    Motor(uint8_t dirPin, uint8_t speedPin, Encoder *encoder, Cfg *cfg, bool simMode = false, uint8_t dir2Pin = DUMMY_PIN, uint8_t homeSwPin = DUMMY_PIN);

    bool isHomed = false;
    bool isEnabled = false;
    bool atPositionAndStill; // motor position is near target position and is still
    bool atPosition;         // motor position is near target position

    bool* sensors;

    bool isStill; // motor velocity is near zero
    double actualPosition;
    int32_t actualEncoderPulses;
    double actualVelocity;
    double targetPosition;
    int8_t targetDir;   // used during jogging
    double targetPower; // used during jogging
    enum States : int8_t
    {
        KILLED = -1,
        IDLE = 0,
        HOMING = 1,
        MOVING = 2,
        JOGGING = 3,
        HOLD_POSITION = 4,
        STOPPING = 5
    };
    int16_t getState();
    double getOutputPower();
   

    bool zero();
    bool setPosition(double position);
    bool enable();
    bool moveAbs(double position);
    bool jogUsingPower(double powerPercent); // dir: -1 for negative, 1 for positive
    bool stop();
    bool disable();
    bool hold_position();
    bool home();

    void init();
    double error;
    void setDebug(bool state);
    String stateToString(); // current state to string
    void update();          // used to udpate motor velocities, should be 1000Hz or better
    // void updateNew();
    void run(); // used to run the state machine

private:
    uint16_t _scanTimeUs = 4000; // TODO: update this inside the run() function

    bool _stopReq = false;
    bool _homeReq = false;
    bool _moveAbsReq = false;
    bool _jogReq = false;
    bool _holdReq = false;
    bool _zeroReq = false;
    bool _enableReq = false;
    bool _disableReq = false;
    double _homingPwr = 20.0;
    StateManager _homingState;
 

    bool _firstScan = false;
    int16_t _nextState = States::KILLED;
    int16_t _state = States::IDLE;
    bool _simMode = false;
    bool _debug = false;
    double _outputPower; // between -100.0 and 100.0
    double _countsPerUnit = 1.0;

    bool _isStill = false;
    bool _atPositionAndStill = false;
    bool _atPosition = false;
    bool _isNudging = false;
    bool _isHomed = false;
    unsigned long _nudgeStartTime = 0;
    bool _nudgingStarted = false;

    void checkIsNudging(bool isJogging = false);
    double pdControl();
    bool homeToHardstop(int8_t dir, bool reset=false);
    bool homeToSwitch(int8_t searchDir, Sensors sensorId, bool reset=false);
    void setOutputs();
    void updatePosition();
    void reconditionFilteredData(double diff);
    bool* updateSensors();

    Cfg *_cfg;
    uint8_t _dirPin;
    uint8_t _dir2Pin; // used for dual direction motor boards, like L298N
    uint8_t _homeSwPin;
    uint8_t _speedPin;
    Encoder *_encoder;
    int16_t _step;

    uint8_t _indexFilter = 0;
    uint8_t _prevIndexFilter = NUM_FILTER_POINTS - 1;
    // double lastPositions[NUM_FILTER_POINTS];
    double lastPositions[NUM_FILTER_POINTS];
    double lastDeltaPositions[NUM_FILTER_POINTS];
    double sumDeltaPositions = 0.0;
    double lastVelocities[NUM_FILTER_POINTS];
    double lastErrors[NUM_FILTER_POINTS];
    double sumVelocities = 0.0;
    double sumSpeeds = 0.0; // used to detect isStill
    double sumErrors = 0.0;
    unsigned long lastTimes[NUM_FILTER_POINTS];
    unsigned long lastDeltaTimes[NUM_FILTER_POINTS];
    unsigned long sumTimes = 0;
    double _prevTargetPosition = -1.0;
    double _prevPosition = 0.0;
    int32_t _prevEncoderPulses = 0;

    void debugPrint(String msg);
    void debugPrintln(String msg);
    double _deltaPulses = 0;
};

#endif // MOTOR_H
