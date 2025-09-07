#pragma once

#include <Arduino.h>
#include "CustomDataTypes.h"
#include "Motor.h"

#define NUM_MOTORS 3
enum Motors
{
    CLUTCH = 0,
    LINEAR_P = 1,
    LINEAR_S = 2,
};

////////////////////////////////////////////////////
// PINOUT
////////////////////////////////////////////////////

// MEGA PWM PINS: 2,3,4,5,6,7,8,9,10,11,12,13,44,45,46
// MEGA EXTERNAL INTERRUPTS FOR ENCODER A SIGNAL PINS: 2, 3, 18, 19, 20, 21 (B SIGNAL CAN USE NORMAL DIG INPUTS)

// LINEAR MOTOR (P) PRIMARY - WIRED TO SHIELD MOTOR A - RED: mtr+, WHITE: mtr-, BLUE: encVCC, BLACK: encGND,  YELLOW: encA, GREEN: encB,
#define PIN_LINEAR_P_PWM 3
#define PIN_LINEAR_P_DIR 12

// LINEAR MOTOR S (SECONDARY) - WIRED TO SHIELD MOTOR B - RED: mtr+, WHITE: mtr-, BLUE: encGND, BLACK: encVCC,  YELLOW: encA, GREEN: encB,
#define PIN_LINEAR_S_PWM 11
#define PIN_LINEAR_S_DIR 13

// CLUTCH MOTOR - WIRED TO STANDALONE MOTOR BOARD A - RED MOTOR WIRE IS CONNECTED TO OUT1 AND BLACK TO OUT2
#define PIN_CLUTCH_PWM 44     // --> wires to ENA pin of L298N driver board
#define PIN_CLUTCH_DIR_IN1 45 // --> wires to IN1 of L298N driver board
#define PIN_CLUTCH_DIR_IN2 46 // --> wires to IN2 of L298N driver board

#define PIN_LINEAR_P_ENC_A 39 // green wire of enc
#define PIN_LINEAR_P_ENC_B 38 // yellow wire of enc

#define PIN_LINEAR_S_ENC_A 36 // green wire of enc
#define PIN_LINEAR_S_ENC_B 37 // yellow wire of enc

#define PIN_CLUTCH_ENC_A 41 // yellow wire of enc
#define PIN_CLUTCH_ENC_B 40 // white wire of enc

#define PIN_EINK_BIT0 22 // NOTE THAT PINS 22, 24, 26, & 28 ARE USED AS OUTPUTS FOR GEAR NUMBER DISPLAY ON E-INK
#define PIN_EINK_BIT1 24
#define PIN_EINK_BIT2 26
#define PIN_EINK_BIT3 28
#define NUM_BITS 4

#define PIN_CLUTCH_POS_LIM 35 // detects 228 deg
#define PIN_CLUTCH_NEG_LIM 34 // detects 0 deg
#define PIN_SHIFT_UP 32       // ORANGE WIRE ON SHIFTER
#define PIN_SHIFT_DOWN 33     // RED WIRE ON SHIFTER

#define PIN_CLUTCH_SOL -1 // not used

const bool SIM_MODE = false; // set to true to simulate motor behavior (encoders positions for now, TODO: simulate lim switches)      

enum Inputs
{
    ShiftDownSw = 0,
    ShiftUpSw = 1,
    ClutchNegLimSw = 2,
    ClutchPosLimSw = 3
};

// ENCODERS
// NOTE: follow Motors enum for order below
// Example with rollover support
Encoder encoders[NUM_MOTORS] = {
    Encoder(PIN_CLUTCH_ENC_A, PIN_CLUTCH_ENC_B), // rollover handling done in Motor class
    Encoder(PIN_LINEAR_P_ENC_A, PIN_LINEAR_P_ENC_B),
    Encoder(PIN_LINEAR_S_ENC_A, PIN_LINEAR_S_ENC_B)};

// pulses / cppu ) 360/394.5 = deg
// 394.68deg reading after 360.0 deg rotation.
const double CLUTCH_PULSES_PER_UNIT = 2.0 * (445.12 / 360.0) * (394.5 / 360.0); // 480.0 / 360.0; //120PPR /360deg for the 600rpm motor
// notes from joe on AUG 19 2025 - 12mm per turn, 6.62 mm per gear
const double LINEAR_P_PULSES_PER_UNIT = 1.0; // 14.0 / 11.0 * 2.0 * 8600.0 / (double(NUM_GEARS) - 1.0); // 9200 is the max position, 488 is the min position
// 12 mm per turn, 6.13mm per gear
const double LINEAR_S_PULSES_PER_UNIT = LINEAR_P_PULSES_PER_UNIT;

const double MANUAL_LINEAR_JOG_PWR = 30.0;
const double MANUAL_CLUTCH_JOG_PWR = 70.0;
const double LinearHomingPwr = 100.0;
const int LinearNudgeTimeMsDuringHomingJog = 50;
const double ClutchHoldingPwr = 25.0;
const int clutchSolenoidJostleTimeMs = 20;
const double LinearKp = 500.0; //@11.05VDC
const double LinearKd = 0.0;
uint16_t LinearNudgeTimeMs = 5; // 15
const double LinearNudgePower = 80.0;
const double POSITION_CLUTCH_PEDALING = 0.0;   // deg
const double POSITION_CLUTCH_SHIFTING = 228.0; // deg
const double LINEAR_P_HOME_OFFSET = 6.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear
const double LINEAR_S_HOME_OFFSET = 0.3;       // units is gears, distance (measured in gears) to move away from limit switch in order to be in 1st Gear


Motor::Cfg clutchMotorCfg = {
    name : "clutch",                             // name
    homingDir : Motor::HomingDir::POSITIVE,      // homing direction
    homingType : Motor::HomingType::HOME_SWITCH, // homing type
    homeOffsetFromZero : 0.0,                    // home offset from zero
    homingPwr : 50.0,
    unit : "deg",
    pulsesPerUnit : CLUTCH_PULSES_PER_UNIT, // unit and pulses per unit
    maxVelocity : 1000.0,                   // max velocity
    softLimitPositive : 0.0,              // soft limit positive
    softLimitNegative : 360.0,             // soft limit negative
    invertEncoderDir : false,               // invert encoder
    encoderRollover : false,                // enable encoder rollover
    invertMotorDir : true,                  // invert motor direction
    positionTol : 3.0,                      // position tolerance
    zeroVelocityTol : 5.0,                  // zero velocity tolerance, units/sec
    kP : 10.0,                              // kP
    kD : 1.0,                               // kD
    nudgeTimeMs : 50.0,                     // nudge time
    nudgePower : 100.0                      // nudge power

};

Motor::Cfg linearPrimaryMotorCfg = {
    name : "linear_primary",
    homingDir : Motor::HomingDir::POSITIVE,
    homingType : Motor::HomingType::HARDSTOP,
    homeOffsetFromZero : LINEAR_P_HOME_OFFSET, // units
    homingPwr : 30.0,
    unit : "gear",
    pulsesPerUnit : LINEAR_P_PULSES_PER_UNIT,
    maxVelocity : 20.0,
    softLimitPositive : (6 + 0.5),
    softLimitNegative : 0.5,
    invertEncoderDir : false,
    encoderRollover : false,
    invertMotorDir : false,
    positionTol : 0.02,
    zeroVelocityTol : 0.05,
    kP : LinearKp,
    kD : LinearKd,
    nudgeTimeMs : LinearNudgeTimeMs,
    nudgePower : LinearNudgePower
};

Motor::Cfg linearSecondaryMotorCfg = {
    name : "linear_secondary",
    homingDir : Motor::HomingDir::NEGATIVE,
    homingType : Motor::HomingType::HARDSTOP,
    homeOffsetFromZero : LINEAR_S_HOME_OFFSET, // units
    homingPwr : 30.0,
    unit : "gear",
    pulsesPerUnit : LINEAR_S_PULSES_PER_UNIT,
    maxVelocity : 20.0,
    softLimitPositive : (3 + 0.5),
    softLimitNegative : 0.5,
    invertEncoderDir : false,
    encoderRollover : false,
    invertMotorDir : false,
    positionTol : 0.02,
    zeroVelocityTol : 0.05,
    kP : LinearKp,
    kD : LinearKd,
    nudgeTimeMs : LinearNudgeTimeMs,
    nudgePower : LinearNudgePower
};

// NOTE: follow Motors enum for order below
Motor::Cfg motorCfgs[NUM_MOTORS] = {
    // name, homeDir, homeType, unit, pulsesPerUnit, maxVelocity, softLimitPositive, softLimitNegative, invertEncoder, invertMotorDir, positionTol, zeroVelocityTol, kP, kD, nudgeTimeMs, nudgePower
    clutchMotorCfg,
    linearPrimaryMotorCfg,
    linearSecondaryMotorCfg};

// NOTE: follow Motors enum for order below
Motor motors[NUM_MOTORS] = {
    Motor(PIN_CLUTCH_DIR_IN1, PIN_CLUTCH_PWM, &encoders[Motors::CLUTCH], &motorCfgs[Motors::CLUTCH], SIM_MODE, PIN_CLUTCH_DIR_IN2, PIN_CLUTCH_NEG_LIM),
    Motor(PIN_LINEAR_P_DIR, PIN_LINEAR_P_PWM, &encoders[Motors::LINEAR_P], &motorCfgs[Motors::LINEAR_P], SIM_MODE),
    Motor(PIN_LINEAR_S_DIR, PIN_LINEAR_S_PWM, &encoders[Motors::LINEAR_S], &motorCfgs[Motors::LINEAR_S], SIM_MODE)};