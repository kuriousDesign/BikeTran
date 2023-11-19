#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

#include <Arduino.h>

class FunkyController
{
public:
    PDController(double Kp, double Kd) : Kp(Kp), Kd(Kd), prevError(0) {}
    double error;
    double errorChange;
    double speedControl;
    void reset()
    {
        firstScan = true;
    }
    double calculateSpeedControl(double currentPosition, double targetPosition, double currentVelocity)
    {
        if (firstScan)
        {
        }
        // Calculate the error (position error)
        error = targetPosition - currentPosition;

        // Calculate the change in error (velocity)
        errorChange = error - prevError;

        // Calculate the speed control parameter (output)
        speedControl = Kp * error + Kd * errorChange;

        // Constrain the speed control parameter between -100% and 100%
        if (speedControl > 100.0)
        {
            speedControl = 100.0;
        }
        else if (speedControl < -100.0)
        {
            speedControl = -100.0;
        }

        // Update the previous error
        prevError = error;
        return speedControl;
    }
    // not really used
    double calculatePDSpeedControl(double currentPosition, double targetPosition) // not really used
    {
        // Calculate the error (position error)
        error = targetPosition - currentPosition;

        // Calculate the change in error (velocity)
        errorChange = error - prevError;

        // Calculate the speed control parameter (output)
        speedControl = Kp * error + Kd * errorChange;

        // Constrain the speed control parameter between -100% and 100%
        if (speedControl > 100.0)
        {
            speedControl = 100.0;
        }
        else if (speedControl < -100.0)
        {
            speedControl = -100.0;
        }

        // Update the previous error
        prevError = error;
        return speedControl;
    }

private:
    double Kp;        // Proportional gain
    double Kd;        // Derivative gain
    double prevError; // Previous position error
    bool firstScan = false;
    int initialTargetDirection = 0; // 1: positive direction 2: negative direction
};

#endif