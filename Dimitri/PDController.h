#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

#include <Arduino.h>

class PDController
{
public:
    PDController(double Kp, double Kd) : Kp(Kp), Kd(Kd), prevError(0) {}
    void reset()
    {
        prevError = 0.0;
    }
    double error;
    double errorChange;
    double speedControl;

    // Calculates the speed control with a range of -100.0 to 100.0%
    double calculatePDSpeedControl(double currentPosition, double targetPosition)
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
};

#endif