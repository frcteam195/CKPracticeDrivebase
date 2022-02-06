#pragma once

#include "CKMath.hpp"
#include <cmath> 

class DriveHelper
{
private:

    double kThrottleDeadband = 0.05;
    double kWheelDeadband = 0.05;

    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    double kHighWheelNonLinearity = 0.65;
    double kLowWheelNonLinearity = 0.5;

    double kHighNegInertiaScalar = 4.0;

    double kLowNegInertiaThreshold = 0.65;
    double kLowNegInertiaTurnScalar = 3.5;
    double kLowNegInertiaCloseScalar = 4.0;
    double kLowNegInertiaFarScalar = 5.0;

    double kHighSensitivity = 0.95;
    double kLowSensitiity = 1.3;

    double kQuickStopDeadband = 0.2;
    double kQuickStopWeight = 0.1;
    double kQuickStopScalar = 5.0;

    double mOldWheel = 0.0;
    double mQuickStopAccumlator = 0.0;
    double mNegInertiaAccumlator = 0.0;
public:
    struct DriveMotorValues
    {
        double left;
        double right;
    };

    DriveMotorValues calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear) {
        return calculateOutput(throttle, wheel, isQuickTurn, isHighGear, 1.0);
    };

    DriveMotorValues calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear, double scalingFactor) {

        wheel = ck::math::normalizeWithDeadband(wheel, kWheelDeadband);
        throttle = ck::math::normalizeWithDeadband(throttle, kThrottleDeadband);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kLowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (std::fabs(wheel) > kLowNegInertiaThreshold) {
                    negInertiaScalar = kLowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kLowNegInertiaCloseScalar;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (std::fabs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                        + alpha * limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = std::fabs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        return DriveMotorValues{leftPwm * scalingFactor, rightPwm * scalingFactor};
    };

    double handleDeadband(double val, double deadband) {
        return (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;
    };
    
    static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    };

    static double limit(double v, double min, double max) {
        return std::fmin(max, std::fmax(min, v));
    };

};