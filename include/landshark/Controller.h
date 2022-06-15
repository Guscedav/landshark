/*
 * Controller.h
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.07.2019
 *      Author: Marcel Honegger
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <cstdlib>
#include <stdint.h>
#include "RealtimeThread.h"
#include "Motion.h"
#include "LowpassFilter.h"
#include "AnalogOut.h"
#include "DigitalOut.h"
#include "BeckhoffEL5101.h"

//class AnalogOut;
//class DigitalOut;
//class BeckhoffEL5101;

/**
 * This class implements the motion controller of the mobile robot LANDSHARK.
 */
class Controller : public RealtimeThread {

public:

    static const float      TRANSLATIONAL_PROFILE_VELOCITY;
    static const float      TRANSLATIONAL_PROFILE_ACCELERATION;
    static const float      TRANSLATIONAL_PROFILE_DECELERATION;
    static const float      ROTATIONAL_PROFILE_VELOCITY;
    static const float      ROTATIONAL_PROFILE_ACCELERATION;
    static const float      ROTATIONAL_PROFILE_DECELERATION;

                Controller(AnalogOut& controlSignalLeft, AnalogOut& controlSignalRight, DigitalOut& backwardLeft, DigitalOut& forwardLeft, DigitalOut& backwardRight, DigitalOut& forwardRight, BeckhoffEL5101& encoderCounterLeft, BeckhoffEL5101& encoderCounterRight);
    virtual     ~Controller();
    void        setTranslationalVelocity(float translationalVelocity);
    void        setRotationalVelocity(float rotationalVelocity);
    double      getTranslationalVelocity();
    double      getRotationalVelocity();
    double      getDesiredTranslationalVelocity();
    double      getDesiredRotationalVelocity();
    double      getSpeedLeft();
    double      getSpeedRight();
    void        setX(double x);
    double      getX();
    void        setY(double y);
    double      getY();
    void        setAlpha(double alpha);
    double      getAlpha();
    double      getPhaseVoltageLeft();
    double      getPhaseVoltageRight();
    void        setKP(double KP);

private:

    static const size_t     STACK_SIZE = 64*1024;   // stack size of private thread in [bytes]

    static const double     WHEEL_DISTANCE;
    static const double     WHEEL_RADIUS;
    static const double     COUNTS_PER_TURN;
    static const double     LOWPASS_FILTER_FREQUENCY;
    static const double     KN;
    static const double     DEADZONE;
    static const double     MAX_VOLTAGE;
    static const double     MIN_CONTROL_VOLTAGE;
    static const double     MAX_CONTROL_VOLTAGE;
    static const double     OFFSET_CONTROL_VOLTAGE;
    static const double     MAX_ZENTRIPETAL_ACCELERATION; 

    AnalogOut&          controlSignalLeft;
    AnalogOut&          controlSignalRight;
    DigitalOut&         backwardLeft;
    DigitalOut&         forwardLeft;
    DigitalOut&         backwardRight;
    DigitalOut&         forwardRight;
    BeckhoffEL5101&     encoderCounterLeft;
    BeckhoffEL5101&     encoderCounterRight;
    Motion              translationalMotion;
    Motion              rotationalMotion;
    int16_t             previousValueCounterLeft;
    int16_t             previousValueCounterRight;
    LowpassFilter       speedLeftFilter;
    LowpassFilter       speedRightFilter;
    float               translationalVelocity;
    float               rotationalVelocity;
    double              ZentripetalAcceleration;
    double              desiredTranslationalVelocity;
    double              desiredRotationalVelocity;
    double              desiredSpeedLeft;
    double              desiredSpeedRight;
    double              feedForwardLeft;
    double              feedForwardRight;
    double              phaseVoltageLeft;
    double              phaseVoltageRight;
    double              controlVoltageLeft;
    double              controlVoltageRight;
    double              actualSpeedLeft;
    double              actualSpeedRight;
    double              actualTranslationalVelocity;
    double              actualRotationalVelocity;
    double              x;
    double              y;
    double              alpha;

    double              KP;

    void        run();
};

#endif /* CONTROLLER_H_ */
