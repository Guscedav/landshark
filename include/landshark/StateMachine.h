/*
 * StateMachine.h
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 24.07.2019
 *      Author: Marcel Honegger
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <cstdlib>
#include <stdint.h>
#include "RealtimeThread.h"
#include "Timer.h"

class Controller;
class DigitalOut;
class DigitalIn;
class AnalogIn;

/**
 * This class implements the state machine of the mobile robot LANDSHARK.
 */
class StateMachine : public RealtimeThread {

public:

    static const int16_t    OFF = 0;
    static const int16_t    IDLE = 1;
    static const int16_t    ON = 2;
    static const int16_t    MANUAL = 3;
    static const int16_t    REMOTE = 4;
    static const int16_t    STOP = 5;
    static const int16_t    BRAKE = 6;

                StateMachine(Controller& controller, DigitalOut& warnLamp, DigitalOut& stopOverSickFront, DigitalOut& stopOverSickRear, DigitalIn& powerEnabled, DigitalIn& brake, AnalogIn& voltageBatteryFront, AnalogIn& voltageBatteryMiddle, AnalogIn& voltageBatteryRear);
    virtual     ~StateMachine();
    void        setState(int16_t stateDemand);
    int16_t     getState();
    float       getInputVoltage();
    void        setManualTranslationalVelocity(float translationalVelocity);
    void        setManualRotationalVelocity(float rotationalVelocity);
    void        setRemoteTranslationalVelocity(float translationalVelocity);
    void        setRemoteRotationalVelocity(float rotationalVelocity);
    float       getManualTranslationalVelocity();
    float       getManualRotationalVelocity();

private:

    static const size_t     STACK_SIZE = 64*1024;       // stack size of private thread in [bytes]

    static const uint32_t   ENABLE_TIMEOUT = 500;       // timeout for enabling the servo motors, given in [ms]
    static const uint32_t   DISABLE_TIMEOUT = 500;      // timeout for disabling the servo motors, given in [ms]
    static const uint32_t   OFF_BLINK_DURATION = 500;   // pause when blinking the green lamp, given in [ms]
    static const uint32_t   IDLE_BLINK_DURATION = 200;  // pause when blinking the green lamp, given in [ms]
    static const uint32_t   WATCHDOG_TIMEOUT_MANUAL = 200;    // timeout for watchdog timer, given in [ms]
    static const uint32_t   WATCHDOG_TIMEOUT_REMOTE = 200;    // timeout for watchdog timer, given in [ms]
    static const float      MIN_CELL_VOLTAGE;           // minimum required battery cell voltage to controller, given in [V]

    Controller&             controller;
    DigitalOut&             warnLamp;
    DigitalOut&             stopOverSickFront;
    DigitalOut&             stopOverSickRear;
    DigitalIn&              powerEnabled;
    DigitalIn&              brake;
    AnalogIn&               voltageBatteryFront;
    AnalogIn&               voltageBatteryMiddle;
    AnalogIn&               voltageBatteryRear;

    int16_t                 state;
    int16_t                 stateDemand;
    float                   manualTranslationalVelocity;
    float                   manualRotationalVelocity;
    float                   remoteTranslationalVelocity;
    float                   remoteRotationalVelocity;
    Timer                   timer;
    Timer                   watchdogTimerManual;
    Timer                   watchdogTimerRemote;

    void        run();
};

#endif /* STATE_MACHINE_H_ */
