/*
 * StateMachine.cpp
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 24.07.2019
 *      Author: Marcel Honegger
 */

#include "Controller.h"
#include "DigitalOut.h"
#include "DigitalIn.h"
#include "AnalogIn.h"
#include "StateMachine.h"

using namespace std;

const float StateMachine::MIN_CELL_VOLTAGE = 6.1f;    // minimum required battery cell voltage to controller, given in [V]

/**
 * Creates a state machine object with references for a signal lamp.
 * @param brake a reference to a digital input for for checking brake state.
 * @param powerEnabled a reference to a digital input for checking the power state.
 * @param voltageBatteries a reference to analog inputs to read the battery voltage.
 */
StateMachine::StateMachine(Controller& controller, DigitalOut& warnLamp, DigitalOut& stopOverSickFront, DigitalOut& stopOverSickRear, DigitalIn& powerEnabled, DigitalIn& brake, AnalogIn& voltageBatteryFront, AnalogIn& voltageBatteryMiddle, AnalogIn& voltageBatteryRear) : RealtimeThread("StateMachine", STACK_SIZE), controller(controller), warnLamp(warnLamp), stopOverSickFront(stopOverSickFront), stopOverSickRear(stopOverSickRear), powerEnabled(powerEnabled), brake(brake), voltageBatteryFront(voltageBatteryFront), voltageBatteryMiddle(voltageBatteryMiddle), voltageBatteryRear(voltageBatteryRear) {

    // initialize periphery objects
    warnLamp = false;
    stopOverSickFront = false;
    stopOverSickRear = false;

    // initialize private objects

    state = OFF;
    stateDemand = OFF;

    manualTranslationalVelocity = 0.0f;
    manualRotationalVelocity = 0.0f;
    remoteTranslationalVelocity = 0.0f;
    remoteRotationalVelocity = 0.0f;
    
    watchdogTimerManual.start();
    watchdogTimerRemote.start();
}

/**
 * Deletes this object and releases all allocated resources.
 */
StateMachine::~StateMachine() {}

/**
 * Sets the desired state for this state machine.
 * @param stateDemand the desired state for this state machine, i.e. <code>OFF</code>, <code>ON</code>, <code>MANUAL</code> or <code>REMOTE</code>.
 */
void StateMachine::setState(int16_t stateDemand) {

    this->stateDemand = stateDemand;
}

/**
 * Gets the actual state of the state machine.
 * @return the actual state, i.e. <code>OFF</code>,
 * <code>IDLE</code>, <code>ON</code> or <code>MANUAL</code>.
 */
int16_t StateMachine::getState() {

    return state;
}

/**
 * Gets the actual input voltage from the batteries.
 * @return the battery voltage in [V].
 */
float StateMachine::getInputVoltage() {

    return voltageBatteryFront+voltageBatteryMiddle+voltageBatteryRear;
}

/**
 * Sets the translational velocity of the robot in the manual operation mode.
 * @param translationalVelocity the translational velocity, given in [m/s].
 */
void StateMachine::setManualTranslationalVelocity(float translationalVelocity) {
    
    manualTranslationalVelocity = translationalVelocity;
    
    watchdogTimerManual.reset();
}

/**
 * Gets the translational velocity of the robot in the manual operation mode.
 * @return the translational velocity, given in [m/s].
 */
float StateMachine::getManualTranslationalVelocity() {

    return manualTranslationalVelocity;

}

/**
 * Sets the rotational velocity of the robot in the manual operation mode.
 * @param rotationalVelocity the rotational velocity, given in [rad/s].
 */
void StateMachine::setManualRotationalVelocity(float rotationalVelocity) {
    
    manualRotationalVelocity = rotationalVelocity;
    
    watchdogTimerManual.reset();
}

/**
 * Gets the rotational velocity of the robot in the manual operation mode.
 * @return the rotational velocity, given in [m/s].
 */
float StateMachine::getManualRotationalVelocity() {

    return manualRotationalVelocity;

}

/**
 * Sets the translational velocity of the robot in the remote operation mode.
 * @param translationalVelocity the translational velocity, given in [m/s].
 */
void StateMachine::setRemoteTranslationalVelocity(float translationalVelocity) {
    
    remoteTranslationalVelocity = translationalVelocity;
    
    watchdogTimerRemote.reset();
}

/**
 * Sets the rotational velocity of the robot in the remote operation mode.
 * @param rotationalVelocity the rotational velocity, given in [rad/s].
 */
void StateMachine::setRemoteRotationalVelocity(float rotationalVelocity) {
    
    remoteRotationalVelocity = rotationalVelocity;
    
    watchdogTimerRemote.reset();
}

/**
 * This is the run logic of the state machine.
 */
void StateMachine::run() {

    while (waitForNextPeriod()) {

        // handle state machine

        switch (state) {

            case OFF:

                if (!brake) {

                    warnLamp = false;
                   stopOverSickFront = true;
                   stopOverSickRear = true;

                } else if (powerEnabled) {  // Manual Interraction neccesary: Press start Button on Robot to enable power!

                    warnLamp = true;
                    stopOverSickFront = false;
                    stopOverSickRear = false;

                    stateDemand = IDLE;
                    state = IDLE;

                } else {

                    warnLamp = false;
                    stopOverSickFront = false;
                    stopOverSickRear = false;
                }

                break;

            case IDLE:

                if (!powerEnabled) {

                    warnLamp = false;

                    state = OFF;

                } else if (brake) {  // Manual Interraction neccesary: Release the brake!

                    warnLamp = false;

                } else if (stateDemand == OFF) {

                    warnLamp = false;

                    state = OFF;

                } else if ( (stateDemand == ON) || (stateDemand == MANUAL) || (stateDemand == REMOTE) ) {

                    if ( (voltageBatteryFront >= MIN_CELL_VOLTAGE) && (voltageBatteryMiddle >= MIN_CELL_VOLTAGE) && (voltageBatteryRear >= MIN_CELL_VOLTAGE) ) {

                    warnLamp = true;

                    manualTranslationalVelocity = 0.0;
                    manualRotationalVelocity = 0.0;
                    remoteTranslationalVelocity = 0.0;
                    remoteRotationalVelocity = 0.0;

                    controller.setTranslationalVelocity(0.0f);
                    controller.setRotationalVelocity(0.0f);

                    controller.start();           

                    state = ON;

                    } else {

                        warnLamp = false;

                        stateDemand = OFF;

                    }

                } else {

                    warnLamp = false;    // if true: warning lamp on when robot could be powered on by software (ROS Node or Webserver)

                }

                break;

            case ON:

                if (!powerEnabled) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (brake) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if ( (voltageBatteryFront <= MIN_CELL_VOLTAGE) || (voltageBatteryMiddle <= MIN_CELL_VOLTAGE) || (voltageBatteryRear <= MIN_CELL_VOLTAGE) ) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == OFF) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == MANUAL) {

                    state = MANUAL;

                } else if (stateDemand == REMOTE) {

                    state = REMOTE;
                    
                } else {
                    
                    controller.setTranslationalVelocity(0.0f);
                    controller.setRotationalVelocity(0.0f);

                    manualTranslationalVelocity = 0.0;
                    manualRotationalVelocity = 0.0;
                    remoteTranslationalVelocity = 0.0;
                    remoteRotationalVelocity = 0.0;

                    warnLamp = true;
                }

                break;

            case MANUAL:

                if (!powerEnabled) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (brake) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if ( (voltageBatteryFront <= MIN_CELL_VOLTAGE) || (voltageBatteryMiddle <= MIN_CELL_VOLTAGE) || (voltageBatteryRear <= MIN_CELL_VOLTAGE) ) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == OFF) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == ON) {

                    state = ON;

                } else if (stateDemand == REMOTE) {

                    state = REMOTE;
                    
                } else {
                    
                    if (watchdogTimerManual > WATCHDOG_TIMEOUT_MANUAL) {
                        
                        controller.setTranslationalVelocity(0.0f);
                        controller.setRotationalVelocity(0.0f);
                        manualTranslationalVelocity = 0.0;
                        manualRotationalVelocity = 0.0;
                        
                    } else {
                        
                        controller.setTranslationalVelocity(manualTranslationalVelocity);
                        controller.setRotationalVelocity(manualRotationalVelocity);

                        warnLamp = true;
                    }
                }

                break;

            case REMOTE:

                if (!powerEnabled) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (brake) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if ( (voltageBatteryFront <= MIN_CELL_VOLTAGE) || (voltageBatteryMiddle <= MIN_CELL_VOLTAGE) || (voltageBatteryRear <= MIN_CELL_VOLTAGE) ) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == OFF) {

                    warnLamp = true;

                    stateDemand = OFF;
                    state = STOP;

                } else if (stateDemand == ON) {

                    state = ON;

                } else if (stateDemand == MANUAL) {

                    state = MANUAL;
                    
                } else {
                    
                    if (watchdogTimerRemote > WATCHDOG_TIMEOUT_REMOTE) {
                        
                        controller.setTranslationalVelocity(0.0f);
                        controller.setRotationalVelocity(0.0f);
                        remoteTranslationalVelocity = 0.0;
                        remoteRotationalVelocity = 0.0;
                        
                    } else {
                        
                        controller.setTranslationalVelocity(remoteTranslationalVelocity);
                        controller.setRotationalVelocity(remoteRotationalVelocity);

                        warnLamp = true;
                    }
                }

                break;

            case STOP:

                if ( (controller.getRotationalVelocity() <= 0.2) && (controller.getTranslationalVelocity() <= 0.2))  {

                    controller.setTranslationalVelocity(0.0f);
                    controller.setRotationalVelocity(0.0f);

                    stopOverSickFront = true;
                    stopOverSickRear = true;

                    controller.stop();

                    state = BRAKE;

                } else {

                    controller.setTranslationalVelocity(0.0f);
                    controller.setRotationalVelocity(0.0f);

                    warnLamp = true;

                }

                break;

            case BRAKE:

                if (powerEnabled) {

                    stopOverSickFront = true;
                    stopOverSickRear = true;

                    warnLamp = true;

                } else if (!brake) {  // not braked yet

                    warnLamp = true;

                } else {

                    stopOverSickFront = false;
                    stopOverSickRear = false;

                    warnLamp = false;



                    state = OFF;

                }

                break;

            default:

                state = OFF;
        }
    }
}