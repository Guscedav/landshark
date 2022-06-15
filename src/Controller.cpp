/*
 * Controller.cpp
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 25.07.2019
 *      Author: Marcel Honegger
 */

#include <cmath>
//#include "AnalogOut.h"
//#include "DigitalOut.h"
//#include "BeckhoffEL5101.h"
#include "Controller.h"

using namespace std;

const float Controller::TRANSLATIONAL_PROFILE_VELOCITY = 6.0f;  // max 8 m/s
const float Controller::TRANSLATIONAL_PROFILE_ACCELERATION = 2.0f;
const float Controller::TRANSLATIONAL_PROFILE_DECELERATION = 2.0f;
const float Controller::ROTATIONAL_PROFILE_VELOCITY = 3.0f;
const float Controller::ROTATIONAL_PROFILE_ACCELERATION = 3.0f;
const float Controller::ROTATIONAL_PROFILE_DECELERATION = 3.0f;

const double Controller::WHEEL_DISTANCE = 0.6397;            // distance between wheels, given in [m]
const double Controller::WHEEL_RADIUS = 0.153;              // radius of wheels, given in [m]
const double Controller::COUNTS_PER_TURN = 29789.1;          // 4*48*40/11 = 698  resolution of encoder counter (2048*4) * gear ratio (40:18 * 18:11 = 40:11 = 3.6363)
const double Controller::LOWPASS_FILTER_FREQUENCY = 200.0;  // frequency of lowpass filter for actual speed values, given in [rad/s]
const double Controller::KN = 11.9;     // motor constant in rpm/V with gear
const double Controller::DEADZONE = 5.0;  //Deadzone for feed forward function in rpm
const double Controller::MAX_VOLTAGE = 36.0;    // supply voltage Motors
const double Controller::MIN_CONTROL_VOLTAGE = 0.1; // minimal control voltage input for motor controller
const double Controller::MAX_CONTROL_VOLTAGE = 3.2; // maximal control voltage input for motor controller
const double Controller::OFFSET_CONTROL_VOLTAGE = 1.65; // offset control voltage input for motor controller
const double Controller::MAX_ZENTRIPETAL_ACCELERATION = 6.0; // maximal zentripetal acceleration in m/s²

/**
 * Create and initialize a motion controller object.
 * @param servoMotorLeft a reference to the servo controller for the left motor.
 * @param servoMotorRight a reference to the servo controller for the right motor.
 */
Controller::Controller(AnalogOut& controlSignalLeft, AnalogOut& controlSignalRight, DigitalOut& backwardLeft, DigitalOut& forwardLeft, DigitalOut& backwardRight, DigitalOut& forwardRight, BeckhoffEL5101& encoderCounterLeft, BeckhoffEL5101& encoderCounterRight) : RealtimeThread("Controller", STACK_SIZE), controlSignalLeft(controlSignalLeft), controlSignalRight(controlSignalRight), backwardLeft(backwardLeft), forwardLeft(forwardLeft), backwardRight(backwardRight), forwardRight(forwardRight), encoderCounterLeft(encoderCounterLeft), encoderCounterRight(encoderCounterRight) {

    // initialize periphery drivers

    controlSignalLeft.write(0.0f);
    controlSignalRight.write(0.0f);

    backwardLeft.write(0);
    forwardLeft.write(0);
    backwardRight.write(0);
    forwardRight.write(0);

    // initialize local values

    translationalMotion.setProfileVelocity(TRANSLATIONAL_PROFILE_VELOCITY);
    translationalMotion.setProfileAcceleration(TRANSLATIONAL_PROFILE_ACCELERATION);
    translationalMotion.setProfileDeceleration(TRANSLATIONAL_PROFILE_DECELERATION);

    rotationalMotion.setProfileVelocity(ROTATIONAL_PROFILE_VELOCITY);
    rotationalMotion.setProfileAcceleration(ROTATIONAL_PROFILE_ACCELERATION);
    rotationalMotion.setProfileDeceleration(ROTATIONAL_PROFILE_DECELERATION);

    translationalVelocity = 0.0f;
    rotationalVelocity = 0.0f;

    previousValueCounterLeft = encoderCounterLeft.readPosition();
    previousValueCounterRight = encoderCounterRight.readPosition();

    x = 0.0;
    y = 0.0;
    alpha = 0.0;

    KP = 0.2;
}


/**
 * Delete the motion controller object and release all allocated resources.
 */
Controller::~Controller() {}

/**
* Sets proportional gain for translation for the controller.
* @param KP_TRANSLATION, given in [V/?].
*/
void Controller::setKP(double KP) {

    this->KP = KP;
}

/**
 * Gets voltage forwarded to the left SEVCON motor controller.
 * @param controlVoltageLeft the set voltage, given in [V].
 */
double Controller::getPhaseVoltageLeft() {

    return phaseVoltageLeft;
}

/**
 * Gets voltage forwarded to the right SEVCON motor controller.
 * @param controlVoltageRight the set voltage, given in [V].
 */
double Controller::getPhaseVoltageRight() {

    return phaseVoltageRight;
}

/**
 * Sets the desired translational velocity of the robot.
 * @param translationalVelocity the translational velocity, given in [m/s].
 */
void Controller::setTranslationalVelocity(float translationalVelocity) {

    this->translationalVelocity = translationalVelocity;
}

/**
 * gets the actual translational velocity of the robot.
 * @param actualTranslationalVelocity the actual translational velocity, given in [m/s].
 */
double Controller::getTranslationalVelocity() {

    return actualTranslationalVelocity;
}

/**
 * gets the desired translational velocity of the robot.
 * @param desiredTranslationalVelocity the desired translational velocity, given in [m/s].
 */
double Controller::getDesiredTranslationalVelocity() {

    return desiredTranslationalVelocity;
}

/**
 * Sets the desired rotational velocity of the robot.
 * @param rotationalVelocity the rotational velocity, given in [rad/s].
 */
void Controller::setRotationalVelocity(float rotationalVelocity) {

    this->rotationalVelocity = rotationalVelocity;
}

/**
 * Gets the actual rotational velocity of the robot.
 * @param actualRotationalVelocity the actual rotational velocity, given in [rad/s].
 */
double Controller::getRotationalVelocity() {

    return actualRotationalVelocity;

}

/**
 * Gets the desired rotational velocity of the robot.
 * @param desiredRotationalVelocity the desired rotational velocity, given in [rad/s].
 */
double Controller::getDesiredRotationalVelocity() {

    return desiredRotationalVelocity;

}

/**
 * Gets the actual speed of the left wheel.
 * @param actualSpeedLeft the actual speed left, given in [rpm].
 */
double Controller::getSpeedLeft() {

    return actualSpeedLeft;

}

/**
 * Gets the actual speed of the right wheel.
 * @param actualSpeedRight the actual speed right, given in [rpm].
 */
double Controller::getSpeedRight() {

    return actualSpeedRight;

}

/**
 * Sets the actual x coordinate of the robots position.
 * @param x the x coordinate of the position, given in [m].
 */
void Controller::setX(double x) {

    this->x = x;
}

/**
 * Gets the actual x coordinate of the robots position.
 * @return the x coordinate of the position, given in [m].
 */
double Controller::getX() {

    return x;
}

/**
 * Sets the actual y coordinate of the robots position.
 * @param y the y coordinate of the position, given in [m].
 */
void Controller::setY(double y) {

    this->y = y;
}

/**
 * Gets the actual y coordinate of the robots position.
 * @return the y coordinate of the position, given in [m].
 */
double Controller::getY() {

    return y;
}

/**
 * Sets the actual orientation of the robot.
 * @param alpha the orientation, given in [rad].
 */
void Controller::setAlpha(double alpha) {

    this->alpha = alpha;
}

/**
 * Gets the actual orientation of the robot.
 * @return the orientation, given in [rad].
 */
double Controller::getAlpha() {

    return alpha;
}

/**
 * This is the run logic of the motion controller.
 */
void Controller::run() {

    // initialize state values

    translationalVelocity = 0.0f;
    rotationalVelocity = 0.0f;

    previousValueCounterLeft = encoderCounterLeft.readPosition();
    previousValueCounterRight = encoderCounterRight.readPosition();

    // initialize analog outputs

    controlSignalLeft.write(0.0f);
    controlSignalRight.write(0.0f);

    // initialize filter

    speedLeftFilter.reset();
    speedLeftFilter.setPeriod(getPeriod());
    speedLeftFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    speedRightFilter.reset();
    speedRightFilter.setPeriod(getPeriod());
    speedRightFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);

    // enter periodic run loop;

    while (waitForNextPeriod()) {

        // increment motion planner for translational and rotational velocities

        translationalMotion.incrementToVelocity(translationalVelocity, static_cast<float>(getPeriod()));
        rotationalMotion.incrementToVelocity(rotationalVelocity, static_cast<float>(getPeriod()));

        // calculate 'ZentripetalAcceleration'

        ZentripetalAcceleration = abs(static_cast<double>(translationalMotion.velocity)*static_cast<double>(rotationalMotion.velocity));

        // limit maximal zentripetal acceleration, calculate 'desiredTranslationalVelocity' and 'desiredRotationalVelocity'

        if (ZentripetalAcceleration > MAX_ZENTRIPETAL_ACCELERATION) {

            desiredTranslationalVelocity = static_cast<double>(translationalMotion.velocity) * MAX_ZENTRIPETAL_ACCELERATION/ZentripetalAcceleration;
            desiredRotationalVelocity = static_cast<double>(rotationalMotion.velocity) * MAX_ZENTRIPETAL_ACCELERATION/ZentripetalAcceleration;

        } else {

            desiredTranslationalVelocity = static_cast<double>(translationalMotion.velocity);
            desiredRotationalVelocity = static_cast<double>(rotationalMotion.velocity);

        }

        // calculate the values 'desiredSpeedLeft' and 'desiredSpeedRight' using the kinematic model

        desiredSpeedLeft = (desiredTranslationalVelocity-WHEEL_DISTANCE/2.0*desiredRotationalVelocity)/WHEEL_RADIUS*60.0/2.0/M_PI;  // in rpm
        desiredSpeedRight = (desiredTranslationalVelocity+WHEEL_DISTANCE/2.0*desiredRotationalVelocity)/WHEEL_RADIUS*60.0/2.0/M_PI;  // in rpm

        // calculate actual speed of motors in [rpm]

        int16_t valueCounterLeft = encoderCounterLeft.readPosition();
        int16_t valueCounterRight = encoderCounterRight.readPosition();

        int16_t countsInPastPeriodLeft = -(valueCounterLeft-previousValueCounterLeft);  // reverse left encoder counter
        int16_t countsInPastPeriodRight = valueCounterRight-previousValueCounterRight;

        previousValueCounterLeft = valueCounterLeft;
        previousValueCounterRight = valueCounterRight;

        actualSpeedLeft = speedLeftFilter.filter(static_cast<double>(countsInPastPeriodLeft)/COUNTS_PER_TURN/getPeriod()*60.0);  // in rpm
        actualSpeedRight = speedRightFilter.filter(static_cast<double>(countsInPastPeriodRight)/COUNTS_PER_TURN/getPeriod()*60.0);  // in rpm

        // calculate feed forward 'feedForwardLeft' and 'feedForwardRight'

        if (desiredSpeedLeft-desiredSpeedRight > DEADZONE) {

            feedForwardLeft = desiredSpeedLeft/KN + 3.5;
            feedForwardRight = desiredSpeedRight/KN - 3.5;

        } else if (desiredSpeedLeft-desiredSpeedRight < -DEADZONE) {

            feedForwardLeft = desiredSpeedLeft/KN -3.5;
            feedForwardRight = desiredSpeedRight/KN + 3.5;

        } else {

            feedForwardLeft = desiredSpeedLeft/KN;
            feedForwardRight = desiredSpeedRight/KN;

        }

        // calculate motor phase voltages

        phaseVoltageLeft = KP*(desiredSpeedLeft-actualSpeedLeft) + feedForwardLeft;
        phaseVoltageRight = KP*(desiredSpeedRight-actualSpeedRight) + feedForwardRight;

        // calculate, limit and set control voltage for left and right motor
/*
	// OLD MOTOR CONTROLLER
	// ====================
        if (phaseVoltageLeft > 0.0) { // rotation forward
            if (phaseVoltageLeft > MAX_VOLTAGE) {
                controlVoltageLeft = MAX_CONTROL_VOLTAGE;

            } else {
                controlVoltageLeft = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*phaseVoltageLeft+MIN_CONTROL_VOLTAGE;
            }
            backwardLeft.write(0);
            forwardLeft.write(1);
        } else if (phaseVoltageLeft < 0.0) {    // rotation backward
            if (phaseVoltageLeft < -MAX_VOLTAGE) {
                controlVoltageLeft = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageLeft = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*(-phaseVoltageLeft)+MIN_CONTROL_VOLTAGE;
            }
            backwardLeft.write(1);
            forwardLeft.write(0);
        } else {                            // no rotation
            controlVoltageLeft = MIN_CONTROL_VOLTAGE;
        }

        if (phaseVoltageRight > 0.0) {           // rotation forward
            if (phaseVoltageRight > MAX_VOLTAGE) {
                controlVoltageRight = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageRight = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*phaseVoltageRight+MIN_CONTROL_VOLTAGE;
            }
            backwardRight.write(0);
            forwardRight.write(1);
        } else if (phaseVoltageRight < 0.0) {    // rotation backward
            if (phaseVoltageRight < -MAX_VOLTAGE) {
                controlVoltageRight = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageRight = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*(-phaseVoltageRight)+MIN_CONTROL_VOLTAGE;
            }
            backwardRight.write(1);
            forwardRight.write(0);
        } else {                            // no rotation
            controlVoltageRight = MIN_CONTROL_VOLTAGE;
        }
*/
	
/*
	// Duty Cycle with reverse
	// ====================
	controlVoltageLeft = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/(2.0*MAX_VOLTAGE)*phaseVoltageLeft + OFFSET_CONTROL_VOLTAGE;
	controlVoltageRight = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/(2.0*MAX_VOLTAGE)*phaseVoltageRight + OFFSET_CONTROL_VOLTAGE;

	//cerr << "left: " << phaseVoltageLeft << " / " << controlVoltageLeft << ",  right: " << phaseVoltageRight << " / " << controlVoltageRight << endl;

	if (controlVoltageLeft > MAX_CONTROL_VOLTAGE) {
		controlVoltageLeft = MAX_CONTROL_VOLTAGE;
	} else if (controlVoltageLeft < MIN_CONTROL_VOLTAGE) {
		controlVoltageLeft = MIN_CONTROL_VOLTAGE;
	}

	if (controlVoltageRight > MAX_CONTROL_VOLTAGE) {
		controlVoltageRight = MAX_CONTROL_VOLTAGE;
	} else if (controlVoltageRight < MIN_CONTROL_VOLTAGE) {
		controlVoltageRight = MIN_CONTROL_VOLTAGE;
	}
*/

	// Duty Cycle with reverse
	// ====================
        if (phaseVoltageLeft > 0.0) { // rotation forward
            if (phaseVoltageLeft > MAX_VOLTAGE) {
                controlVoltageLeft = MAX_CONTROL_VOLTAGE;

            } else {
                controlVoltageLeft = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*phaseVoltageLeft+MIN_CONTROL_VOLTAGE;
            }
            forwardLeft.write(1);
        } else if (phaseVoltageLeft < 0.0) {    // rotation backward
            if (phaseVoltageLeft < -MAX_VOLTAGE) {
                controlVoltageLeft = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageLeft = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*(-phaseVoltageLeft)+MIN_CONTROL_VOLTAGE;
            }
            forwardLeft.write(0);
        } else {                            // no rotation
            controlVoltageLeft = MIN_CONTROL_VOLTAGE;
        }

        if (phaseVoltageRight > 0.0) {           // rotation forward
            if (phaseVoltageRight > MAX_VOLTAGE) {
                controlVoltageRight = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageRight = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*phaseVoltageRight+MIN_CONTROL_VOLTAGE;
            }
            forwardRight.write(1);
        } else if (phaseVoltageRight < 0.0) {    // rotation backward
            if (phaseVoltageRight < -MAX_VOLTAGE) {
                controlVoltageRight = MAX_CONTROL_VOLTAGE;
            } else {
                controlVoltageRight = (MAX_CONTROL_VOLTAGE-MIN_CONTROL_VOLTAGE)/MAX_VOLTAGE*(-phaseVoltageRight)+MIN_CONTROL_VOLTAGE;
            }
            forwardRight.write(0);
        } else {                            // no rotation
            controlVoltageRight = MIN_CONTROL_VOLTAGE;
        }


        controlSignalLeft.write(static_cast<float>(controlVoltageLeft));
        controlSignalRight.write(static_cast<float>(controlVoltageRight));

        // calculate the values 'actualTranslationalVelocity' and 'actualRotationalVelocity' using the kinematic model

        actualTranslationalVelocity = (actualSpeedLeft+actualSpeedRight)*2.0*M_PI/60.0*WHEEL_RADIUS/2.0;
        actualRotationalVelocity = (actualSpeedRight-actualSpeedLeft)*2.0*M_PI/60.0*WHEEL_RADIUS/WHEEL_DISTANCE;

        // calculate the actual robot pose

        double deltaTranslation = actualTranslationalVelocity*getPeriod();
        double deltaOrientation = actualRotationalVelocity*getPeriod();

        double sinAlpha = sin(alpha+deltaOrientation);
        double cosAlpha = cos(alpha+deltaOrientation);

        x += cosAlpha*deltaTranslation;
        y += sinAlpha*deltaTranslation;
        double alpha = this->alpha+deltaOrientation;

        while (alpha > M_PI) alpha -= 2.0*M_PI;
        while (alpha < -M_PI) alpha += 2.0*M_PI;

        this->alpha = alpha;

	//cerr << "KP, " << KP << ", transSet, " << translationalVelocity << ", desiredTransl, " << desiredTranslationalVelocity << ", actualTransl, " << actualTranslationalVelocity << ", desiredRot, " << desiredRotationalVelocity << ", actualRot, " << actualRotationalVelocity << endl;
    }

    // Reset values of Motion class

    translationalMotion.setVelocity(0.0);
    rotationalMotion.setVelocity(0.0);
    translationalMotion.setAcceleration(0.0);
    rotationalMotion.setAcceleration(0.0);

    // Reset desired translation and rotation

    desiredTranslationalVelocity = 0.0;
    desiredRotationalVelocity = 0.0;

    // reset analog outputs to zero

    controlSignalLeft.write(0.0);
    controlSignalRight.write(0.0);

}
