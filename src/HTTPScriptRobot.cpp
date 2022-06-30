/*
 * HTTPScriptRobot.cpp
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 10.09.2019
 *      Author: Marcel Honegger
 */

#include <iomanip>
#include <stdint.h>
#include "Thread.h"
#include "../include/landshark/Controller.h"
#include "../include/landshark/StateMachine.h"
#include "../include/landshark/HTTPScriptRobot.h"

using namespace std;

template <class T> inline string type2String(const T& t) {

    stringstream out;
    out << fixed << setprecision(3) << t;

    return out.str();
}

inline string int2String(int32_t i) {

    char buffer[32];
    sprintf(buffer, "%d", i);

    return string(buffer);
}

inline string float2String(float f, uint16_t d) {

    char buffer[32];
    if (d == 0) sprintf(buffer, "%.0f", f);
    else if (d == 1) sprintf(buffer, "%.1f", f);
    else if (d == 2) sprintf(buffer, "%.2f", f);
    else if (d == 3) sprintf(buffer, "%.3f", f);
    else if (d == 4) sprintf(buffer, "%.4f", f);
    else if (d == 5) sprintf(buffer, "%.5f", f);
    else if (d == 6) sprintf(buffer, "%.6f", f);
    else if (d == 7) sprintf(buffer, "%.7f", f);
    else if (d == 8) sprintf(buffer, "%.8f", f);
    else sprintf(buffer, "%.9f", f);

    return string(buffer);
}

HTTPScriptRobot::HTTPScriptRobot(Controller& controller, StateMachine& stateMachine) : controller(controller), stateMachine(stateMachine) {

    // Inuitialize local values
    previousWatchdogState = false;

}

HTTPScriptRobot::~HTTPScriptRobot() {}

string HTTPScriptRobot::call(vector<string> names, vector<string> values) {

    // parse and process given arguments

    for (uint16_t i = 0; i < min(names.size(), values.size()); i++) {

        if (names[i].compare("state") == 0) {

            if (values[i].compare("off") == 0) stateMachine.setState(StateMachine::OFF);
            else if (values[i].compare("on") == 0) stateMachine.setState(StateMachine::ON);
            else if (values[i].compare("manual") == 0) stateMachine.setState(StateMachine::MANUAL);
            else if (values[i].compare("remote") == 0) stateMachine.setState(StateMachine::REMOTE);
            else if (values[i].compare("track") == 0) ros::NodeHandle.setParam("/start_tracking", true); //Attention link to ROSInterface for NodeHandle
            else if (values[i].compare("follow") == 0) stateMachine.setState(StateMachine::REMOTE);
            else if (values[i].compare("home") == 0) ros::NodeHandle.setParam("/nav_dir", 1);
            else if (values[i].compare("goal") == 0) ros::NodeHandle.setParam("/nav_dir", 2);

            translationalVelocity = 0.0;
            rotationalVelocity = 0.0;

        } else if (names[i].compare("translation") == 0) {

            translationalVelocity = strtof(values[i].c_str(), NULL);

        } else if (names[i].compare("rotation") == 0) {

            rotationalVelocity = strtof(values[i].c_str(), NULL);

        } else if (names[i].compare("watchdog") == 0) {  //

            int watchdogState = strtol(values[i].c_str(), NULL, 0);

            if (previousWatchdogState != watchdogState) {

                stateMachine.setManualRotationalVelocity(-rotationalVelocity);
                stateMachine.setManualTranslationalVelocity(translationalVelocity);

            } else {

                stateMachine.setManualRotationalVelocity(0.0);
                stateMachine.setManualTranslationalVelocity(0.0);

            }

            previousWatchdogState = watchdogState;

        } else if (names[i].compare("KP") == 0) {

            KP = strtof(values[i].c_str(), NULL);
            controller.setKP(static_cast<double>(KP));
            controller.setX(0.0);
            controller.setY(0.0);
            controller.setAlpha(0.0);
        }
    }

    // create response

    string response;

    response += "  <time><int>"+int2String(Thread::currentTimeMillis())+"</int></time>\r\n";
    response += "  <stateMachine>\r\n";
    response += "    <state><int>"+int2String(stateMachine.getState())+"</int></state>\r\n";
    response += "    <inputVoltage><float>"+float2String(stateMachine.getInputVoltage(), 1)+"</float></inputVoltage>\r\n";
    response += "  </stateMachine>\r\n";
    response += "  <controller>\r\n";
    response += "    <x><float>"+float2String(static_cast<float>(controller.getX()), 2)+"</float></x>\r\n";
    response += "    <y><float>"+float2String(static_cast<float>(controller.getY()), 2)+"</float></y>\r\n";
    response += "    <alpha><float>"+float2String(static_cast<float>(controller.getAlpha()), 2)+"</float></alpha>\r\n";
    response += "  </controller>\r\n";

    return response;
}
