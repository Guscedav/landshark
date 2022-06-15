/*
 * HTTPScriptRobot.h
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 10.09.2019
 *      Author: Marcel Honegger
 */

#ifndef HTTP_SCRIPT_ROBOT_H_
#define HTTP_SCRIPT_ROBOT_H_

#include <cstdlib>
#include <string>
#include <vector>
#include "HTTPScript.h"

using namespace std;

class Controller;
class StateMachine;

class HTTPScriptRobot : public HTTPScript {

    public:

                    HTTPScriptRobot(Controller& controller, StateMachine& stateMachine);
        virtual     ~HTTPScriptRobot();
        string      call(vector<string> names, vector<string> values);

    private:

        Controller&         controller;
        StateMachine&       stateMachine;
        float               translationalVelocity;
        float               rotationalVelocity;
        int                 previousWatchdogState;
        float               KP;

};

#endif /* HTTP_SCRIPT_ROBOT_H_ */
