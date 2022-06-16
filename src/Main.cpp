/*
 * Main.cpp
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 24.07.2019
 *      Author: Marcel Honegger
 */

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdint.h>
#include "Thread.h"
#include "EtherCAT.h"
#include "CoE.h"
#include <BeckhoffEL1000.h>
#include <BeckhoffEL2000.h>
#include <BeckhoffEL3104.h>
#include <BeckhoffEL4732.h>
#include <BeckhoffEL5101.h>
#include "DigitalOut.h"
#include "DigitalIn.h"
#include "AnalogOut.h"
#include "AnalogIn.h"
#include "../include/landshark/Controller.h"
#include "../include/landshark/StateMachine.h"
#include "HTTPServer.h"
#include "../include/landshark/HTTPScriptRobot.h"
#include "../include/landshark/ROSInterface.h"
#include "Timer.h"

using namespace std;

int main(int argc, char* argv[]) {

    // create peripheral objects

    EtherCAT etherCAT("192.168.20.10");  // create EtherCAT object with given interface
    CoE coe(etherCAT, 0.002);            // create CANopen over EtherCAT driver

    BeckhoffEL5101 encoderCounterLeft(etherCAT, coe, 0xFFFF); // create EL5101 driver
    BeckhoffEL5101 encoderCounterRight(etherCAT, coe, 0xFFFE); // create EL5101 driver
    BeckhoffEL1000 beckhoffEL1104(etherCAT, coe, 0xFFFD); // create EL1000 driver
    BeckhoffEL2000 beckhoffEL2004(etherCAT, coe, 0xFFFC, 1000); // create EL2004 driver
    BeckhoffEL3104 beckhoffEL3104(etherCAT, coe, 0xFFFB); // create EL3104 driver
    BeckhoffEL4732 beckhoffEL4732(etherCAT, coe, 0xFFFA, 0.005); // create EL4732 driver
    BeckhoffEL1000 beckhoffEL1124(etherCAT, coe, 0xFFF9); // create EL1000 driver
    BeckhoffEL2000 beckhoffEL2124(etherCAT, coe, 0xFFF8, 1000); // create EL2004 driver

    // create the 24V digital outputs
    DigitalOut warnLamp(beckhoffEL2004, 0);  // to turn warn lamp on
    DigitalOut stopOverSickFront(beckhoffEL2004, 1);  // input signal for SICK sensor, front
    DigitalOut stopOverSickRear(beckhoffEL2004, 2);  // input signal for SICK sensor, back
    DigitalOut digitalOut24V4(beckhoffEL2004, 3);
    // create the 24V digital inputs
    DigitalIn powerEnabled(beckhoffEL1104, 0);  // power enabled from safety relais
    DigitalIn sickOutputFront(beckhoffEL1104, 1);  // output signal from SICK sensor, front
    DigitalIn sickOutputRear(beckhoffEL1104, 2);  // output signal from SICK sensor, back
    DigitalIn digitalIn24V4(beckhoffEL1104, 3);
    // create the +-10V analog outputs
    AnalogOut controlSignalLeft(beckhoffEL4732, 0);  // control signal motor left
    AnalogOut controlSignalRight(beckhoffEL4732, 1);  // control signal motor right
    // create the +10V analog inputs
    AnalogIn voltageBatteryFront(beckhoffEL3104, 0);  // voltage of front battery
    AnalogIn voltageBatteryMiddle(beckhoffEL3104, 1);  // voltage of middle battery
    AnalogIn voltageBatteryRear(beckhoffEL3104, 2);  // voltage of rear battery
    AnalogIn analogIn10V4(beckhoffEL3104, 3);
    // create the 5V digital outputs
    DigitalOut backwardLeft(beckhoffEL2124, 0);  // 1 -> backward left Motor (0 -> forward)
    DigitalOut forwardLeft(beckhoffEL2124, 1);  // 1 -> forward left Motor (0 -> backward)
    DigitalOut backwardRight(beckhoffEL2124, 2);  // 1 -> backward right Motor (0 -> forward)
    DigitalOut forwardRight(beckhoffEL2124, 3);  // 1 -> forward right Motor (0 -> backward)
    // create the 5V digital inputs
    DigitalIn brake(beckhoffEL1124, 0);  // limit switch on brake mechanism
    DigitalIn digitalIn5V2(beckhoffEL1124, 1);
    DigitalIn digitalIn5V3(beckhoffEL1124, 2);
    DigitalIn digitalIn5V4(beckhoffEL1124, 3);

    // set gains for analog inputs
    voltageBatteryFront.setGain(20.0f);  // gain for analog input with voltage divider of front battery
    voltageBatteryMiddle.setGain(20.0f);  // gain for analog input with voltage divider of middle battery
    voltageBatteryRear.setGain(20.0f);  // gain for analog input with voltage divider of rear battery

    // set gain for analag outputs
    controlSignalLeft.setGain(0.1f); // gain for analog output (10V)
    controlSignalRight.setGain(0.1f); // gain for analog output (10V)

    // create controller objects
    Controller controller(controlSignalLeft, controlSignalRight, backwardLeft, forwardLeft, backwardRight, forwardRight, encoderCounterLeft, encoderCounterRight);
    controller.setPriority(RealtimeThread::RT_MAX_PRIORITY-20);
    controller.setPeriod(0.002);

    StateMachine stateMachine(controller, warnLamp, stopOverSickFront, stopOverSickRear, powerEnabled, brake, voltageBatteryFront, voltageBatteryMiddle, voltageBatteryRear);
    stateMachine.setPriority(RealtimeThread::RT_MAX_PRIORITY-21);
    stateMachine.setPeriod(0.010);
    stateMachine.start();
    
    // create communication objects

    //HTTPServer httpServer(80, "html/", "index.html");
    HTTPServer httpServer(80, "/home/landshark/ros_ws/src/landshark/html/", "index.html");
    httpServer.add("robot", new HTTPScriptRobot(controller, stateMachine));
    httpServer.start();

    ROSInterface rosInterface(argc, argv, controller, stateMachine);
    rosInterface.start();
    
    // enter command loop

    char command[128];
    double KP;

    //controller.start();

    cerr << "init done" << endl;

    do {


        cerr << "controller> ";
        cin >> command;

        if (strcmp(command, "help") == 0) {

            cerr << "available commands:" << endl;
            cerr << " - help" << endl;
            cerr << " - info" << endl;
            cerr << " - OFF" << endl;
            cerr << " - KP" << endl;
            cerr << " - quit" << endl;

        } else if (strcmp(command, "info") == 0) {

            cerr << "BatteryVoltages are: ";
            cerr << voltageBatteryFront << " - ";
            cerr << voltageBatteryMiddle << " - ";
            cerr << voltageBatteryRear << " - " << endl;

	    cerr << "StateMachine: " << stateMachine.getState() << endl;
	    cerr << "DOUT stopOverSickFront: " << stopOverSickFront << endl;
	    cerr << "DOUT stopOverSickRear: " << stopOverSickRear << endl;
	    cerr << "DIN sickOutputFront: " << sickOutputFront << endl;
	    cerr << "DIN sickOutputRear: " << sickOutputRear << endl;
	    cerr << "DIN brake: " << brake << endl;



        } else if (strcmp(command, "OFF") == 0) {

            cout <<  "Setting State to OFF: ";
            stateMachine.setState(StateMachine::OFF);

        } else if (strcmp(command, "KP") == 0) {

            cin >> KP;
            cout <<  "Setting KP to " << KP << "./";

            controller.setKP(KP);

        } else if (strcmp(command, "quit") != 0) {

           // cout << "don't know... " << command << "./" << endl;
        }

    } while (strcmp(command, "quit") != 0);

    return EXIT_SUCCESS;
}
