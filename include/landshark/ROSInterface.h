/*
 * ROSInterface.h
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.09.2019
 *      Author: Marcel Honegger
 */

#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

#include <cstdlib>
#include <stdint.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <landshark/TurnOnOff.h>
#include "Thread.h"
#include "Timer.h"
#include <tf/transform_broadcaster.h>

class Controller;
class StateMachine;

class ROSInterface : public Thread {

    public:

                    ROSInterface(int argc, char* argv[], Controller& controller, StateMachine& stateMachine);
        virtual     ~ROSInterface();

    private:

        Controller&         controller;
        StateMachine&       stateMachine;
        ros::Subscriber     remoteVelocitySubscriber;
        ros::Publisher      odometryPublisher;
        ros::Publisher      actualVelocityPublisher;
        ros::Publisher      manualVelocityPublisher;
        ros::Publisher      desiredVelocityPublisher;

        ros::ServiceServer  serviceServer;

        Timer       lastTimeServerCalled;
        
        void        robotVelocityReceived(const geometry_msgs::Twist& message);
        bool        serviceReceived(landshark::TurnOnOff::Request &request, landshark::TurnOnOff::Response &response);
        void        run();
};

#endif /* ROS_INTERFACE_H_ */
