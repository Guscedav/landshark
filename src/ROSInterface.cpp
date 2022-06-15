/*
 * ROSInterface.cpp
 * Copyright (c) 2019, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.09.2019
 *      Author: Marcel Honegger
 */

#include "Controller.h"
#include "StateMachine.h"
#include "ROSInterface.h"
#include <tf/transform_broadcaster.h>


using namespace std;

ROSInterface::ROSInterface(int argc, char* argv[], Controller& controller, StateMachine& stateMachine) : controller(controller), stateMachine(stateMachine) {

    // initialize ROS components
    
    ros::init(argc, argv, "landshark_mobile_robot"); // initialize the ROS system, node-name
    ros::NodeHandle nodeHandle; // establish this program as a ROS node
    
    ROS_INFO_STREAM("creating remoteVelocitySubscriber..."); // send some output as a log message
    
    remoteVelocitySubscriber = nodeHandle.subscribe("robot/cmd_vel", 1000, &ROSInterface::robotVelocityReceived, this);
    
    ROS_INFO_STREAM("creating odometryPublisher...");
    
    odometryPublisher = nodeHandle.advertise<nav_msgs::Odometry>("robot/odom", 1000);

    ROS_INFO_STREAM("creating actualVelocityPublisher...");

    actualVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("robot/actualVelocity", 1000);

    ROS_INFO_STREAM("creating manualVelocityPublisher...");

    manualVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("robot/manualVelocity", 1000);

    ROS_INFO_STREAM("creating desiredlVelocityPublisher...");

    desiredVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("robot/desiredVelocity", 1000);


//    tf::TransformBroadcaster _odometry_broadcaster;
    ROS_INFO_STREAM("creating serviceServer...");
    
    serviceServer = nodeHandle.advertiseService("robot/set_master_control", &ROSInterface::serviceReceived, this);

}

ROSInterface::~ROSInterface() {}

// Callbackfunction: Receives ROS message and sets values

void ROSInterface::robotVelocityReceived(const geometry_msgs::Twist& message) {
    
    //ROS_INFO_STREAM(std::setprecision(3) << std::fixed << "velocity received: linear=" << message.linear.x << " angular=" << message.angular.z);
    
    stateMachine.setRemoteTranslationalVelocity(message.linear.x);
    stateMachine.setRemoteRotationalVelocity(message.angular.z);
}

bool ROSInterface::serviceReceived(landshark::TurnOnOff::Request &request, landshark::TurnOnOff::Response &response) {

    Timer lastTimeServerCalled;
    lastTimeServerCalled.start();
    lastTimeServerCalled.reset();

    if (request.turn_on && !request.turn_off) {
        
        stateMachine.setState(StateMachine::REMOTE);
        
    } else {
        
        stateMachine.setState(StateMachine::OFF);
    }
    
    while (lastTimeServerCalled < 50) {}

    response.is_on = stateMachine.getState() == StateMachine::REMOTE;
    
    return true;
}

void ROSInterface::run() {

    ros::Rate rate(20); // loop rate given in [Hz]
    tf::TransformBroadcaster _odometry_broadcaster;  
    while (ros::ok()) {
        
        ros::spinOnce(); // used by ROS for subscriber
        
        double x = controller.getX();
        double y = controller.getY();
        double alpha = controller.getAlpha();

        double actualTranslation = controller.getTranslationalVelocity();
        double actualRotation = controller.getRotationalVelocity();

        float manualTranslation = stateMachine.getManualTranslationalVelocity();
        float manualRotation = stateMachine.getManualRotationalVelocity();

        double desiredTranslation = controller.getDesiredTranslationalVelocity();
        double desiredRotation = controller.getDesiredRotationalVelocity();
        
	ros::Time current_time;
	current_time = ros::Time::now();

        nav_msgs::Odometry odometryMessage;
	    odometryMessage.header.stamp = current_time;
        odometryMessage.header.frame_id = "odom";
        odometryMessage.child_frame_id = "base_link";
        odometryMessage.pose.pose.position.x = x;
        odometryMessage.pose.pose.position.y = y;
        odometryMessage.pose.pose.orientation.x = 0.0;
        odometryMessage.pose.pose.orientation.y = 0.0;
        odometryMessage.pose.pose.orientation.z = sin(alpha/2.0);
        odometryMessage.pose.pose.orientation.w = cos(alpha/2.0);
//        odometryMessage.twist.twist.linear.x = actualTranslation;
//        odometryMessage.twist.twist.linear.y = 0;
//        odometryMessage.twist.twist.linear.z = 0;
//        odometryMessage.twist.twist.angular.x = 0;
//        odometryMessage.twist.twist.angular.y = 0;
//        odometryMessage.twist.twist.angular.z = actualRotation;

        geometry_msgs::Twist actualVelocityMessage;
        actualVelocityMessage.linear.x = actualTranslation;
        actualVelocityMessage.angular.z = actualRotation;
        
        geometry_msgs::Twist manualVelocityMessage;
        manualVelocityMessage.linear.x = static_cast<double>(manualTranslation);
        manualVelocityMessage.angular.z = static_cast<double>(manualRotation);

        geometry_msgs::Twist desiredVelocityMessage;
        desiredVelocityMessage.linear.x = desiredTranslation;
        desiredVelocityMessage.angular.z = desiredRotation;

        odometryPublisher.publish(odometryMessage);
        actualVelocityPublisher.publish(actualVelocityMessage);
        manualVelocityPublisher.publish(manualVelocityMessage);
        desiredVelocityPublisher.publish(desiredVelocityMessage);




	tf::Vector3 trans(odometryMessage.pose.pose.position.x, 
		    odometryMessage.pose.pose.position.y,
		    odometryMessage.pose.pose.position.z);
  	tf::Quaternion quat(odometryMessage.pose.pose.orientation.x, 
		      odometryMessage.pose.pose.orientation.y,
		      odometryMessage.pose.pose.orientation.z,
		      odometryMessage.pose.pose.orientation.w);
		      
  	tf::StampedTransform odom_transform;
        odom_transform = tf::StampedTransform(tf::Transform(quat,trans), odometryMessage.header.stamp, "odom", "base_link");
	_odometry_broadcaster.sendTransform(odom_transform);

        //ROS_INFO_STREAM("publish pose: x=" << x << "  y=" << y << "  alpha=" << alpha);

        // Debug information on debug message stream
//        ROS_DEBUG_STREAM("transSpeed is: " << actualTranslation << "rotSpeed is: " << actualRotation);
        //ROS_DEBUG_STREAM(controller.getPhaseVoltageLeft() << ";" << controller.getPhaseVoltageRight() << ";" << controller.getSpeedLeft() << ";" << controller.getSpeedRight());
	ROS_INFO_STREAM(controller.getDesiredTranslationalVelocity() << ";" << controller.getTranslationalVelocity() << ";" << controller.getDesiredRotationalVelocity() << ";" << controller.getRotationalVelocity());
ROS_DEBUG_STREAM("phase left: " << controller.getPhaseVoltageLeft());


        rate.sleep();
    }
}
