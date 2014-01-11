/*
 * Parking.h
 *
 *  Created on: Dec 6, 2013
 *  Author: Razi Ghassemi
 */

#ifndef PARKING_H_
#define PARKING_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "GapCalculator.h"
#include "ConstPark.h"
#include "ParallelController.h"
#include "ParkingController.h"
#include "PositionController.h"


#include "MoveToGap.h"

class Parking {

	private:

	ros::NodeHandle parkingNode;
	ros::Subscriber hokuyoSubscriber;

	ros::Subscriber backIRSubscriber;
	DriveIntoGap intoGap;

	float currentInfraredValue;	//voltage

public:
	Parking();
	virtual ~Parking();
	void scanValues(const sensor_msgs::LaserScan laser);
	void getIRData(const std_msgs::Float32 infrared);
	void init();
	bool GapCalculator_, ParallelController_, PositionController_, ParkingController_;
	GapCalculator gapcal;
	ParallelController parallel;
	PositionController position;
	//ParkingController parkControll;

	ros::Publisher angle_pub;
	ros::Publisher speed_pub;

};

#endif /* PARKING_H_ */
