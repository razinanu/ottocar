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

class Parking {

	private:

	ros::NodeHandle parkingNode;
	ros::Publisher angle_pub;
	ros::Publisher speed_pub;
	ros::Subscriber hokuyoSubscriber;



public:
	Parking();
	virtual ~Parking();
	void scanValues(const sensor_msgs::LaserScan laser);
	void init();
};

#endif /* PARKING_H_ */
