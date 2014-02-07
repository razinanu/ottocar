/*
 * DriveToGap.h
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#ifndef MOVETOGAP_H_
#define MOVETOGAP_H_

#include "ConstPark.h"
#include "ParallelController.h"
#include "Parking.h"

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class MoveToGap
{
public:
	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int8 angle;
		std_msgs::UInt8 led1;
		std_msgs::UInt8 led2;
		std_msgs::UInt8 led3;
	};

	MoveToGap();
	virtual ~MoveToGap();

	driveData moveToGap(sensor_msgs::LaserScan laser, float dataIRside, float dataIRback,  float distanceToGap, float voltage, int odometry, float gapSize);
private:
	bool cartonSeen;
	bool distanceGot;
	ros::Time gapBegin;
	ros::Time distanceFound;
	ros::Time lastTime;
	int mode;
	float distanceToDrive;

	int lastOdometry;
	float drivenM(int odometry);

	void waitForDistance(float, int);
	void driveFirstHalf(int, float);
	void driveSecondHalf(float, int);
	MoveToGap::driveData positioning(int, int, int, float);


};

#endif /* MOVETOGAP_H_ */
