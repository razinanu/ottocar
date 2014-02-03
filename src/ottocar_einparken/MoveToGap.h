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
	};

	MoveToGap();
	virtual ~MoveToGap();

	driveData moveToGap(sensor_msgs::LaserScan laser, float dataIRside, float dataIRback,  float distanceToGap, float voltage, int odometry);
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

	void waitForDistance(float distanceToGap, float dataIRside, int odometry);
	void driveFirstHalf(float dataIRside, int odometry);
	void driveSecondHalf(float dataIRside, int odometry);

};

#endif /* MOVETOGAP_H_ */
