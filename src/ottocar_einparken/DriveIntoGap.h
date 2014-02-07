/*
 * DriveIntoGap.h
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#ifndef DRIVEINTOGAP_H_
#define DRIVEINTOGAP_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

class DriveIntoGap
{
public:

	struct twoInts
	{
		int angle;
		int speed;
		int led1;
		int led2;
		int led3;
	};

	DriveIntoGap();
	virtual ~DriveIntoGap();

	/**
	 * [1]: angle
	 * [0]: speed
	 */
	twoInts  drive(sensor_msgs::LaserScan laser, float gapSize, float distanceBack, float distanceSide, int odometry, float voltage);

private:

	//float timeToDrive;
	ros::Time lastTime;
	int mode;
	int lastOdometry;
	int SPEED;

	float calculateSpeed10(float voltage);
	float drivenM(int odometry);

	twoInts init();
	twoInts wait1(int odometry);
	twoInts back1(float gapSize, int odometry);
	twoInts wait2(int odometry);
	twoInts back2(const sensor_msgs::LaserScan laser, float distanceBack, int odometry, float gapSize);
	twoInts wait3();
	twoInts waitTurn(int odometry);
	twoInts forwards(const sensor_msgs::LaserScan laser, float gapSize, int odometry);
	twoInts wait4(int odometry);
	twoInts backLast(float distanceBack, int odometry);
};

#endif /* DRIVEINTOGAP_H_ */
