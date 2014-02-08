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
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

class DriveIntoGap
{
public:
	// GLOBAL
	bool blinkDone;


	struct twoInts
	{
		int angle;
		int speed;
		int led1;
		int led2;
		int led3;
	};

	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int8 angle;
		std_msgs::UInt8 led0;
		std_msgs::UInt8 led1;
		std_msgs::UInt8 led2;
		std_msgs::UInt8 led3;
		std_msgs::UInt8 led4;
		std_msgs::UInt8 led5;
		std_msgs::UInt8 led6;
		std_msgs::UInt8 led7;
		std_msgs::UInt8 led8;
	};

	DriveIntoGap();
	virtual ~DriveIntoGap();

	/**
	 * [1]: angle
	 * [0]: speed
	 */
	driveData  drive(sensor_msgs::LaserScan laser, float gapSize, float distanceBack, float distanceSide, int odometry, float voltage, float distanceToGap, float dataIRside);

private:

	//float timeToDrive;
	ros::Time lastTime;
	int mode;
	int lastOdometry;
	int SPEED;

	float calculateSpeed10(float voltage);
	float drivenM(int odometry);

	driveData init();
	driveData wait1(int odometry);
	driveData back1(float gapSize, int odometry);
	driveData wait2(int odometry);
	driveData back2(const sensor_msgs::LaserScan laser, float distanceBack, int odometry, float gapSize);
	driveData wait3();
	driveData waitTurn(int odometry);
	driveData forwards(const sensor_msgs::LaserScan laser, float gapSize, int odometry);
	driveData wait4(int odometry);
	driveData backLast(float distanceBack, int odometry);

	driveData waitForDistance(float distanceToGap, int odometry);
	driveData driveFirstHalf(int odometry, float dataIRside);
	driveData driveSecondHalf(float dataIRside, int odometry);
	driveData positioning(int odometry, float gapSize);

	bool blinkerOn;
	ros::Time lastTimeBlinkerChange;
	float distanceToDrive;
	ros::Time gapBegin;

	bool ledBlinkerRechts;
	bool ledBlinkerLinks;
	bool ledScheinwerferVorneRechts;
	bool ledScheinwerferVorneLinks;
	bool ledBremse;
};

#endif /* DRIVEINTOGAP_H_ */
