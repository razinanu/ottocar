/*
 * Calibration.h
 *
 *  Created on: 17.01.2014
 *      Author: licht
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "ros/ros.h"
#include "ConstPark.h"
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "MoveToGap.h"

class Calibration
{
public:
	Calibration();
	virtual ~Calibration();
	void init();

	bool driveEnable;
	ros::Publisher anglePub;
	ros::Publisher speedPub;

private:
	void scanValues(sensor_msgs::LaserScan laser);
	void voltageValues(std_msgs::Float32 msg);
	void writeToBuffer(float value_V);
	float getAverage();

	float buffer_voltage[20];
	int bufferSize;
	int bufferPointer;

	float voltage;
	int status;
	ros::Time start;

	ros::NodeHandle n;

	ros::Subscriber hokuyoSub;
	ros::Subscriber voltageSub;
};

#endif /* CALIBRATION_H_ */
