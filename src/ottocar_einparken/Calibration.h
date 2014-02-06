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
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "MoveToGap.h"
#include "Parking.h"
#include "RingBuffer.h"

class Calibration
{
public:
	Calibration();
	virtual ~Calibration();
	void init();

	float voltage;
	bool driveEnable;
	ros::Publisher anglePub;
	ros::Publisher speedPub;


	void scanValues(sensor_msgs::LaserScan laser);
	void voltageValues(std_msgs::Float32 msg);
	void writeToBuffer(float value_V);
	float getAverage();

	float buffer_voltage[20];
	int bufferSize;
	int bufferPointer;

	void ir1Values(std_msgs::Float32 sensor);
	void ir2Values(std_msgs::Float32 sensor);
	void motorValues(std_msgs::Int32 sensor);
	float linearizeBack(float value);
	float linearizeSide(float value);

	float distanceBack;
	float distanceSide;

	bool motorRevolutionsSet;
	int motorRevolutionsStart;
	int motorRevolutions;

	int status;
	ros::Time start;

	ros::NodeHandle n;

	ros::Subscriber hokuyoSub;
	ros::Subscriber voltageSub;

	ros::Subscriber sensor_ir1_Subscriber;
	ros::Subscriber sensor_ir2_Subscriber;

	ros::Subscriber sensor_motor_revolutions_Subscriber;

private:

	RingBuffer* bufferBack;
};

#endif /* CALIBRATION_H_ */
