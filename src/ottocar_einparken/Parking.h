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
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <iostream>
#include "GapCalculator.h"
#include "ConstPark.h"
#include "ParallelController.h"
#include "DriveIntoGap.h"

#include "RingBuffer.h"

class Parking
{

private:

	ros::NodeHandle parkingNode;
	ros::Subscriber hokuyoSubscriber;
	ros::Subscriber sensor_ir1_Subscriber;
	ros::Subscriber sensor_ir2_Subscriber;
	ros::Subscriber sensor_voltage;
	ros::Subscriber sensor_motor_revolutions_Subscriber;
	ros::Subscriber imu_dataRaw_Subscriber;

	ros::Time lastImuTime;


public:
	Parking();
	virtual ~Parking();
	void scanValues(const sensor_msgs::LaserScan laser);
	void init();
	bool GapCalculator_, ParallelController_, ParkingController_;
	GapCalculator gapcal;
	ParallelController parallel;
	DriveIntoGap driveIntoGap;

	ros::Publisher angle_pub;
	ros::Publisher speed_pub;
	ros::Publisher led_pub;
	std_msgs::UInt8 msg_led;
	ros::Time lastTime;

	int count;
	void finishedParkLed();
	void allLightsOff();
	void orientation(const sensor_msgs::Imu imu);
	void ir1Values(const std_msgs::Float32 sensor);
	void ir2Values(const std_msgs::Float32 sensor);
	void voltageValues(std_msgs::Float32 msg);
	void motorValues(std_msgs::Int32 sensor);
	float linearizeBack(float sensor);
	float linearizeSide(float value);

	float intoGapAngle;
	float intoGapSpeed;
	float distanceBack;
	float distanceSide;
	float voltage;
	sensor_msgs::LaserScan g_laser;

	double gapDistance;
	ros::Time lastLaserscanTime;

	int motorRevolutions;
	RingBuffer* bufferBack;
	RingBuffer* bufferSide;

};

void driveToGap();
void driveIntoGap();
void shake();
int init();

#endif /* PARKING_H_ */
