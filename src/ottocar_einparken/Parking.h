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
#include "std_msgs/Bool.h"

#include <sstream>
#include <iostream>
#include "GapCalculator.h"
#include "ConstPark.h"
#include "ParallelController.h"
#include "DriveIntoGap.h"

#include "RingBuffer.h"

/**\brief
 *The main parking class
 * @note       This class represents RosNode.
 * @see        RosNode
 * @see        http://wiki.ros.org/rosnode**/

class Parking
{

private:

	ros::NodeHandle parkingNode;
	/**\brief subscribe Scan data **/
	ros::Subscriber hokuyoSubscriber;
	/**\brief subscribe infrared sensor **/
	ros::Subscriber sensor_ir1_Subscriber;
	/**\brief subscribe infrared sensor **/
	ros::Subscriber sensor_ir2_Subscriber;
	/**\brief subscribe battery voltage **/
	ros::Subscriber sensor_voltage;
	ros::Subscriber sensor_motor_revolutions_Subscriber;
	ros::Subscriber imu_dataRaw_Subscriber;
	ros::Subscriber buttonOnCar_Subscriber;

	ros::Time lastImuTime;


public:
	Parking();
	virtual ~Parking();
	/**\brief this function gives the laser scanner data to GapCalculator.cpp and ParallelContorller.cpp
		 * \param sensor_msgs::LaserScan& msg the laser scanner data
		 * **/
	void scanValues(const sensor_msgs::LaserScan laser);
	void init();
	void initButton();
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
	void ir1Values(const std_msgs::Float32 sensor);
	void ir2Values(const std_msgs::Float32 sensor);
	void voltageValues(std_msgs::Float32 msg);
	void motorValues(std_msgs::Int32 sensor);
	/**\brief linearize the infrared sensor data
			 * \param float sensor the infrared sensor data
			 * **/
	float linearizeBack(float sensor);
	/**\brief linearize the infrared sensor data
			 * \param float value the infrared sensor data
			 * **/
	float linearizeSide(float value);
	void buttonPressed(std_msgs::Bool msg);

	float intoGapAngle;
	float intoGapSpeed;
	float distanceBack;
	float distanceSide;
	float voltage;
	sensor_msgs::LaserScan g_laser;

	double gapDistance;
	ros::Time lastLaserscanTime;
	bool button;

	int motorRevolutions;
	RingBuffer* bufferBack;
	RingBuffer* bufferSide;

};

void driveToGap();
void driveIntoGap();
void shake();
int init();

#endif /* PARKING_H_ */
