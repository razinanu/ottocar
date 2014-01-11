/*
 * Parking.cpp
 *
 *  Created on: Dec 6, 2013
 *  Author: Razi Ghassemi
 */

#include "Parking.h"

Parking::Parking() :
		GapCalculator_(true), ParallelController_(true), PositionController_(
				true), ParkingController_(false)
{

}

Parking::~Parking()
{
	// TODO Auto-generated destructor stub
}

void Parking::scanValues(sensor_msgs::LaserScan laser)
{
	//todo each class create a COPY of Laser data, if it changes the data!

	for (unsigned int i = 0; i < laser.ranges.size(); i++)
	{
		if (laser.ranges[i] == INFINITY || isnan(laser.ranges[i]))
		{
			laser.ranges[i] = BIGRANGE;
		}
	}

	//as long as the best Gap was found
	if (GapCalculator_)
	{
		gapcal.LaserScanGapCal(laser);
	}

	if (ParallelController_)
	{
		parallel.laserScanParallel(laser);
	}
	//Whether the car is at correct Position to park.
	//in case that car is at correct Position, PrkingController must be set to true
	if (PositionController_)
	{
		position.LaserScanPosition(laser);
	}
	//PrkingController set to true, if the car is at correct position to park
	ParkingController_ = true;
	if (ParkingController_)
	{
		float size = 60.0;
		parkControll.LaserScanParkControll(laser);
		driveIntoGap.drive(laser, size);

	}

}

float Parking::linearlize(float value)
{
	float error = 40.0;

// drei geraden zur annaeherung an die funktion
	if (value > 1.25 && value < 4.0)
	{
		return (1 / (-1.45 / 6)) * value + (432 / 29);
	}
	else if (value > 0.8)
	{
		return (1 / (-1.45 / 6)) * value + (432 / 29);
	}
	else if (value > 0.3)
	{
		return (1 / (-0.075)) * value + (80 / 3);
	}
	else
	{
		ROS_INFO("[PAR]: linearlize of %f", value);
		return error;
	}
}

void Parking::ir1Values(std_msgs::Float32 sensor)
{
	this->distanceBack = linearlize(sensor.data);
	ROS_INFO("[PAR]: IR1: (V,%f) and (D,%f)", sensor.data, distanceBack);
}

void Parking::ir2Values(const std_msgs::Float32 sensor)
{
	this->distanceSide = linearlize(sensor.data);
	ROS_INFO("[PAR]: IR2: (V,%f) and (D,%f)", sensor.data, distanceSide);
}

void Parking::init()
{
	angle_pub = parkingNode.advertise<std_msgs::Int8>("angle_cmd", 1);
	speed_pub = parkingNode.advertise<std_msgs::Int8>("speed_cmd", 1);
	hokuyoSubscriber = parkingNode.subscribe("/scan", 1, &Parking::scanValues,
			this);
	sensor_ir1_Subscriber = parkingNode.subscribe("/sensor_IR1", 1,
			&Parking::ir1Values, this);
	sensor_ir2_Subscriber = parkingNode.subscribe("/sensor_IR2", 1,
			&Parking::ir2Values, this);

	ros::Duration(1).sleep();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");
	Parking park;
	try
	{
		park.init();
	} catch (std::exception& error)
	{
		ROS_ERROR("Error: %s\n", error.what());
		return -1;
	} catch (...)
	{
		ROS_ERROR("Unknown Error\n\r");
		return -1;
	}

	ros::Rate loop_rate(LOOP_RATE);

	while (ros::ok)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
