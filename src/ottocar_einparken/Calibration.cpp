/*
 * Calibration.cpp
 *
 *  Created on: 17.01.2014
 *      Author: licht
 */

#include "Calibration.h"

Calibration::Calibration()
{
	// TODO Auto-generated constructor stub
	driveEnable = false;
	status = 0;
	bufferSize = 20;
	bufferPointer = 0;
}

Calibration::~Calibration()
{
	// TODO Auto-generated destructor stub
}

void Calibration::scanValues(sensor_msgs::LaserScan laser)
{
	//pruefen, ob vor dem Auto ein Hindernis steht (nicht elegant)
	if (laser.ranges[laser.ranges.size() - (255 + 1)] < 0.3)
	{
		driveEnable = false;
	}
	else
	{
		driveEnable = true;
	}


	//Zeitmessung
	switch (status)
	{
	//vor dem 1. Karton
	case 0:
		if (laser.ranges[laser.ranges.size() - 1] < 0.3)
		{
			status = 1;
		}
		break;
	//neben dem 1. Karton
	case 1:
		if (laser.ranges[laser.ranges.size() - 1] > 0.3)
		{
//			ROS_INFO("Voltage: %2.8f", voltage);
			status = 2;
			start = ros::Time::now();
		}
		break;
	//zwischen dem 1. und 2. Karton
	case 2:
		if (laser.ranges[laser.ranges.size() - 1] < 0.3)
		{
			ROS_INFO("Voltage: %2.8f", getAverage());
			cout << "[ INFO] [-#-#-#-#-#.-#-#-#-#-]: [Pat]: Fahrzeit: " << (ros::Time::now() - start) << endl;
			status = 3;
		}
		break;

//	case 3:
//		ROS_INFO("Voltage: %2.8f", voltage);
//		status = 4;
//		break;
	default:
		//nur fuer Testzwecke
		driveEnable = false;
		break;
	}
}

void Calibration::voltageValues(std_msgs::Float32 msg)
{
	voltage = msg.data;
	writeToBuffer(voltage);
//	ROS_INFO("Voltage: %2.8f", voltage);
}

void Calibration::init()
{
	anglePub = n.advertise<std_msgs::Int8>("angle_cmd", 1);
	speedPub = n.advertise<std_msgs::Int8>("speed_cmd", 1);
	hokuyoSub = n.subscribe("/scan", 1, &Calibration::scanValues,
			this);
	voltageSub = n.subscribe("/sensor_voltage", 1, &Calibration::voltageValues, this);

	ros::Duration(1).sleep();
}

void Calibration::writeToBuffer(float value_V)
{
	buffer_voltage[bufferPointer] = value_V;
	bufferPointer++;

	if (bufferPointer >= bufferSize)
	{
		bufferPointer = 0;
	}
}

float Calibration::getAverage()
{
	float result = 0;

	for (int i = 0; i < bufferSize; i++)
	{
		result += buffer_voltage[i];
	}
	result = result/ bufferSize;

	return result;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibration");

	Calibration cal;
	try
	{
		cal.init();
	} catch (std::exception& error)
	{
		ROS_ERROR("Error: %s\n", error.what());
		return -1;
	} catch (...)
	{
		ROS_ERROR("Unknown Error\n\r");
		return -1;
	}

	MoveToGap::driveData data;
	ros::Rate loop_rate(LOOP_RATE);

	data.angle.data = 0;
	cal.anglePub.publish(data.angle);
	ros::Duration(0.4).sleep();

	data.angle.data = -90;
	cal.anglePub.publish(data.angle);
	ros::Duration(0.4).sleep();

	data.angle.data = 0;
	cal.anglePub.publish(data.angle);
	ros::Duration(0.4).sleep();

	ROS_INFO("[PAR]: Parking gestartet");


	while (ros::ok)
	{

		if (cal.driveEnable)
		{
			data.angle.data = STRAIGHTFORWARD;
			data.speed.data = -8;
		}
		else
		{
			data.angle.data = STRAIGHTFORWARD;
			data.speed.data = 0;
		}
		cal.anglePub.publish(data.angle);
		cal.speedPub.publish(data.speed);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_WARN("[PAR]: Parking beendet");
	return 0;
}

