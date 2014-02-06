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
	motorRevolutions = 0;
	motorRevolutionsSet = false;
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

	if (abs (abs(motorRevolutions) - abs(motorRevolutionsStart)) > 3200)
	{
		driveEnable = false;
		ROS_INFO("[CAL]: motorRevolutions: %d", abs (abs(motorRevolutions) - abs(motorRevolutionsStart)));
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
			ROS_INFO("[CAL]: Messung start");
			status = 2;
			start = ros::Time::now();
			motorRevolutionsStart = motorRevolutions;
		}
		break;
	//zwischen dem 1. und 2. Karton
	case 2:
		if (laser.ranges[laser.ranges.size() - 1] < 0.3)
		{
			ROS_INFO("[CAL]: Voltage: %2.8f", getAverage());
			cout << "[ INFO] [-#-#-#-#-#.-#-#-#-#-]: [CAL]: Fahrzeit: " << (ros::Time::now() - start) << endl;
			ROS_INFO("[CAL]: motorRevolutions: %d", abs (abs(motorRevolutions) - abs(motorRevolutionsStart)));
			status = 3;
		}
		break;

	default:
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

float Calibration::linearizeBack(float value)
{
	if (value > 0.1)
	{
		return 0.1194 / (value + 0.028);
	}
	else
	{
		return 0.4;
	}


//	float error = 40.0;
//
//// drei geraden zur annaeherung an die funktion
//	if (value > 1.25 && value < 4.0)
//	{
//		return (1 / (-1.45 / 6)) * value + (432 / 29);
//	}
//	else if (value > 0.8)
//	{
//		return (1 / (-1.45 / 6)) * value + (432 / 29);
//	}
//	else if (value > 0.3)
//	{
//		return (1 / (-0.075)) * value + (80 / 3);
//	}
//	else
//	{
////		ROS_INFO("[PAR]: linearlize of %f", value);
//		return error;
//	}
}

float Calibration::linearizeSide(float value)
{
	if (value > 0.1)
	{
		return 0.1128 / (value - 0.124);
	}
	else
	{
		return 0.4;
	}
}

void Calibration::ir1Values(std_msgs::Float32 sensor)
{
	this->distanceBack = linearizeBack(sensor.data);

}

void Calibration::ir2Values(const std_msgs::Float32 sensor)
{
	this->distanceSide = linearizeSide(sensor.data);

}

void Calibration::motorValues(const std_msgs::Int32 sensor)
{
	if (!motorRevolutionsSet)
	{
		motorRevolutionsStart = sensor.data;
		ROS_INFO("[CAL]: motorRevolutionsStart: %d", motorRevolutionsStart);
		motorRevolutionsSet = true;
	}

	motorRevolutions = sensor.data;
}

void Calibration::init()
{
	anglePub = n.advertise<std_msgs::Int8>("angle_cmd", 1);
	speedPub = n.advertise<std_msgs::Int8>("speed_cmd", 1);
	hokuyoSub = n.subscribe("/scan", 1, &Calibration::scanValues,
			this);
	voltageSub = n.subscribe("/sensor_voltage", 1, &Calibration::voltageValues, this);

	sensor_ir1_Subscriber = n.subscribe("/sensor_IR1", 1,
			&Calibration::ir1Values, this);
	sensor_ir2_Subscriber = n.subscribe("/sensor_IR2", 1,
			&Calibration::ir2Values, this);

	sensor_motor_revolutions_Subscriber = n.subscribe("/sensor_motor_revolutions", 1,
			&Calibration::motorValues, this);

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

	ROS_INFO("[CAL]: Calibration gestartet");

	while (ros::ok)
	{

//		ROS_INFO("[CAL]: distanceBack: %2.4f ",cal.distanceBack);
//		ROS_INFO("[CAL]: distanceSide: %2.4f ",cal.distanceSide);

		if (cal.driveEnable)
		{
			data.angle.data = STRAIGHTFORWARD;
			data.speed.data = 8;
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

	ROS_WARN("[CAL]: Calibration beendet");
	return 0;
}

