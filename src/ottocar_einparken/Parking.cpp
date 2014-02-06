/*
 * Parking.cpp
 *
 *  Created on: Dec 6, 2013
 *  Author: Razi Ghassemi
 */

#include "Parking.h"

Parking::Parking() :
		GapCalculator_(true), ParallelController_(true), ParkingController_(false)
{
	intoGapAngle = 0;
	intoGapSpeed = 0;
	distanceBack = -1;
	distanceSide = -1;

	motorRevolutions = 0;
	voltage = -1;

	bufferBack = new RingBuffer();
	bufferSide = new RingBuffer();
}

Parking::~Parking()
{
	bufferBack->~RingBuffer();
	bufferSide->~RingBuffer();
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

	g_laser = laser;
}

float Parking::linearizeBack(float value)
{
	float result = 0.1194 / (value + 0.028);

	if (result > 0 && result <= 0.4)
	{
		return result;
	}
	else
	{
		return 0.4;
	}
}

float Parking::linearizeSide(float value)
{
	float result = 0.1128 / (value - 0.124);

	if (result > 0 && result <= 0.4)
	{
		return result;
	}
	else
	{
		return 0.4;
	}
}

void Parking::ir1Values(std_msgs::Float32 sensor)
{
	bufferSide->insert(linearizeBack(sensor.data));
}

void Parking::ir2Values(const std_msgs::Float32 sensor)
{
	bufferBack->insert(linearizeSide(sensor.data));
}

void Parking::voltageValues(std_msgs::Float32 msg)
{
	voltage = msg.data;
	//todo Klasse Ringpuffer nutzen
}

void Parking::motorValues(const std_msgs::Int32 sensor)
{
	motorRevolutions = sensor.data;
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
	sensor_voltage = parkingNode.subscribe("/sensor_voltage", 1,
			&Parking::voltageValues, this);
	sensor_motor_revolutions_Subscriber = parkingNode.subscribe(
			"/sensor_motor_revolutions", 1, &Parking::motorValues, this);

	ros::Duration(1).sleep();
}

int init(Parking park)
{
	try
	{
		park.init();
	}
	catch (std::exception& error)
	{
		ROS_ERROR("Error: %s\n", error.what());
		return -1;
	}
	catch (...)
	{
		ROS_ERROR("Unknown Error\n\r");
		return -1;
	}

	return 0;
}

void shake(Parking park, MoveToGap::driveData data)
{
	data.angle.data = 0;
	park.angle_pub.publish(data.angle);
	ros::Duration(0.4).sleep();

	data.angle.data = -90;
	park.angle_pub.publish(data.angle);
	ros::Duration(0.4).sleep();

	data.angle.data = 0;
	park.angle_pub.publish(data.angle);
	ros::Duration(0.4).sleep();

	ROS_INFO("[PAR]: Parking gestartet");
}

void driveToGap(Parking park, MoveToGap::driveData data, MoveToGap driver)
{
	if (park.parallel.driveEnable())
	{
		data = driver.moveToGap(park.g_laser, park.distanceSide,
				park.distanceBack, park.gapcal.getGapDistance(),
				park.voltage, park.motorRevolutions);

		if (data.speed.data == 0)
		{
			park.ParkingController_ = true;
		}
	}
	else
	{
		data.speed.data = 0;
	}
	park.angle_pub.publish(data.angle);
	park.speed_pub.publish(data.speed);
}

void driveIntoGap(Parking park)
{
	DriveIntoGap::twoInts twoInts = park.driveIntoGap.drive(
			park.g_laser, BESTGAPLENGTH, park.distanceBack,
			park.distanceSide, park.motorRevolutions, park.voltage);
	park.intoGapAngle = twoInts.angle;
	park.intoGapSpeed = twoInts.speed;

	std_msgs::Int8 angle;
	angle.data = park.intoGapAngle;

	std_msgs::Int8 speed;
	speed.data = park.intoGapSpeed;

	park.angle_pub.publish(angle);
	park.speed_pub.publish(speed);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");

	//-- initialisiere die Kommunikationsstruktur von Parking

	Parking park;
	if(init(park) == -1)
		return -1;

	ros::spinOnce();

	//-- zittere ein wenig mit den Rädern, um den Start anzuzeigen

	MoveToGap::driveData data;
	MoveToGap driver;

	shake(park, data);

	//-- die eigentliche Handlung:

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok)
	{
		if (!park.ParkingController_)
		{
			driveToGap(park, data, driver);
		}
		else if (park.ParkingController_)
		{
			driveIntoGap(park);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_WARN("[PAR]: Parking beendet");
	return 0;
}
