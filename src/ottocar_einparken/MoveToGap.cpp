/*
 * DriveToGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#include "MoveToGap.h"

MoveToGap::MoveToGap()
{
	cartonSeen = false;
	distanceGot = false;
	gapBegin = ros::Time::now() - ros::Duration(2);
	mode = 0;
	lastOdometry = 0;
	distanceToDrive = 0;

	lastTimeBlinkerChange = ros::Time::now() - ros::Duration(2);
	blinkerOn = false;
}

MoveToGap::~MoveToGap()
{
}

float MoveToGap::drivenM(int odometry)
{
	//gibt die gefahrenen Distanz in m vom letzten Zeitpunkt unabhaengig von der Fahrtrichtung zurueck
	return abs(abs(lastOdometry) - abs(odometry)) / REVOLUTIONS_PER_M;
}

MoveToGap::driveData MoveToGap::moveToGap(sensor_msgs::LaserScan laser,
		float dataIRside, float dataIRback, float distanceToGap, float voltage,
		int odometry, float gapSize)
{
	driveData result;

	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = SPEED_PARKING;	//todo 8

	switch (mode)
	{
	case 0:
		waitForDistance(distanceToGap, odometry);
		break;

	case 1:
		result = driveFirstHalf(odometry, dataIRside, result.speed.data, result.angle.data);
		break;

	case 2:
		result = driveSecondHalf(dataIRside, odometry, result.speed.data, result.angle.data);
		break;

	case 3:
		result = positioning(odometry, result.speed.data, result.angle.data, gapSize);
		break;


	default:	//fertig
		result.angle.data = STRAIGHTFORWARD;
		result.speed.data = 0;
		result.led1.data = 105;
		result.led2.data = 106;
		result.led3.data = 107;
		break;
	}

	return result;
}

//warte auf eine Distanz
void MoveToGap::waitForDistance(float distanceToGap, int odometry)
{
	if (distanceToGap > 0 && distanceToGap < 1.0) //todo ab wann den Wert akzeptieren?
	{
		distanceToDrive = distanceToGap;
		lastOdometry = odometry;
		mode = 1;
	}
}

//fahre bis zur Mitte der Lücke
MoveToGap::driveData MoveToGap::driveFirstHalf(int odometry, float dataIRside, int speed, int angle)
{
	//bei dem Rückgabewert nur die LEDs betrachten!
	driveData result;

	if (drivenM(odometry) > distanceToDrive - 0.1)
	{
		if (dataIRside > 0.2)
			ROS_INFO(
					"[MTG]: In der Mitte neben der Luecke: %2.2f", dataIRside);
		else
			ROS_WARN(
					"[MTG]: In der Mitte neben der Luecke: %2.2f", dataIRside);

		mode = 2;
	}

	if (!blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		result.led1.data = 100;
		result.led2.data = 107;
		blinkerOn = true;
		lastTimeBlinkerChange = ros::Time::now();
	}
	else if (!blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) > ros::Time::now())
	{
		result.led1.data = 0;
		result.led2.data = 7;
	}

	if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		result.led1.data = 0;
		result.led2.data = 7;
		blinkerOn = false;
		lastTimeBlinkerChange = ros::Time::now();
	}
	else if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) > ros::Time::now())
	{
		result.led1.data = 100;
		result.led2.data = 107;
	}
	result.angle.data = angle;
	result.speed.data = speed;
	return result;
}

//auf das Ende der Luecke warten, bis der  IR-Sensor den Karton sieht
MoveToGap::driveData MoveToGap::driveSecondHalf(float dataIRside, int odometry, int speed, int angle)
{
	//bei dem Rückgabewert nur die LEDs betrachten!
	driveData result;

	if (dataIRside < 0.2)
	{
		gapBegin = ros::Time::now();
		lastOdometry = odometry;
		mode = 3;
	}

	if (!blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		result.led1.data = 100;
		result.led2.data = 107;
		blinkerOn = true;
		lastTimeBlinkerChange = ros::Time::now();
	}
	else if (!blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) > ros::Time::now())
	{
		result.led1.data = 0;
		result.led2.data = 7;
	}

	if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		result.led1.data = 0;
		result.led2.data = 7;
		blinkerOn = false;
		lastTimeBlinkerChange = ros::Time::now();
	}
	else if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) > ros::Time::now())
	{
		result.led1.data = 100;
		result.led2.data = 107;
	}

	result.angle.data = angle;
	result.speed.data = speed;
	return result;
}

//x cm hinter der Luecke anhalten
MoveToGap::driveData MoveToGap::positioning(int odometry, int speed, int angle, float gapSize)
{
	driveData result;

	if (drivenM(odometry) > 0.17)
	{
		ROS_INFO("[MTG]: hinter der Luecke angehalten: %2.4f",drivenM(odometry));
		mode = 4;
		result.speed.data = 0;
		result.angle.data = angle;
		result.led1.data = 105;
		result.led2.data = 106;
		result.led3.data = 108;
		return result;
	}

	if (gapSize == (float) 0.8)
	{
		if (drivenM(odometry) > 0.15)
		{
			ROS_INFO("[MTG]: hinter Luecke 80cm angehalten: %2.4f",drivenM(odometry));
			mode = 4;
			result.speed.data = 0;
			result.angle.data = angle;
			result.led1.data = 105;
			result.led2.data = 106;
			result.led3.data = 108;
			return result;
		}
	}

	result.speed.data = speed;
	result.angle.data = angle;

	if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		result.led1.data = 0;
		result.led2.data = 7;
		blinkerOn = false;
		lastTimeBlinkerChange = ros::Time::now();
	}
	else if (blinkerOn && lastTimeBlinkerChange + ros::Duration(0.5) > ros::Time::now())
	{
		result.led1.data = 100;
		result.led2.data = 107;
	}
	return result;
}

