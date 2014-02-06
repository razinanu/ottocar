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
		int odometry)
{
	driveData result;

	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = 8;	//todo 8

//	ROS_INFO("distanceTOGap: %2.4f | IR: %2.4f", distanceToGap, dataIRside);

	switch (mode)
	{
		//auf eine Entfernung zur Luecke warten
	case 0:
	{
		if (distanceToGap > 0 && distanceToGap < 1.0) //todo ab wann den Wert akzeptieren?
		{
			ROS_INFO(
					"[MTG]: distanceTOGap: %2.4f | IR: %2.4f", distanceToGap, dataIRside);
			distanceToDrive = distanceToGap;
			lastOdometry = odometry;
			mode = 1;
		}
		break;
	}
		//bis zur Mitte der Luecke fahren
	case 1:
	{
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
		break;
	}
		//auf das Ende der Luecke warten, bis der  IR-Sensor den Karton sieht
	case 2:
	{
		if (dataIRside < 0.2)
//		if (laser.ranges[511] < 0.25)
		{
			gapBegin = ros::Time::now();
			lastOdometry = odometry;
			mode = 3;
		}
		break;
	}
		//x cm hinter der Luecke anhalten
	case 3:
	{
		if (drivenM(odometry) > 0.20)
		{
			ROS_INFO("[MTG]: hinter der Luecke angehalten: %2.4f",drivenM(odometry));
			result.speed.data = 0;
			mode = 4;
		}
		break;
	}
		//fertig
	default:
	{
		result.angle.data = STRAIGHTFORWARD;
		result.speed.data = 0;
		break;
	}
	}

	return result;
}
