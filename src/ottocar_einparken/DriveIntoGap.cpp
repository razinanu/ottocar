/*
 * DriveIntoGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#include "DriveIntoGap.h"
#include "ConstPark.h"

DriveIntoGap::DriveIntoGap()
{
	mode = 3;
	lastOdometry = 0;
	SPEED = SPEED_PARKING;
}

DriveIntoGap::~DriveIntoGap()
{

}

float DriveIntoGap::drivenM(int odometry)
{
	//gibt die gefahrenen Distanz in m vom letzten Zeitpunkt unabhaengig von der Fahrtrichtung zurueck
	return abs((lastOdometry) - (odometry)) / REVOLUTIONS_PER_M;
}

DriveIntoGap::twoInts DriveIntoGap::drive(sensor_msgs::LaserScan laser,
		float gapSize, float distanceBack, float distanceSide, int odometry, float voltage)
{
	twoInts speedAndAngle; //[1]angle, [0]speed
	speedAndAngle.angle = STRAIGHTFORWARD;
	speedAndAngle.speed = 0;

	switch (mode)
	{
	case 3:
		speedAndAngle = init();
		break;

	case 4:
		speedAndAngle = wait1(odometry);
		break;

	case 5:
		speedAndAngle = back1(gapSize, odometry);
		break;

	case 6:
		speedAndAngle = wait2(odometry);
		break;

	case 7:
		speedAndAngle = back2(laser, distanceBack, odometry, gapSize);
		break;

	case 8:
		speedAndAngle = wait3();
		break;

	case 9:
		speedAndAngle = waitTurn(odometry);
		break;

	case 10:
		speedAndAngle = forwards(laser, gapSize, odometry);
		break;

	case 11:
		speedAndAngle = wait4(odometry);
		break;

	case 12:
		speedAndAngle = backLast(distanceBack, odometry);
		break;

	default:
		speedAndAngle.angle = STRAIGHTFORWARD;
		speedAndAngle.speed = 0;
		break;
	}

	return speedAndAngle;
}

float DriveIntoGap::calculateSpeed10(float voltage)
{
	float result;
	result = 0.05083733 * voltage - 0.3547296;
	ROS_INFO("[DIG]: voltage: %2.4f | speed: %2.4f", voltage, result);
	return result;
}

DriveIntoGap::twoInts DriveIntoGap::init()
{
	lastTime = ros::Time::now();
	mode = 4;

	twoInts ti;
	ti.angle = STRAIGHTFORWARD;
	ti.speed = 0;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::wait1(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 5;
	}

	twoInts ti;
	ti.angle = STRAIGHTFORWARD;
	ti.speed = 0;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::back1(float gapSize, int odometry)
{
	//60 cm
	if (gapSize == (float) 0.6)
	{
		if (drivenM(odometry) > 0.45)
		{
			ROS_INFO("[DIG]: 1. Teil 60cm Rueckwaerts: %2.4f", drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
		}

	}

	//70 cm
	else if (gapSize == (float) 0.7)
	{
		if (drivenM(odometry) > 0.43)
		{
			ROS_INFO("[DIG]: 1. Teil 70cm Rueckwaerts: %2.4f", drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
		}
	}

	//80 cm
	else if (gapSize == (float) 0.8)
	{
		if (drivenM(odometry) > 0.45)
		{
			ROS_INFO("[DIG]: 1. Teil 80cm Rueckwaerts: %2.4f", drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
		}
	}
	else
	{
		ROS_INFO("gapSize: %2.32f", gapSize);
		mode = 25;
	}

	twoInts speedAndAngle;
	speedAndAngle.angle = RIGHT_MAX;
	speedAndAngle.speed = - SPEED;

	return speedAndAngle;
}

DriveIntoGap::twoInts DriveIntoGap::wait2(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 7;
	}
	twoInts ti;
	ti.speed = 0;
	ti.angle = STRAIGHTFORWARD;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::back2(const sensor_msgs::LaserScan laser, float distanceBack, int odometry, float gapSize)
{
	//60 cm
	if (gapSize == (float) 0.6)
	{
//		if (distanceBack < 0.12) //todo 0.12
//		{
//			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
//			ROS_INFO("[DIG]: --> gefahrene Strecke: %2.4f", drivenM(odometry));
//			lastTime = ros::Time::now();
//			mode = 8;
//		}

		float minValue = 1;
		for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
		{
			if (laser.ranges[i] < minValue)
			{
				minValue = laser.ranges[i];
			}
		}

		if (minValue > 0.14 && minValue < 1 && drivenM(odometry) > 0.22)
		{
			ROS_WARN(
					"[DIG]: in Luecke mit Laserscanner gefahren: %2.4f", minValue);
			lastTime = ros::Time::now();
			mode = 8;
		}

		if ((lastTime + ros::Duration(5)) < ros::Time::now()) //todo 0.12
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			ROS_WARN(
					"[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue, drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 8;
		}
	}

	//70 cm
	if (gapSize == (float) 0.7)
	{
//		if (distanceBack < 0.16) //todo 0.12
//		{
//			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
//			ROS_INFO("[DIG]: --> gefahrene Strecke: %2.4f", drivenM(odometry));
//			lastTime = ros::Time::now();
//			mode = 8;
//		}

		float minValue = 1;
		for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
		{
			if (laser.ranges[i] < minValue)
			{
				minValue = laser.ranges[i];
			}
		}

		if (minValue > 0.17 && minValue < 1 && drivenM(odometry) > 0.25 )
		{
			ROS_WARN("[DIG]: in Luecke mit Laserscanner gefahren: %2.4f", minValue);
			lastTime = ros::Time::now();
			mode = 8;
		}

		if ((lastTime + ros::Duration(5)) < ros::Time::now()) //todo 0.12
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			ROS_WARN("[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue, drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 8;
		}

//		ROS_INFO("odo: %2.4f | minValue: %2.4f", drivenM(odometry), minValue);
	}

	//80 cm
	if (gapSize == (float) 0.8)
	{
		//		if (distanceBack < 0.16) //todo 0.12
		//		{
		//			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
		//			ROS_INFO("[DIG]: --> gefahrene Strecke: %2.4f", drivenM(odometry));
		//			lastTime = ros::Time::now();
		//			mode = 8;
		//		}

				float minValue = 1;
				for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
				{
					if (laser.ranges[i] < minValue)
					{
						minValue = laser.ranges[i];
					}
				}

				if (minValue > 0.18 && minValue < 1 && drivenM(odometry) > 0.25 )
				{
					ROS_WARN("[DIG]: in Luecke mit Laserscanner gefahren: %2.4f", minValue);
					lastTime = ros::Time::now();
					mode = 8;
				}

				if ((lastTime + ros::Duration(5)) < ros::Time::now()) //todo 0.12
				{
					ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
					ROS_WARN("[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue, drivenM(odometry));
					lastTime = ros::Time::now();
					mode = 8;
				}
	}

	twoInts ti;
	ti.angle = LEFT_MAX;
	ti.speed = - SPEED;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::wait3()
{
	if ((lastTime + ros::Duration(0.3)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		mode = 9;
	}
	twoInts ti;
	ti.speed = 0;
	ti.angle = STRAIGHTFORWARD;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::waitTurn(int odometry)
{
	if ((lastTime + ros::Duration(0.2)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 10;
	}

	twoInts ti;
	ti.angle = RIGHT_MAX;
	ti.speed = 0;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::forwards(const sensor_msgs::LaserScan laser, float gapSize, int odometry)
{
	twoInts ti;
	ti.angle = STRAIGHTFORWARD;
	ti.speed = 0;

	if ((lastTime + ros::Duration(2.0)) < ros::Time::now())	//todo timeout sinnvoll?
	{
		ROS_WARN("[DIG]: nach vorne gefahren: timeOut");
		lastTime = ros::Time::now();
		mode = 11;
		return ti;
	}

	//60cm Lücke
	if (gapSize == (float) 0.6)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
		{
			if (laser.ranges[i] < 0.13)
			{
				ROS_INFO(
						"[DIG]: nach vorne gefahren: %2.4f | i: %d", laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				return ti;
			}
		}
		ti.angle = 45;
	}

	//70cm Lücke
	if (gapSize == (float) 0.7)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
		{
			if (laser.ranges[i] < 0.17)
			{
				ROS_INFO(
						"[DIG]: nach vorne gefahren: %2.4f | i: %d", laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				return ti;
			}
		}
		ti.angle = 25;
	}

	//80 cm
	if (gapSize == (float) 0.8)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++) //todo vllt. Odometrie
		{
			if (laser.ranges[i] < 0.19)
			{
				ROS_INFO(
						"[DIG]: nach vorne gefahren: %2.4f | i: %d", laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				return ti;
			}
		}
		ti.angle = 25;
	}


	ti.speed = SPEED;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::wait4(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 12;
	}

	twoInts ti;
	ti.angle = STRAIGHTFORWARD;
	ti.speed = 0;

	return ti;
}

DriveIntoGap::twoInts DriveIntoGap::backLast(float distanceBack, int odometry)
{
	//todo das auto faehrt nicht an

	if (drivenM(odometry) > 0.15 || distanceBack < 11)
	{
		ROS_INFO("[DIG]: kurzes Stueck zurueckgesetzt: %2.4f",distanceBack);
		ROS_INFO("[DIG]: gefahrene Odometrie: %2.4f",drivenM(odometry));
		lastTime = ros::Time::now();
		mode = 13;
	}
	twoInts ti;
	ti.angle = STRAIGHTFORWARD;
	ti.speed =  - SPEED;

	return ti;
}

