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
	currentDrivingDirection = back;
	gapSize = -1;
	minimalLaserDistance = -1;
	mode = 3;
}

DriveIntoGap::~DriveIntoGap()
{

}

DriveIntoGap::twoInts DriveIntoGap::drive(sensor_msgs::LaserScan laser,
		float gapSize, float distanceBack, float distanceSide)
{
//	this->gapSize = gapSize;
//	parkingController.LaserScanParkControll(laser);
//	this->minimalLaserDistance = parkingController.getMinimalDistance();

	//test:
	//enoughSpaceInTheBack();
	//enoughSpaceOnTheFront();

	twoInts speedAndAngle; //[1]angle, [0]speed

	//licht START
	switch (mode)
	{
	case 3:
	{
		lastTime = ros::Time::now();
		mode = 4;
		break;
	}
	case 4:
	{
		//warten
		if ((lastTime + ros::Duration(1)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 5;
		}
		speedAndAngle.angle = -21;
		speedAndAngle.speed = 0;
		break;
	}
	case 5:
	{
		//rückwärts in die Parklücke einfahren 1. Teil
		if ((lastTime + ros::Duration(1.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 6;
		}
		speedAndAngle.angle = RIGHT_MAX;
		speedAndAngle.speed = 8;
		break;
	}
	case 6:
	{
		//warten
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 7;
		}
		speedAndAngle.speed = 0;
		break;
	}
	case 7:
	{
		//rückwärts in die Parklücke einfahren 2. Teil
		if (distanceBack < 11)
		{
			lastTime = ros::Time::now();
			mode = 8;
		}
		speedAndAngle.angle = LEFT_MAX;
		speedAndAngle.speed = 7;
		break;
	}
	case 8:
	{
		//warten
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 9;
		}
		speedAndAngle.speed = 0;
		break;
	}
	case 9:
	{
		//vorwärts mit laser & zeit fahren
		for (int i = 255 - 50; i <= 255 + 50; i++)
		{
			if ((laser.ranges[i] < 0.2) || (lastTime + ros::Duration(1.0)) < ros::Time::now())		//todo timeout sinnvoll?
			{
				lastTime = ros::Time::now();
				mode = 10;
			}
		}

		//kurz vorwärts fahren
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 10;
//		}

		speedAndAngle.angle = RIGHT_MAX;
		speedAndAngle.speed = - 7;
		break;
	}
	case 10:
	{
		//warten
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 11;
		}
		speedAndAngle.angle = -21;
		speedAndAngle.speed = 0;
		break;
	}

	case 11:
	{
		//wieder kurz zurück, um mindestabstand einzuhalten
		if ((lastTime + ros::Duration(0.4)) < ros::Time::now() && distanceBack > 11)
		{
			lastTime = ros::Time::now();
			mode = 12;
		}
		speedAndAngle.angle = -21;
		speedAndAngle.speed =  7;
		break;
	}

	default:
	{
		speedAndAngle.angle = -21;
		speedAndAngle.speed = 0;
		break;
	}
	}

//licht ENDE

//	if(currentDrivingDirection == back)// && enoughSpaceInTheBack())
//	{
//		speedAndAngle = backward();
//	}
//	else if(enoughSpaceOnTheFront())
//	{
//		currentDrivingDirection = forth;
//		speedAndAngle = forward();
//	}
//	else
//	{
//		currentDrivingDirection = back;
//		speedAndAngle = backward();
//	}

	return speedAndAngle;
}

bool DriveIntoGap::enoughSpaceInTheBack()
{
//eine Methode von Simone aufrufen
	if (gapSize == -1 || minimalLaserDistance == -1
			|| minimalLaserDistance + SECUREDISTANCE_BACK > gapSize)
	{
		ROS_INFO("not enough space - back");
		return false;
	}

	return true;

//temp:
//return enoughSpaceBackSimulation();
}

bool DriveIntoGap::enoughSpaceOnTheFront()
{
//eine Methode von Simone aufrufen
	if (minimalLaserDistance < SECUREDISTANCE_FRONT) //10 cm bis zur Spitze, dann 5 cm Sicherheitsabstand
	{
		ROS_INFO("not enough space - front");
		return false;
	}

	return true;

//temp:
//return enoughSpaceFrontSimulation();
}

bool DriveIntoGap::firstHalf()
{
	return parkingController.rightTurn();

//temp:
//return firstHalfSimulation();

	return true;
}

bool DriveIntoGap::isStraight()
{
//eine Methode von Simone aufrufen
	return false;
}

DriveIntoGap::twoInts DriveIntoGap::backward()
{
//Todo statt konstanter Werte einen variablen Geschwindigkeit

	twoInts speedAndAngle;

	if (firstHalf()) //solange er sich in der vorderen Hälfte befindet, ...
	{
		ROS_ERROR("[DriveIntoGap] first half");
		//... soll der Roboter maximal nach rechts einschlagen und rückwärts fahren
		speedAndAngle.speed = PARKINGSPEED;
		speedAndAngle.angle = RIGHT_MAX;
	}
	else //ab der hinteren Hälfte muss er wieder zurücksetzen
	{
		ROS_ERROR("[DriveIntoGap] second half");
		if (isStraight()) //falls er bereits parallel zur Straße steht, setzt er noch ein kleines Stück zurück (muss er?)
		{
			speedAndAngle.speed = PARKINGSPEED;
			speedAndAngle.angle = STRAIGHTFORWARD;
		}
		else //ansonsten dreht er sich maximal nach links und fährt rückwärts
		{
			speedAndAngle.speed = PARKINGSPEED;
//			speedAndAngle.speed = 0;
			speedAndAngle.angle = LEFT_MAX;
		}
	}

	return speedAndAngle;
}

DriveIntoGap::twoInts DriveIntoGap::forward()
{
//Todo nicht nur vorwärts, sondern Ausrichtung korrigieren
//Todo Geschwindigkeit variabel machen

	twoInts speedAndAngle;
	speedAndAngle.speed = -PARKINGSPEED;
	speedAndAngle.angle = STRAIGHTFORWARD;

	return speedAndAngle;
}
