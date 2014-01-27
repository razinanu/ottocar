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
		float gapSize, float distanceBack, float distanceSide, int odometrie, float voltage)
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
	//initialisieren
	case 3:
	{
		lastTime = ros::Time::now();
		mode = 4;
		break;
	}

	//warten
	case 4:
	{
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			lastOdometrie = odometrie;
			mode = 5;
			timeToDrive = (0.8 / 0.45 /*calculateSpeed10(voltage) */);
//			ROS_INFO("lastodo: %d", odometrie);
		}
		speedAndAngle.angle = STRAIGHTFORWARD;
		speedAndAngle.speed = 0;
		break;
	}

	//rückwärts in die Parklücke einfahren 1. Teil
	case 5:
	{
		if (gapSize == (float) 0.6)
		{
//			ROS_INFO("odo: %d | lastodo: %d | d: %d",odometrie, lastOdometrie, abs( abs(lastOdometrie) - abs(odometrie)));
//			if (abs( abs(lastOdometrie) - abs(odometrie)) > 37) //((lastTime + ros::Duration(1.5)) < ros::Time::now())
//			{
//				lastTime = ros::Time::now();
//				mode = 6;
//			}

			if ((lastTime + ros::Duration(timeToDrive)) < ros::Time::now())
			{
				lastTime = ros::Time::now();
				mode = 6;	//todo 6
			}

		}

		else if (gapSize == (float) 0.7)
		{
			if ((lastTime + ros::Duration(1.6)) < ros::Time::now())
			{
				lastTime = ros::Time::now();
				mode = 6;
			}
		}

		else if (gapSize == (float) 0.8)
		{
			if ((lastTime + ros::Duration(1.6)) < ros::Time::now())
			{
				lastTime = ros::Time::now();
				mode = 6;
			}
		}
		else
		{
			ROS_INFO("gapSize: %2.32f", gapSize);
			ROS_INFO("gapSize: %2.32f", (float) 0.6);
			mode = 25;
		}

		speedAndAngle.angle = RIGHT_MAX;
		speedAndAngle.speed = 10;
		break;
	}

	//warten
	case 6:
	{
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 7;
		}
		speedAndAngle.speed = 0;
		break;
	}

	//rückwärts in die Parklücke einfahren 2. Teil mit timeout
	case 7:
	{
		if (distanceBack < 0.10 || (lastTime + ros::Duration(3)) < ros::Time::now())
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			lastTime = ros::Time::now();
			mode = 8;
		}
		ROS_INFO("[DIG]: in die Luecke fahren: %2.4f", distanceBack);
		speedAndAngle.angle = LEFT_MAX;
		speedAndAngle.speed = 9;
		break;
	}

	//warten
	case 8:
	{
		if ((lastTime + ros::Duration(0.3)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 9;
		}
		speedAndAngle.speed = 0;
		break;
	}

	//warten und Lenkung einschlagen
	case 9:
	{
		if ((lastTime + ros::Duration(0.2)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 10;
		}
		speedAndAngle.angle = RIGHT_MAX;
		speedAndAngle.speed = 0;
		break;
	}

	//vorwärts mit laser & zeit fahren
	case 10:
	{
		for (int i = 255 - 50; i <= 255 + 50; i++)
		{
			if ((laser.ranges[i] < 0.17) || (lastTime + ros::Duration(1.0)) < ros::Time::now())		//todo timeout sinnvoll?
			{
				ROS_INFO("[DIG]: nach vorne gefahren: %2.4f | i: %d", laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				break;
			}
		}
		speedAndAngle.angle = RIGHT_MAX;
		speedAndAngle.speed = - 9;
		break;
	}

	//warten
	case 11:
	{
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			lastTime = ros::Time::now();
			mode = 12;
		}
		speedAndAngle.angle = STRAIGHTFORWARD;
		speedAndAngle.speed = 0;
		break;
	}

	//wieder kurz zurück, um mindestabstand einzuhalten
	case 12:
	{
		if ((lastTime + ros::Duration(0.4)) < ros::Time::now() || distanceBack < 11)
		{
			ROS_INFO("[DIG]: kurzes Stueck zurueckgesetzt: %2.4f",distanceBack);
			lastTime = ros::Time::now();
			mode = 13;
		}
		speedAndAngle.angle = STRAIGHTFORWARD;
		speedAndAngle.speed =  8;
		break;
	}

	default:
	{
		speedAndAngle.angle = STRAIGHTFORWARD;
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

float DriveIntoGap::calculateSpeed10(float voltage)
{
	float result;
	result = 0.05083733 * voltage - 0.3547296;
	ROS_INFO("[DIG]: voltage: %2.4f | speed: %2.4f", voltage, result);
	return result;
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
