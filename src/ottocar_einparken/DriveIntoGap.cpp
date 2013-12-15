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
}

DriveIntoGap::~DriveIntoGap()
{

}

DriveIntoGap::twoInts DriveIntoGap::drive(sensor_msgs::LaserScan laser, float gapSize)
{
	this->gapSize = gapSize;
	parkingController.LaserScanParkControll(laser);
	this->minimalLaserDistance = parkingController.getMinimalDistance();

	//test:
//	enoughSpaceInTheBack();
//	enoughSpaceOnTheFront();


	twoInts speedAndAngle;	//[1]angle, [0]speed

	if(currentDrivingDirection == back)// && enoughSpaceInTheBack())
	{
		speedAndAngle = backward();
	}
	else if(enoughSpaceOnTheFront())
	{
		currentDrivingDirection = forth;
		speedAndAngle = forward();
	}
	else
	{
		currentDrivingDirection = back;
		speedAndAngle = backward();
	}

	return speedAndAngle;
}

bool DriveIntoGap::enoughSpaceInTheBack()
{
	//eine Methode von Simone aufrufen
	if(gapSize == -1 || minimalLaserDistance == -1 || minimalLaserDistance + SECUREDISTANCE_BACK > gapSize)
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
	if(minimalLaserDistance < SECUREDISTANCE_FRONT)//10 cm bis zur Spitze, dann 5 cm Sicherheitsabstand
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

	if(firstHalf())	//solange er sich in der vorderen Hälfte befindet, ...
	{
		//... soll der Roboter maximal nach rechts einschlagen und rückwärts fahren
		speedAndAngle.x = PARKINGSPEED;
		speedAndAngle.y = RIGHT_MAX;
	}
	else	//ab der hinteren Hälfte muss er wieder zurücksetzen
	{
		if(isStraight())	//falls er bereits parallel zur Straße steht, setzt er noch ein kleines Stück zurück (muss er?)
		{
			speedAndAngle.x = PARKINGSPEED;
			speedAndAngle.y = STRAIGHTFORWARD;
		}
		else	//ansonsten dreht er sich maximal nach links und fährt rückwärts
		{
//			speedAndAngle.x = PARKINGSPEED;
			speedAndAngle.x = 0;
			speedAndAngle.y = LEFT_MAX;
		}
	}


	return speedAndAngle;
}

DriveIntoGap::twoInts DriveIntoGap::forward()
{
	//Todo nicht nur vorwärts, sondern Ausrichtung korrigieren
	//Todo Geschwindigkeit variabel machen

	twoInts speedAndAngle;
	speedAndAngle.x = - PARKINGSPEED;
	speedAndAngle.y = STRAIGHTFORWARD;

	return speedAndAngle;
}
