/*
 * DriveIntoGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#include "DriveIntoGap.h"
#include "ConstPark.h"


DriveIntoGap::DriveIntoGap() : firstH(2), spaceback(4), spacefront(8)
{
	currentDrivingDirection = back;
	lastMark = currentTime.now();
	minimalLaserDistance = -1;
	gapSize = -1;
}

DriveIntoGap::~DriveIntoGap()
{

}

DriveIntoGap::twoInts DriveIntoGap::drive(float minimalLaserDistance, float gapSize)
{
	this->minimalLaserDistance = minimalLaserDistance;
	this->gapSize = gapSize;

	twoInts speedAndAngle;	//[0]angle, [1]speed

	if(currentDrivingDirection == back && enoughSpaceInTheBack())
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
	}

	return speedAndAngle;
}

bool DriveIntoGap::enoughSpaceInTheBack()
{
	//eine Methode von Simone aufrufen
	if(gapSize == -1 || minimalLaserDistance == -1 || minimalLaserDistance + 35 < gapSize)
	{
		return false;
	}

	return true;

	//temp:
	//return enoughSpaceBackSimulation();
}

bool DriveIntoGap::enoughSpaceOnTheFront()
{
	//eine Methode von Simone aufrufen
	if(minimalLaserDistance < 15)	//10 cm bis zur Spitze, dann 5 cm Sicherheitsabstand
	{
		return false;
	}

	return true;

	//temp:
	//return enoughSpaceFrontSimulation();
}

bool DriveIntoGap::firstHalf()
{
	//eine Methode von Simone aufrufen

	//temp:
	return firstHalfSimulation();

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
		speedAndAngle.x = 4;
		speedAndAngle.y = RIGHT_MAX;
	}
	else	//ab der hinteren Hälfte muss er wieder zurücksetzen
	{
		if(isStraight())	//falls er bereits parallel zur Straße steht, setzt er noch ein kleines Stück zurück (muss er?)
		{
			speedAndAngle.x = 4;
			speedAndAngle.y = STRAIGHTFORWARD;
		}
		else	//ansonsten dreht er sich maximal nach links und fährt rückwärts
		{
			speedAndAngle.x = 4;
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
	speedAndAngle.x = -4;
	speedAndAngle.y = STRAIGHTFORWARD;

	return speedAndAngle;
}



//temp:



bool DriveIntoGap::enoughSpaceBackSimulation()
{
	if(currentTime.now() > lastMark + spaceback)
	{
		return false;
	}

	return true;
}

bool DriveIntoGap::enoughSpaceFrontSimulation()
{
	if(currentTime.now() > lastMark + spacefront)
	{
		lastMark = currentTime.now();
		return false;
	}

	return true;
}

bool DriveIntoGap::firstHalfSimulation()
{
	if(currentTime.now() > lastMark + firstH)
	{
		return false;
	}

	return true;
}
