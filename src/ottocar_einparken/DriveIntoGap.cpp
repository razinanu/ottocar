/*
 * DriveIntoGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#include "DriveIntoGap.h"
#include "ros/ros.h"
#include "ConstPark.h"


DriveIntoGap::DriveIntoGap()
{

}

DriveIntoGap::~DriveIntoGap()
{

}

DriveIntoGap::twoInts DriveIntoGap::drive()
{
	twoInts speedAndAngle;	//[0]angle, [1]speed

	if(enoughSpaceInTheBack())
	{
		speedAndAngle = backward();
	}
	else
	{
		speedAndAngle = forward();
	}

	return speedAndAngle;
}

bool DriveIntoGap::enoughSpaceInTheBack()
{
	//eine Methode von Simone aufrufen
	return true;
}

bool DriveIntoGap::firstHalf()
{
	//eine Methode von Simone aufrufen
	return true;
}

bool DriveIntoGap::isStraight()
{
	return false;
}

DriveIntoGap::twoInts DriveIntoGap::backward()
{
	//Todo statt konstanter Werte einen variablen Geschwindigkeit

	twoInts speedAndAngle;

	if(firstHalf())	//solange er sich in der vorderen Hälfte befindet, ...
	{
		//... soll der Roboter maximal nach rechts einschlagen und rückwärts fahren
		speedAndAngle.x = 8;
		speedAndAngle.y = RIGHT_MAX;
	}
	else	//ab der hinteren Hälfte muss er wieder zurücksetzen
	{
		if(isStraight())	//falls er bereits parallel zur Straße steht, setzt er noch ein kleines Stück zurück (muss er?)
		{
			speedAndAngle.x = 8;
			speedAndAngle.y = STRAIGHTFORWARD;
		}
		else	//ansonsten dreht er sich maximal nach links und fährt rückwärts
		{
			speedAndAngle.x = 8;
			speedAndAngle.y = LEFT_MAX;
		}
	}

	return speedAndAngle;
}

DriveIntoGap::twoInts DriveIntoGap::forward()
{
	//Todo nicht nur vorwärts, sondern Ausrichtung korrigieren

	twoInts speedAndAngle;
	speedAndAngle.x = 8;
	speedAndAngle.y = STRAIGHTFORWARD;

	return speedAndAngle;
}
