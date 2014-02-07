/*
 * Orientation.cpp
 *
 *  Created on: 07.02.2014
 *      Author: jsabsch
 */

#include "Orientation.h"

Orientation::Orientation()
{
	orientX = 0;
	orientY = 0;
	orientZ = 0;
}

Orientation::~Orientation()
{

}



//timespan in nsec
void Orientation::updateOrientation(float velX, float velY, float velZ, float timeSpan)
{
	velX = calculateOrient(velX, timeSpan);
	velY = calculateOrient(velY, timeSpan);
	velZ = calculateOrient(velZ, timeSpan);
}

float Orientation::calculateOrient(float vel, float timeSpan)
{
	ROS_INFO("[ORI] vel: %f time: %f", vel, timeSpan);
	return vel * timeSpan;
}

float Orientation::orientationX()
{
	return orientX;
}

float Orientation::orientationY()
{
	return orientY;
}

float Orientation::orientationZ()
{
	return orientZ;
}
