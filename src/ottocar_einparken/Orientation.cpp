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



//timespan in msec
void Orientation::updateOrientation(float velX, float velY, float velZ, int timeSpan)
{
	orientX += calculateOrient(velX, timeSpan);
	orientY += calculateOrient(velY, timeSpan);
	orientZ += calculateOrient(velZ, timeSpan);
}

float Orientation::calculateOrient(float vel, int timeSpan)
{
	ROS_INFO("[ORI] vel: %f time: %i erg: %f", vel, timeSpan, (vel * timeSpan));
	return vel * (timeSpan);
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
